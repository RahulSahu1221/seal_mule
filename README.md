<div align='center'>

# SEAL MULE

---

### *An Autonomous Kitting & Data-Mule Robot with PLC-Coordinated Part Verification*

A simulation-stage automation project combining a PLC-controlled safety/sequencing layer with an autonomous mobile robot (AMR) that verifies parts before transporting them, and doubles as a wireless **"data mule"** for battery-free, vibration-powered sensor nodes placed along its route.

Built as a bridge between **ladder-logic PLC control** and **practical embedded/robotics engineering**.

<br>

<img src='https://img.shields.io/badge/Status-Code%20Complete%20%7C%20Simulation%20Pending-c96a28?style=for-the-badge' />
<img src='https://img.shields.io/badge/Architecture-Dual%20Controller-0b79c8?style=for-the-badge' />
<img src='https://img.shields.io/badge/PLC-Ladder%20Logic%20(LDmicro)-8a0ea8?style=for-the-badge' />
<img src='https://img.shields.io/badge/MCU-Arduino%20Mega%202560-d9534f?style=for-the-badge' />

<img src='https://img.shields.io/badge/Simulation-Proteus%208%20Professional-008b8b?style=for-the-badge' />
<img src='https://img.shields.io/badge/Wireless-NRF24L01-808080?style=for-the-badge' />
<img src='https://img.shields.io/badge/Verification-RFID%20(MFRC522)-7aa300?style=for-the-badge' />
<img src='https://img.shields.io/badge/Type-Robotics%20%2B%20Automation-7fa6d9?style=for-the-badge' />

</div>

---

<div align='center'>

## Table of Contents

- [Overview](#overview)
- [The Problem](#the-problem)
- [System Architecture](#system-architecture)
- [Subsystem: Motion Controller](#subsystem-motion-controller)
- [Subsystem: PLC Ladder Logic](#subsystem-plc-ladder-logic)
- [Subsystem: Sensor Node](#subsystem-sensor-node)
- [End-to-End Operating Sequence](#end-to-end-operating-sequence)
- [Repository Contents](#repository-contents)
- [Tech Stack](#tech-stack)
- [Project Status](#project-status)
- [Simulation Notes & Known Limitations](#simulation-notes--known-limitations)
- [Author](#author)

---

## Overview

SEAL MULE is built around one core idea: an AMR that already travels a fixed route for material handling can, without any extra trips, also serve as a mobile data collector for sensors that can't afford to run their own wireless radio continuously.

The system is split into two independently-programmed controllers with clearly separated responsibilities — a **PLC layer** that owns every permission and safety decision, and a **motion controller** that owns every physical action. This separation mirrors how real industrial control systems distinguish sequencing/interlock logic (PLC) from continuous motion control (servo/motion controllers), rather than combining both into a single monolithic program.

---

## The Problem

Two distinct engineering problems motivate this design:

**1. Part verification in high-mix environments.** When a large number of visually similar parts exist in inventory, manually picking the correct one for a given job is error-prone. An automated pick-and-transport step that verifies part identity before moving it removes that risk at the source, rather than catching it downstream.

**2. Powering wireless sensors without wiring or batteries.** Vibration-energy-harvesting sensor nodes can power themselves indefinitely, but only if they conserve energy aggressively — and running a wireless radio continuously is typically the single largest energy cost for a low-power node. Having a mobile "collector" that only requests data when nearby avoids that cost almost entirely.

SEAL MULE addresses both problems with one physical robot rather than two separate systems.

---

## System Architecture

| | **Controller A — PLC Layer** | **Controller B — Motion Controller** |
|---|---|---|
| Hardware | ATmega16 | Arduino Mega 2560 |
| Programming | Ladder Logic (via LDmicro) | Arduino C/C++ |
| Responsibility | Start/Reset handling, part-match interlock, fault latching, beacon-timing sequencing | Motor driving, closed-loop PID speed control, RFID scanning, wireless beacon exchange, LCD status output |
| Style of logic | Discrete, permission-based ("IF condition THEN allow") | Continuous, calculation-based (real-time feedback control) |

The two controllers exchange a small set of discrete digital handshake signals — `Permit_to_Move`, `SKU_Match`, `Arrived_At_Node`, `Beacon_Trigger`, and `Done_or_Fault`. Each controller only needs to understand these shared signals, not the other's internal logic.

---

## Subsystem: Motion Controller

Runs on the Arduino Mega 2560 (`SEAL_MULE_Controller_B.ino`). Responsibilities:

- **Differential drive control** — two DC motors via an L293D H-bridge, independently controlled for direction and speed.
- **Closed-loop speed control** — dual-channel wheel encoders feed a PID loop, correcting actual speed against a target rather than running motors open-loop.
- **Part verification** — an MFRC522 RFID reader scans a tag and compares it against an expected ID before permitting onward movement.
- **Wireless data collection** — an NRF24L01 module sends a beacon at each sensor-node checkpoint and receives back a buffered reading.
- **Status display** — a 16x2 I2C LCD shows live task state.

---

## Subsystem: PLC Ladder Logic

Runs on the ATmega16 (built in LDmicro, compiled to `.hex`), and owns every decision in the system:

| Rung | Logic |
|---|---|
| 1 | Start pressed AND no active fault → set `Ready` |
| 2 | `Ready` AND `SKU_Match` → set `Permit_to_Move` |
| 3 | `Ready` AND NOT `SKU_Match` → latch `Fault` |
| 4 | `Fault` active → drive buzzer/LED alarm output |
| 5 | Reset pressed → clear `Fault` (manual reset only — a fault never self-clears) |
| 6 | `Arrived_At_Node` → after a short timer delay → set `Beacon_Trigger` |

Ladder logic was chosen deliberately for this layer because it is purpose-built for exactly this kind of permission/interlock reasoning, and is the same paradigm used in real industrial PLCs.

---

## Subsystem: Sensor Node

A separate low-power microcontroller (`Node_Code.ino`), representing a machine-mounted condition-monitoring device:

- Powered from a simulated vibration-energy harvester (AC source → bridge rectifier → storage capacitor) rather than the main supply rail — modeling a genuinely battery-free node.
- Spends almost all of its time in a low-power sleep state, taking occasional readings and buffering them locally.
- Wakes its radio only on receiving a beacon from the AMR, transmits its buffered reading, and returns to sleep.

---

## End-to-End Operating Sequence

1. Start pressed → PLC checks for an active fault → grants `Ready` → grants `Permit_to_Move`.
2. Robot drives to the part location under PID-controlled motion.
3. Robot scans the part's RFID tag, reports match/mismatch to the PLC.
4. **On match:** robot proceeds toward the destination, passing sensor-node checkpoint(s) along the way — sending a beacon at each, collecting the buffered reading, then continuing.
5. **On mismatch:** PLC latches a fault, buzzer/LED activate, robot halts — nothing proceeds until a human presses Reset.
6. Robot reaches its destination, reports task completion, and the PLC resets to a ready state for the next cycle.

---

## Repository Contents

| File | Description |
|---|---|
| `SEAL_MULE_Controller_B.ino` | Motion controller firmware (Arduino Mega 2560) |
| `Node_Code.ino` | Sensor node firmware |
| `SEAL_MULE_Presentation.pptx` | Project overview deck |
| `README.md` | This file |

*Ladder logic source and the full Proteus simulation project will be added once circuit simulation is complete.*

---

## Tech Stack

- **Proteus 8 Professional** — circuit design and simulation
- **Arduino IDE** — motion controller and sensor node firmware
- **LDmicro** — PLC ladder logic
- **Libraries:** MFRC522, RF24, LiquidCrystal_I2C

---

## Project Status

- [x] Architecture and problem scope finalized
- [x] Motion controller firmware written
- [x] Sensor node firmware written
- [x] Presentation deck prepared
- [ ] PLC ladder logic (LDmicro build)
- [ ] Full circuit simulation in Proteus
- [ ] End-to-end simulation test (match path + fault path)

This project is currently **simulation-only** — no physical hardware has been built.

---

## Simulation Notes & Known Limitations

- MFRC522 and NRF24L01 Proteus library models currently available require Proteus 8.9+; this project is being built on Proteus 8.6 Professional. A version upgrade or alternate library source is being evaluated.
- Proteus does not simulate true wireless RF propagation between two separate radio module instances — the wireless link between the AMR and sensor node is represented as a direct logical connection, clearly documented as such rather than presented as a literal RF simulation.
- The vibration-energy harvester is modeled as an AC source feeding a rectifier and storage capacitor, standing in for a piezoelectric element that cannot be directly simulated in Proteus.

---

## Author

Rahul — Final-year B.Tech, Electrical and Electronics Engineering
