<div align="center">
    
# SEAL MULE

### *An Autonomous Kitting & Data-Mule Robot with PLC-Coordinated Part Verification*

A simulation-based automation project combining a PLC-controlled safety/sequencing layer with an autonomous mobile robot (AMR) that verifies parts before transporting them, and doubles as a wireless "data mule" for battery-free, vibration-powered sensor nodes placed along its route.

Built as a bridge between **ladder-logic PLC control** and **practical embedded/robotics engineering**.

![Status](https://img.shields.io/badge/Status-Simulation%20Working-brightgreen)
![Architecture](https://img.shields.io/badge/Architecture-Dual%20Controller-blue)
![PLC](https://img.shields.io/badge/PLC-Ladder%20Logic%20(LDmicro)-purple)
![MCU](https://img.shields.io/badge/MCU-Arduino%20Mega%202560-red)
![Simulation](https://img.shields.io/badge/Simulation-Proteus%208%20Professional-teal)
![Type](https://img.shields.io/badge/Type-Robotics%20%2B%20Automation-9cf)

---
</div>

## Table of Contents

- [Overview](#overview)
- [The Problem](#the-problem)
- [System Architecture](#system-architecture)
- [Hardware Substitutions](#hardware-substitutions)
- [Controller B: Full Wiring](#controller-b-full-wiring)
- [Controller A: Full Wiring](#controller-a-full-wiring)
- [Ladder Logic Program](#ladder-logic-program)
- [Chip Configuration Notes](#chip-configuration-notes)
- [Simulation Results](#simulation-results)
- [Repository Contents](#repository-contents)
- [Tech Stack](#tech-stack)
- [Project Status](#project-status)
- [Author](#author)

---

## Overview

SEAL MULE is built around one core idea: an AMR that already travels a fixed route for material handling can, without any extra trips, also serve as a mobile data collector for sensors that can't afford to run their own wireless radio continuously.

The system is split into two independently-programmed controllers with clearly separated responsibilities — a **PLC layer** that owns every permission and safety decision, and a **motion controller** that owns every physical action. This separation mirrors how real industrial control systems distinguish sequencing/interlock logic (PLC) from continuous motion control, rather than combining both into a single monolithic program.

---

## The Problem

**1. Part verification in high-mix environments.** When a large number of visually similar parts exist in inventory, manually picking the correct one for a given job is error-prone. An automated pick-and-transport step that verifies part identity before moving it removes that risk at the source.

**2. Powering wireless sensors without wiring or batteries.** Vibration-energy-harvesting sensor nodes can power themselves indefinitely, but only if they conserve energy aggressively — running a wireless radio continuously is typically the largest energy cost for a low-power node. A mobile collector that only requests data when nearby avoids that cost almost entirely.

---

## System Architecture

| | **Controller A — PLC Layer** | **Controller B — Motion Controller** |
|---|---|---|
| Hardware | ATmega16 | Arduino Mega 2560 |
| Programming | Ladder Logic (LDmicro) | Arduino C/C++ |
| Responsibility | Start/Reset handling, part-match interlock, fault latching, beacon-timing sequencing | Motor driving, closed-loop PID speed control, part verification, wireless beacon exchange, LCD status output |

The two controllers exchange discrete digital handshake signals: `Permit_to_Move`, `SKU_Match`, `Arrived_At_Node`, `Beacon_Trigger`, and a shared `Fault_Alarm_Output` line to the buzzer/LED.

---

## Hardware Substitutions

Two components originally planned required Proteus 8.9+ library models, while this project is built on **Proteus 8.6 Professional**. Rather than block progress on an unsupported upgrade, the following honest, documented substitutions were made — same logic, different physical signal source:

| Original plan | Substitution used | Reasoning |
|---|---|---|
| MFRC522 RFID reader | 4-position DIP switch (`DIPSW_4`) | Represents the scanned part code as a 4-bit digital value; same match/mismatch logic |
| NRF24L01 wireless module | Direct wire + potentiometer (`POT-HG`) | Represents the wireless data link and the sensor node's buffered reading |
| Incremental encoders | Push buttons (`BUTTON`) | Manually triggered pulses stand in for wheel-rotation pulses during simulation/demo |
| PCF8574 I2C LCD backpack | Direct-wired LM044L (parallel interface) | Proteus's PCF8574 model lacked usable VCC/GND/data pin mapping for this LCD; direct wiring uses the built-in `LiquidCrystal` library instead |

---

## Controller B: Full Wiring

**L293D motor driver:**

| L293D Pin | Connects to |
|---|---|
| 1 (Enable 1,2) | Mega pin 5 |
| 2 (Input 1) | Mega pin 22 |
| 3 (Output 1) | Left motor |
| 4, 5 (GND) | Ground |
| 6 (Output 2) | Left motor |
| 7 (Input 2) | Mega pin 23 |
| 8 (Vcc2, motor power) | Battery+ (9V) |
| 9 (Enable 3,4) | Mega pin 6 |
| 10 (Input 3) | Mega pin 24 |
| 11 (Output 3) | Right motor |
| 12, 13 (GND) | Ground |
| 14 (Output 4) | Right motor |
| 15 (Input 4) | Mega pin 25 |
| 16 (Vcc1, logic power) | +5V |

**Pulse inputs (encoder substitute):** Left button → Mega pin 2, Right button → Mega pin 3, both through a ground return.

**SKU DIP switch:** Switch 1–4 → Mega pins 40, 41, 42, 43, with pins 5–8 on the switch's return side commoned together and tied to Ground.

**Wireless/data-mule substitute:** Beacon output → Mega pin 7; sensor reading (potentiometer wiper) → Mega pin A8.

**LCD (LM044L, direct-wired):**

| LCD Pin | Connects to |
|---|---|
| 1 (VSS) | Ground |
| 2 (VDD) | +5V |
| 3 (VO) | Ground |
| 4 (RS) | Mega pin 44 |
| 5 (RW) | Ground |
| 6 (E) | Mega pin 45 |
| 11–14 (D4–D7) | Mega pins 46, 47, 48, 49 |

**Handshake to Controller A:** Permit-In → Mega pin 30, SKU-Match-Out → pin 32, Arrived-Out → pin 33, Done/Fault-Out → pin 34.

---

## Controller A: Full Wiring

| Signal | ATmega16 Pin |
|---|---|
| VCC / AVCC | Pins 10, 30 → +5V |
| GND | Pins 11, 31 → Ground |
| RESET | Pin 9 → +5V through 10kΩ pull-up |
| AREF | Pin 32 → Ground through 100nF |
| Start button | PB0 (pin 1) |
| Reset button | PB1 (pin 2) |
| SKU_Match (in, from Controller B pin 32) | PB2 (pin 3) |
| Arrived_At_Node (in, from Controller B pin 33) | PB3 (pin 4) |
| Permit_to_Move (out, to Controller B pin 30) | PC0 (pin 22) |
| Beacon_Trigger (out) | PC1 (pin 23) |
| Fault_Alarm_Output (out, to buzzer + LED) | PC2 (pin 24) |

---

## Ladder Logic Program

Built in LDmicro, targeting the ATmega16 at 16MHz. Six rungs:

| Rung | Logic |
|---|---|
| 1 | `StartButton` AND NOT `Fault` → set internal relay `Ready` |
| 2 | `Ready` AND `SKU_Match` → set `Permit_to_Move` |
| 3 | `Ready` AND NOT `SKU_Match` → **latch (Set)** internal relay `Fault` |
| 4 | `Fault` → drive `Fault_Alarm_Output` |
| 5 | `ResetButton` → **Reset** `Fault` (manual clear only — never self-clears) |
| 6 | `Arrived_At_Node` → after a 500ms timer (`TBeaconDelay`) → set `Beacon_Trigger` |

`Ready` and `Fault` are configured as **Internal Relays** (not pin-mapped I/O) — this LDmicro version requires every I/O-type signal to have a real pin assigned to compile, so purely internal flags must explicitly use the Internal Relay type rather than an unused pin.

---

## Chip Configuration Notes

Two Proteus/LDmicro configuration issues were found and resolved during bring-up, worth documenting for anyone reproducing this build:

1. **CKSEL Fuses mismatch caused a watchdog reset loop.** The ATmega16 defaulted to "(0001) Int.RC 1MHz" while the project's Clock Frequency field and LDmicro's PLC configuration were both set to 16MHz. This mismatch caused continuous AVR watchdog resets at simulation start. Fix: set CKSEL Fuses to the external high-frequency crystal option, matching the 16MHz clock setting everywhere else.
2. **WDTON should remain "(1) Unprogrammed"** so LDmicro's generated code can manage the watchdog itself rather than having it forced permanently on.

---

## Simulation Results

Both operating paths have been verified in Proteus:

- **Fault path:** DIP switch set to a non-matching code → LCD displays "SKU MISMATCH!" → buzzer and LED activate → robot halts → clears only after the Reset button is pressed.
- **Success path:** DIP switch set to match `expectedSKUCode` (`0b1010`) in the Controller B firmware → LCD progresses through "Moving to rack" → "SKU OK - Moving" → "At sensor node" (sensor reading displayed) → "Delivered!" → motors register active output during the drive segments.

---

## Repository Contents

| File | Description |
|---|---|
| `SEAL_MULE_Controller_B.ino` | Motion controller firmware (Arduino Mega 2560) |
| `Node_Code.ino` | Sensor node firmware |
| `SEAL_MULE_PLC.ld` | Ladder logic source (LDmicro) |
| `SEAL_MULE_PLC.hex` | Compiled PLC firmware (ATmega16) |
| `SEAL_MULE_Presentation.pptx` | Project overview deck |
| `README.md` | This file |

---

## Tech Stack

- **Proteus 8.6 Professional** — circuit design and simulation
- **Arduino IDE** — motion controller and sensor node firmware
- **LDmicro** — PLC ladder logic

---

## Project Status

- [x] Architecture and problem scope finalized
- [x] Motion controller firmware written
- [x] Sensor node firmware written
- [x] Hardware substitutions designed and documented (DIP switch, potentiometer, buttons, direct-wired LCD)
- [x] Ladder logic (6 rungs) built and compiled in LDmicro
- [x] Full circuit wired and simulated in Proteus
- [x] End-to-end simulation verified — both match path and fault path confirmed working
- [ ] Sensor node circuit (energy-harvester model) integration into the same schematic
- [ ] Final documentation pass and presentation screenshots

---

## Author

Rahul — Final-year B.Tech, Electrical and Electronics Engineering
