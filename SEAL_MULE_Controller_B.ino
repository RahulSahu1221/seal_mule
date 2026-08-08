/*
  ============================================================
  SEAL MULE — Controller B (Motion Controller)
  Board: Arduino Mega 2560
  Handles: motor driving, PID speed control, RFID SKU check,
           wireless beacon to sensor nodes, LCD status display,
           handshake signals to/from Controller A (the PLC)
  ============================================================
*/

#include <SPI.h>              // needed for RFID and NRF24L01 (both use SPI bus)
#include <MFRC522.h>          // RFID reader library
#include <RF24.h>             // NRF24L01 wireless library
#include <Wire.h>             // needed for I2C (LCD)
#include <LiquidCrystal_I2C.h> // I2C LCD library

// ---------------- PIN DEFINITIONS ----------------
// Motor driver (L293D) pins
const int ENA = 5;    // Left motor speed (PWM signal)
const int IN1 = 22;   // Left motor direction control bit 1
const int IN2 = 23;   // Left motor direction control bit 2
const int ENB = 6;    // Right motor speed (PWM signal)
const int IN3 = 24;   // Right motor direction control bit 1
const int IN4 = 25;   // Right motor direction control bit 2

// Encoder pins (must be interrupt-capable pins on Mega: 2, 3, 18, 19, 20, 21)
const int ENC_LEFT_PIN  = 2;
const int ENC_RIGHT_PIN = 3;

// RFID module pins
const int RFID_RST_PIN = 8;
const int RFID_SS_PIN  = 53;

// NRF24L01 module pins
const int NRF_CE_PIN  = 7;
const int NRF_CSN_PIN = 9;

// Handshake pins with Controller A (the PLC)
const int PIN_PERMIT_IN       = 30; // INPUT  — PLC tells us we can move
const int PIN_BEACON_IN       = 31; // INPUT  — PLC tells us to fire the wireless beacon
const int PIN_SKU_MATCH_OUT   = 32; // OUTPUT — we tell PLC if the scanned part matched
const int PIN_ARRIVED_OUT     = 33; // OUTPUT — we tell PLC we reached a sensor-node stop
const int PIN_DONE_FAULT_OUT  = 34; // OUTPUT — we tell PLC the task finished (or failed)

// ---------------- OBJECTS ----------------
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2); // 0x27 is the common default I2C address for these backpacks

// ---------------- ENCODER PULSE COUNTERS ----------------
// "volatile" is required because these are changed inside an interrupt
volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;

// These small functions run automatically every time a pulse is detected
void leftEncoderInterrupt()  { leftPulseCount++; }
void rightEncoderInterrupt() { rightPulseCount++; }

// ---------------- PID CONTROL VARIABLES ----------------
// PID = Proportional-Integral-Derivative — a standard method to make actual
// speed match a target speed by continuously correcting the error.
float targetPulsesPer100ms = 40.0;  // tune this number by testing — represents your desired speed

float Kp = 2.0;   // how strongly we react to the CURRENT error
float Ki = 0.3;   // how strongly we react to error that has built up over time
float Kd = 0.5;   // how strongly we react to how FAST the error is changing

float leftIntegral = 0, leftLastError = 0;
float rightIntegral = 0, rightLastError = 0;

// The RFID tag we EXPECT to see (example values — replace with your real test tag's ID
// after you scan it once and print its UID to Serial Monitor to find out its real number)
byte expectedTagUID[4] = {0x12, 0x34, 0x56, 0x78};

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);   // for debugging — open Serial Monitor to watch what's happening

  SPI.begin();           // starts the shared SPI bus used by RFID and NRF24L01
  rfid.PCD_Init();        // initializes the RFID reader

  radio.begin();          // initializes the wireless module
  radio.openWritingPipe(0xF0F0F0F0E1LL);   // an address the AMR uses to send beacons
  radio.openReadingPipe(1, 0xF0F0F0F0D2LL); // an address the AMR listens on for replies
  radio.setPALevel(RF24_PA_LOW);            // low power level is enough for short range

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SEAL MULE");
  lcd.setCursor(0, 1);
  lcd.print("Waiting...");

  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Encoder pins — INPUT_PULLUP means the pin reads HIGH by default,
  // and drops LOW when the encoder signal triggers it
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), leftEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), rightEncoderInterrupt, RISING);

  // Handshake pins
  pinMode(PIN_PERMIT_IN, INPUT);
  pinMode(PIN_BEACON_IN, INPUT);
  pinMode(PIN_SKU_MATCH_OUT, OUTPUT);
  pinMode(PIN_ARRIVED_OUT, OUTPUT);
  pinMode(PIN_DONE_FAULT_OUT, OUTPUT);

  // Make sure all outputs start LOW (off)
  digitalWrite(PIN_SKU_MATCH_OUT, LOW);
  digitalWrite(PIN_ARRIVED_OUT, LOW);
  digitalWrite(PIN_DONE_FAULT_OUT, LOW);
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // Do nothing until Controller A (the PLC) grants permission
  if (digitalRead(PIN_PERMIT_IN) == HIGH) {

    lcd.clear(); lcd.print("Moving to rack");
    driveForDuration(1500);  // drive forward for 1.5 seconds toward the rack (tune this)
    stopMotors();

    bool matched = checkRFIDTag();
    digitalWrite(PIN_SKU_MATCH_OUT, matched ? HIGH : LOW);

    if (matched) {
      lcd.clear(); lcd.print("SKU OK");
      Serial.println("SKU matched. Continuing route.");

      // Drive to the first sensor-node checkpoint
      driveForDuration(1000);
      stopMotors();
      digitalWrite(PIN_ARRIVED_OUT, HIGH);   // tell PLC we arrived
      waitForBeaconAndCollectData();
      digitalWrite(PIN_ARRIVED_OUT, LOW);

      // Continue to the production line
      driveForDuration(1500);
      stopMotors();

      lcd.clear(); lcd.print("Delivered!");
      digitalWrite(PIN_DONE_FAULT_OUT, HIGH); // tell PLC the task is done
      delay(2000);
      digitalWrite(PIN_DONE_FAULT_OUT, LOW);

    } else {
      lcd.clear(); lcd.print("SKU MISMATCH");
      Serial.println("SKU did NOT match. Stopping.");
      stopMotors();
      digitalWrite(PIN_DONE_FAULT_OUT, HIGH); // tell PLC a fault occurred
      // Wait here until the PLC removes the Permit signal (after a human resets the fault)
      while (digitalRead(PIN_PERMIT_IN) == HIGH) {
        delay(100);
      }
      digitalWrite(PIN_DONE_FAULT_OUT, LOW);
    }

  } else {
    stopMotors();
  }
}

// ---------------- SUPPORT FUNCTIONS ----------------

// Drives both motors forward, using PID to correct each side's speed,
// for a fixed amount of time (in milliseconds).
void driveForDuration(int durationMs) {
  unsigned long startTime = millis();
  unsigned long lastPidTime = millis();

  leftPulseCount = 0;
  rightPulseCount = 0;

  while (millis() - startTime < (unsigned long)durationMs) {
    if (millis() - lastPidTime >= 100) {  // run the PID correction every 100ms
      runPID();
      lastPidTime = millis();
    }
  }
}

// One PID correction cycle — checks how many pulses came in during
// the last 100ms window compared to the target, and adjusts motor PWM.
void runPID() {
  // ----- LEFT MOTOR -----
  float leftError = targetPulsesPer100ms - leftPulseCount;
  leftIntegral += leftError;
  float leftDerivative = leftError - leftLastError;
  float leftOutput = (Kp * leftError) + (Ki * leftIntegral) + (Kd * leftDerivative);
  leftLastError = leftError;

  int leftPWM = constrain(150 + (int)leftOutput, 0, 255); // 150 = base speed, adjusted by PID
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);          // set direction: forward
  analogWrite(ENA, leftPWM);

  // ----- RIGHT MOTOR -----
  float rightError = targetPulsesPer100ms - rightPulseCount;
  rightIntegral += rightError;
  float rightDerivative = rightError - rightLastError;
  float rightOutput = (Kp * rightError) + (Ki * rightIntegral) + (Kd * rightDerivative);
  rightLastError = rightError;

  int rightPWM = constrain(150 + (int)rightOutput, 0, 255);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);          // forward
  analogWrite(ENB, rightPWM);

  // reset pulse counts for the next 100ms window
  leftPulseCount = 0;
  rightPulseCount = 0;
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Reads an RFID tag if present, and compares its ID against the expected value.
// Returns true if it matches, false if it doesn't (or if no tag was found).
bool checkRFIDTag() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return false; // no tag detected
  }

  bool isMatch = true;
  for (byte i = 0; i < 4; i++) {
    if (rfid.uid.uidByte[i] != expectedTagUID[i]) {
      isMatch = false;
    }
  }

  // Print the scanned tag's real ID to Serial Monitor —
  // use this the FIRST time you run this to find out your test tag's actual
  // UID, then update the expectedTagUID[] array above with real values.
  Serial.print("Scanned UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  rfid.PICC_HaltA();
  return isMatch;
}

// Sends a wireless "wake up" beacon, waits briefly, and prints
// whatever data comes back from the sensor node.
void waitForBeaconAndCollectData() {
  lcd.clear(); lcd.print("Sending beacon");

  radio.stopListening();
  const char beaconMsg[] = "WAKE";
  radio.write(&beaconMsg, sizeof(beaconMsg));

  radio.startListening();
  unsigned long waitStart = millis();
  bool gotReply = false;

  while (millis() - waitStart < 1000) {  // wait up to 1 second for a reply
    if (radio.available()) {
      float sensorReading;
      radio.read(&sensorReading, sizeof(sensorReading));
      Serial.print("Sensor node reading received: ");
      Serial.println(sensorReading);
      gotReply = true;
      break;
    }
  }

  if (!gotReply) {
    Serial.println("No reply from sensor node (timeout).");
  }
}