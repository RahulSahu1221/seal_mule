/*
  ============================================================
  SEAL MULE — Controller B (Motion Controller) — SIMPLIFIED VERSION
  Board: Arduino Mega 2560

  This version does NOT use MFRC522 or NRF24L01 libraries, because
  the Proteus models for those parts require Proteus 8.9+, and this
  project is being built on Proteus 8.6.

  Substitutions made (documented honestly, see README):
    - RFID reader  -> 4-position DIP switch (represents scanned SKU code)
    - NRF24L01 link -> direct wire to sensor node (represents wireless link)

  All other logic (PID motor control, handshake with the PLC, fault
  handling) is unchanged from the original design.
  ============================================================
*/

// ---------------- PIN DEFINITIONS ----------------
// Motor driver (L293D) pins
const int ENA = 5;
const int IN1 = 22;
const int IN2 = 23;
const int ENB = 6;
const int IN3 = 24;
const int IN4 = 25;

// Encoder pins (interrupt-capable)
const int ENC_LEFT_PIN  = 2;
const int ENC_RIGHT_PIN = 3;

// DIP switch pins — represents the "scanned SKU code" (replaces MFRC522)
const int SKU_BIT0 = 40;
const int SKU_BIT1 = 41;
const int SKU_BIT2 = 42;
const int SKU_BIT3 = 43;

// Direct-wire "wireless" link to the sensor node (replaces NRF24L01)
const int BEACON_OUT_PIN     = 7;   // sends the beacon signal to the node
const int SENSOR_DATA_IN_PIN = A8;  // reads the node's reading (always available on this line)

// Handshake pins with Controller A (the PLC)
const int PIN_PERMIT_IN      = 30;
const int PIN_SKU_MATCH_OUT  = 32;
const int PIN_ARRIVED_OUT    = 33;
const int PIN_DONE_FAULT_OUT = 34;

// ---------------- ENCODER COUNTERS ----------------
volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;

void leftEncoderInterrupt()  { leftPulseCount++; }
void rightEncoderInterrupt() { rightPulseCount++; }

// ---------------- PID VARIABLES ----------------
float targetPulsesPer100ms = 40.0;  // tune by testing
float Kp = 2.0, Ki = 0.3, Kd = 0.5;
float leftIntegral = 0, leftLastError = 0;
float rightIntegral = 0, rightLastError = 0;

// The 4-bit code we expect the DIP switch to show for a "correct" part.
// Example: 0b1010 means switches 1 and 3 ON, switches 2 and 4 OFF.
// Set your test switches to match this pattern to simulate a MATCH,
// and to any other pattern to simulate a MISMATCH.
const int expectedSKUCode = 0b1010;

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), leftEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), rightEncoderInterrupt, RISING);

  // DIP switch pins — INPUT_PULLUP means "not pressed" reads HIGH,
  // and each switch pulls its pin LOW when turned ON
  pinMode(SKU_BIT0, INPUT_PULLUP);
  pinMode(SKU_BIT1, INPUT_PULLUP);
  pinMode(SKU_BIT2, INPUT_PULLUP);
  pinMode(SKU_BIT3, INPUT_PULLUP);

  pinMode(BEACON_OUT_PIN, OUTPUT);
  digitalWrite(BEACON_OUT_PIN, LOW);

  pinMode(PIN_PERMIT_IN, INPUT);
  pinMode(PIN_SKU_MATCH_OUT, OUTPUT);
  pinMode(PIN_ARRIVED_OUT, OUTPUT);
  pinMode(PIN_DONE_FAULT_OUT, OUTPUT);

  digitalWrite(PIN_SKU_MATCH_OUT, LOW);
  digitalWrite(PIN_ARRIVED_OUT, LOW);
  digitalWrite(PIN_DONE_FAULT_OUT, LOW);
}

// ---------------- MAIN LOOP ----------------
void loop() {
  if (digitalRead(PIN_PERMIT_IN) == HIGH) {

    driveForDuration(1500);   // drive to the rack position
    stopMotors();

    bool matched = checkSKUSwitch();
    digitalWrite(PIN_SKU_MATCH_OUT, matched ? HIGH : LOW);

    if (matched) {
      Serial.println("SKU matched. Continuing route.");

      driveForDuration(1000);  // drive to sensor-node checkpoint
      stopMotors();
      digitalWrite(PIN_ARRIVED_OUT, HIGH);
      sendBeaconAndReadData();
      digitalWrite(PIN_ARRIVED_OUT, LOW);

      driveForDuration(1500);  // drive to production line
      stopMotors();

      digitalWrite(PIN_DONE_FAULT_OUT, HIGH);
      delay(2000);
      digitalWrite(PIN_DONE_FAULT_OUT, LOW);

    } else {
      Serial.println("SKU mismatch. Stopping.");
      stopMotors();
      digitalWrite(PIN_DONE_FAULT_OUT, HIGH);
      // wait here until the PLC removes Permit (after a human resets the fault)
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

void driveForDuration(int durationMs) {
  unsigned long startTime = millis();
  unsigned long lastPidTime = millis();
  leftPulseCount = 0;
  rightPulseCount = 0;

  while (millis() - startTime < (unsigned long)durationMs) {
    if (millis() - lastPidTime >= 100) {
      runPID();
      lastPidTime = millis();
    }
  }
}

void runPID() {
  float leftError = targetPulsesPer100ms - leftPulseCount;
  leftIntegral += leftError;
  float leftDerivative = leftError - leftLastError;
  float leftOutput = (Kp * leftError) + (Ki * leftIntegral) + (Kd * leftDerivative);
  leftLastError = leftError;

  int leftPWM = constrain(150 + (int)leftOutput, 0, 255);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, leftPWM);

  float rightError = targetPulsesPer100ms - rightPulseCount;
  rightIntegral += rightError;
  float rightDerivative = rightError - rightLastError;
  float rightOutput = (Kp * rightError) + (Ki * rightIntegral) + (Kd * rightDerivative);
  rightLastError = rightError;

  int rightPWM = constrain(150 + (int)rightOutput, 0, 255);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, rightPWM);

  leftPulseCount = 0;
  rightPulseCount = 0;
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Reads the DIP switch and compares it to the expected code.
// Returns true if it matches, false otherwise.
bool checkSKUSwitch() {
  int code = 0;
  code |= (!digitalRead(SKU_BIT0)) << 0;  // switch ON = pin reads LOW
  code |= (!digitalRead(SKU_BIT1)) << 1;
  code |= (!digitalRead(SKU_BIT2)) << 2;
  code |= (!digitalRead(SKU_BIT3)) << 3;

  Serial.print("Scanned SKU code: ");
  Serial.println(code, BIN);

  return code == expectedSKUCode;
}

// Sends a beacon to the sensor node and reads back its data over the
// direct wire link (representing the wireless exchange).
void sendBeaconAndReadData() {
  digitalWrite(BEACON_OUT_PIN, HIGH);
  delay(200); // gives the node a moment to "respond" (represents the wireless handshake delay)

  int rawValue = analogRead(SENSOR_DATA_IN_PIN);
  float sensorReading = rawValue * (5.0 / 1023.0);

  Serial.print("Sensor node reading received: ");
  Serial.println(sensorReading);

  digitalWrite(BEACON_OUT_PIN, LOW);
}
