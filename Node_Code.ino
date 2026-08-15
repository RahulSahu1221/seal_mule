/*
  ============================================================
  SEAL MULE — Sensor Node — SIMPLIFIED VERSION
  Board: any small AVR chip (ATtiny85 / ATmega16 / similar)

  This version does NOT use the NRF24L01 library. The node's reading
  is represented by a potentiometer wired directly to the AMR's
  analog input (SENSOR_DATA_IN_PIN in the Controller B code) —
  this direct wire stands in for the wireless link, as documented
  in the project README.

  This code's only real job is to detect the beacon signal from the
  AMR and light an LED briefly, visually showing the node "waking up" —
  matching the sleep/wake story from the project design, even though
  the actual reading is delivered by the direct wire, not by this code.
  ============================================================
*/

const int BEACON_IN_PIN = 2;  // receives the beacon signal from the AMR (direct wire)
const int WAKE_LED_PIN  = 8;  // lights up while the node is "awake"

void setup() {
  pinMode(BEACON_IN_PIN, INPUT);
  pinMode(WAKE_LED_PIN, OUTPUT);
  digitalWrite(WAKE_LED_PIN, LOW);
}

void loop() {
  if (digitalRead(BEACON_IN_PIN) == HIGH) {
    digitalWrite(WAKE_LED_PIN, HIGH);  // node "wakes up" while beacon is active
  } else {
    digitalWrite(WAKE_LED_PIN, LOW);   // node returns to "sleep"
  }
}
