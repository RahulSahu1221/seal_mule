/*
  SEAL MULE — Sensor Node
  Board: ATtiny85 (or substitute chip)
  Sleeps most of the time, wakes on a wireless beacon, sends one reading, sleeps again.
*/

#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8); // adjust CE/CSN pins to match your wiring

void setup() {
  radio.begin();
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL);  // must match the AMR's writing pipe address
  radio.openWritingPipe(0xF0F0F0F0D2LL);      // must match the AMR's reading pipe address
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    char incoming[5];
    radio.read(&incoming, sizeof(incoming));

    // A real beacon message triggers a reply
    if (String(incoming) == "WAKE") {
      radio.stopListening();

      // Simulated vibration reading — replace with a real analogRead()
      // from a sensor pin if you add one
      float simulatedReading = random(100, 500) / 10.0;

      radio.write(&simulatedReading, sizeof(simulatedReading));
      radio.startListening();
    }
  }
}