
/* 
 * Project: pump with IRLZ44N test
 * Author: Nicole
 * Date: 3/24/2026
 * Description: Test code for controlling a pump using an IRLZ44N MOSFET.
 * The button is connected to D7 with an internal pullup, and the pump is connected to D4 (gate of the MOSFET). 
 * When the button is pressed, the pump should turn on, and when released, it should turn off. 
 * This code is just for testing that basic functionality.
 */

 #include "Particle.h"

SYSTEM_MODE(SEMI_AUTOMATIC);

const int BUTTON_PIN = D7;
const int PUMP_PIN = D4; // Gate of IRLZ44N MOSFET  

void setup() {
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button connects to GND when pressed
  digitalWrite(PUMP_PIN, LOW); // Ensure pump is off at start
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) { // Button pressed
    digitalWrite(PUMP_PIN, HIGH); // Turn pump on
  } 
  else { // Button released
    digitalWrite(PUMP_PIN, LOW); // Turn pump off
  }
}
