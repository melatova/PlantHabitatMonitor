/* 
 * Project: Read DIY Moisture Sensor - is your water reservoir dry?
 * Author: Nicole
 * Date: 03-28-26
 * Description: Moisture sensor make from stainless steel earring hooks and resistor (e.g. 10K)
 * Also made one for the gravel dish with air plants with 3 MOhm resistors in series.
 * 
 * Used a 3by black connector from the rainbow wire kits.
 * Used female connector from rainbow kit to hold each straightened earring hook. 
 * Used a rainbow wire (double, to connect). 
 * Slathered in UV adhesive and cured.
 *  
 * ********* How to Wire It: 
 * Hook 1 (Power): Connect to Digital Pin OUTPUT , e.g D2.
 * Hook 2 (Signal): Connect to Analog Pin INPUT, e.g A0.
 * The 10k Resistor: Connect one leg to A0 and the other leg to GND (pulldown in parallel)
 * 
 * ********* Mechanism:
 * 
 * When Dry: The resistor "pulls" the voltage at A0 down to 0V (Ground). 
 * analogRead(A0) will be 0.
 * 
 * When Wet: Water acts as a bridge. Electricity flows from D2, through the water, into A0. 
 * This overcomes the resistor and gives a high reading (e.g., 2500).
 * 
 * ********* Pulsed Power:
 * Since the stainless hooks will be sitting in water 24/7, 
 * remember to only turn D2 HIGH for a split second right before analogRead(A0), 
 * then turn it LOW immediately. This prevents the "battery effect" that would 
 * eventually eat away at the stainless steel hooks and mess up the plant's water chemistry.
 * 
 * ********* For triggering notifications:
 * Set a threshold of 500 to notify that the resevoir needs a refill. 
 * 
 *  ********* Thresholds:
 * For using the sensor with the gravel dish with 3 @ 1MOhm resistors in series, use these threshholds: 
 * 
 * Hysteresis (The Buffer):
 * Turn Pump ON when Gravel is < 500 (example).
 * Turn Pump OFF when Gravel is > 1500 (example).

 */


#include "Particle.h"


SYSTEM_MODE(MANUAL); // Set the system mode to MANUAL to control when the device connects to the cloud

int moistureValue = 0; // Variable to store the moisture level

void setup() {
  Serial.begin(9600); // Start the Serial communication for debugging
  waitFor(Serial.isConnected, 10000); // Wait for the Serial connection to be established 
  Serial.println("Starting Moisture Sensor...");

  //pinMode(D8, OUTPUT); // Set D2 as an Digital OUTPUT for the power hook
  //pinMode(A2, INPUT);  // Set A0 as an Analog INPUT for the signal hook  
  pinMode(A0, OUTPUT); // Set D2 as an Digital OUTPUT for the power hook
  pinMode(A5, INPUT);  // Set A0 as an Analog INPUT for the signal hook  

}

void loop() {
  digitalWrite(A0, HIGH); // Power the sensor by setting D2 HIGH
  delay(100); // Short delay to allow the sensor to stabilize
  
  moistureValue = analogRead(A5); // Read the moisture level from A0
  Serial.println(moistureValue); // Print the moisture value to the Serial Monitor
  
  digitalWrite(A0, LOW); // Turn off power to the sensor immediately after reading
  delay(5000); // Wait for 5 seconds before the next reading (only for testing, in reality will only test every hour)
}


