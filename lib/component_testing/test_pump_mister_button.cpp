
/* 
 * Project: Test pump and mister
 * Author: Nicole
 * Date: 3/20/2026
 * Description: Verify pump and mister are wired correctly and turn on and off. Include pullup button at d7 
 * for pump
 *
 */

 #include "Particle.h"

SYSTEM_MODE(SEMI_AUTOMATIC);


const int TEST_PUMP_PIN = D4;
const int TEST_MIST_PIN = D5;
const int BUTTON_PUMP_PIN = D7;

void setup() {
    pinMode(TEST_PUMP_PIN, OUTPUT);
    pinMode(TEST_MIST_PIN, OUTPUT);
    pinMode(BUTTON_PUMP_PIN, INPUT_PULLUP);
      Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);
    Serial.printf("Ready to test the pump and mister!\n");
}

void loop() {

        // Test Button for Pump 
    if (digitalRead(BUTTON_PUMP_PIN)) ==  {
        Serial.println("Button Pump is ON");        

    } else {
        Serial.println("Button Pump is OFF");
    }   
    Serial.println("Testing Button Pump... ON");
    digitalWrite(BUTTON_PUMP_PIN, HIGH);
    delay(2000); // Run for 2 seconds
    // Test Pump
    Serial.println("Testing Pump... ON");
    digitalWrite(TEST_PUMP_PIN, HIGH);
    delay(2000); // Run for 2 seconds
    
    Serial.println("Testing Pump... OFF");
    digitalWrite(TEST_PUMP_PIN, LOW);
    delay(9000);

    // Test Mister
    Serial.println("Testing Mister... ON");
    digitalWrite(TEST_MIST_PIN, HIGH);
    delay(2000); 
    
    Serial.println("Testing Mister... OFF");
    digitalWrite(TEST_MIST_PIN, LOW);
    
    //Serial.println("Waiting 5 seconds for next cycle...");

    delay(10000); // Run for 2 seconds

}