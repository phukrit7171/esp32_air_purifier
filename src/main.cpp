// Main entry point for the ESP32 Air Purifier project.
// The file initializes the AirPurifier system and continuously updates it.

#include <Arduino.h>
#include "AirPurifier.h"

// Instantiate and initialize the air purifier system.
AirPurifier airPurifier;

void setup() {
    // Start serial communication for debugging.
    Serial.begin(9600);
    // Initialize the air purifier system, sensors, display, fan, etc.
    airPurifier.begin();
}

void loop() {
    // Continuously update the air purifier functionalities.
    airPurifier.update();
}