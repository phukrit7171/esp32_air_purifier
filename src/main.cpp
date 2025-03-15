#include <Arduino.h>
#include "AirPurifier.h"

AirPurifier airPurifier;

void setup() {
    Serial.begin(9600);
    airPurifier.begin();
}

void loop() {
    airPurifier.update();
}