#pragma once

#include "Config.h"
#include "EnvironmentData.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BME280.h>
#include "PMSensor.h"

enum class DisplayState {
    BME280,
    PM_SENSOR
};

class AirPurifier {
public:
    AirPurifier();
    void begin();
    void update();

private:
    LiquidCrystal_I2C lcd;
    Adafruit_BME280 bme;
    PMSensor pmSensor;
    EnvironmentData envData;
    
    bool systemEnabled;
    bool displayEnabled;
    bool ledEnabled;
    bool autoFanControl;
    DisplayState currentDisplay;
    int currentFanSpeed;
    
    // Timing variables
    unsigned long lastSensorRead;
    unsigned long lastDebugOutput;
    unsigned long lastButtonCheck;
    unsigned long lastDisplayChange;
    
    // Button handling
    int lastButtonState;
    unsigned long buttonPressStartTime;
    unsigned long lastDebounceTime;
    
    void readSensors();
    void updateDisplay();
    void handleButton();
    void updateLED();
    void updateFan();
    void debugOutput();
    
    void displayBME280Data();
    void displayPMData();
};
