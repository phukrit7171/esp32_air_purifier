#ifndef AIRPURIFIER_H
#define AIRPURIFIER_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <PMSensor.h>

struct PinConfig {
    static const int RGB_LED[3];
    static const int BUTTON;
    static const int FAN;
    static const int PM_SENSOR_RX;
    static const int PM_SENSOR_TX;
};

struct AirQualityConfig {
    static const int PM25_GOOD;
    static const int PM25_MODERATE;
    static const int PM25_BAD;
    static const float PM_CALIBRATION_FACTOR;
};

struct TimingConfig {
    static const unsigned long SENSOR_READ_INTERVAL;
    static const unsigned long DEBUG_OUTPUT_INTERVAL;
    static const unsigned long BUTTON_CHECK_INTERVAL;
    static const unsigned long DISPLAY_INTERVAL;
    static const unsigned long DEBOUNCE_DELAY;
    static const unsigned long HOLD_DURATION;
};

// Using FanConfig.h instead of redefining FanControlMethod, FanPolarity and FanConfig here
#include "FanConfig.h"

class AirPurifier {
public:
    AirPurifier();
    void begin();
    void update();

private:
    void readSensors();
    void updateDisplay();
    void handleButton();
    void updateLED();
    void updateFan();
    void debugOutput();
    void displayBME280Data();
    void displayPMData();

    LiquidCrystal_I2C lcd;
    Adafruit_BME280 bme;
    PMSensor pmSensor;

    struct EnvData {
        float temperature;
        float pressure;
        float humidity;
        float altitude;
        int pm1_0;
        int pm2_5;
        int pm10;
    } envData;

    enum class DisplayState {
        BME280,
        PM_SENSOR
    } currentDisplay;

    bool systemEnabled;
    bool displayEnabled;
    bool ledEnabled;
    bool autoFanControl;
    int currentFanSpeed;
    bool isPwmAttached;
    FanControlMethod previousControlMethod;

    unsigned long lastSensorRead;
    unsigned long lastDebugOutput;
    unsigned long lastButtonCheck;
    unsigned long lastDisplayChange;
    int lastButtonState;
    unsigned long buttonPressStartTime;
    unsigned long lastDebounceTime;
};

#endif // AIRPURIFIER_H
