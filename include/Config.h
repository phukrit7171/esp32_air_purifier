#pragma once

// Pin Configurations
struct PinConfig {
    static const int RGB_LED[3];
    static const int BUTTON;
    static const int FAN;
    static const int PM_SENSOR_RX;
    static const int PM_SENSOR_TX;
};

// Air Quality Thresholds
struct AirQualityConfig {
    static const int PM25_GOOD;
    static const int PM25_MODERATE;
    static const int PM25_BAD;
    static const float PM_CALIBRATION_FACTOR;
};

// Timing Configurations
struct TimingConfig {
    static const unsigned long SENSOR_READ_INTERVAL;
    static const unsigned long DEBUG_OUTPUT_INTERVAL;
    static const unsigned long BUTTON_CHECK_INTERVAL;
    static const unsigned long DISPLAY_INTERVAL;
    static const unsigned long DEBOUNCE_DELAY;
    static const unsigned long HOLD_DURATION;
};

// Fan Configuration
struct FanConfig {
    static const int SPEED_MIN;
    static const int SPEED_MAX;
    static const int PWM_CHANNEL;
    static const int PWM_FREQ;
    static const int PWM_RESOLUTION;
};
