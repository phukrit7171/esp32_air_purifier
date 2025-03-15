#pragma once

class EnvironmentData {
public:
    float temperature;
    float pressure;
    float humidity;
    float altitude;
    int pm1_0;
    int pm2_5;
    int pm10;
    
    void reset() {
        temperature = 0;
        pressure = 0;
        humidity = 0;
        altitude = 0;
        pm1_0 = 0;
        pm2_5 = 0;
        pm10 = 0;
    }
};
