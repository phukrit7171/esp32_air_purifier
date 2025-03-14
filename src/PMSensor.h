#ifndef PMSensor_h
#define PMSensor_h

#include <Arduino.h>
#include <SoftwareSerial.h>

class PMSensor {
  public:
    PMSensor(uint8_t rxPin, uint8_t txPin);
    void begin(long baudRate = 9600);
    bool read();
    
    // Data getters
    unsigned int getPM1() const;
    unsigned int getPM2_5() const;
    unsigned int getPM10() const;
    
    // Print data to Serial
    void printData() const;
    
  private:
    SoftwareSerial _serial;
    unsigned int _pm1;
    unsigned int _pm2_5;
    unsigned int _pm10;
};

#endif