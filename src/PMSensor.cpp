#include "PMSensor.h"

PMSensor::PMSensor(uint8_t rxPin, uint8_t txPin) : _serial(rxPin, txPin) {
  _pm1 = 0;
  _pm2_5 = 0;
  _pm10 = 0;
}

void PMSensor::begin(long baudRate) {
  _serial.begin(baudRate);
}

bool PMSensor::read() {
  int index = 0;
  char value;
  char previousValue;
  bool headerFound = false;
  bool dataRead = false;
  
  // Wait for available data
  unsigned long startTime = millis();
  while (_serial.available() < 32 && (millis() - startTime) < 1000) {
    delay(10);
  }
  
  while (_serial.available()) {
    value = _serial.read();
    
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) {
      // Expected header bytes not found, resetting index.
      index = 0;
      continue;
    }
    
    if (index == 0 && value == 0x42) {
      headerFound = true;
    }
    
    if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14) {
      previousValue = value;
    }
    else if (index == 5) {
      _pm1 = 256 * previousValue + value;
    }
    else if (index == 7) {
      _pm2_5 = 256 * previousValue + value;
    }
    else if (index == 9) {
      _pm10 = 256 * previousValue + value;
      dataRead = true;
    }
    else if (index > 15) {
      break;
    }
    
    index++;
  }
  
  // Clear any remaining data
  while (_serial.available()) {
    _serial.read();
  }
  
  return dataRead && headerFound;
}

unsigned int PMSensor::getPM1() const {
  return _pm1;
}

unsigned int PMSensor::getPM2_5() const {
  return _pm2_5;
}

unsigned int PMSensor::getPM10() const {
  return _pm10;
}

void PMSensor::printData() const {
  Serial.print("{ ");
  Serial.print("\"pm1\": ");
  Serial.print(_pm1);
  Serial.print(" ug/m3");
  Serial.print(", ");
  Serial.print("\"pm2_5\": ");
  Serial.print(_pm2_5);
  Serial.print(" ug/m3");
  Serial.print(", ");
  Serial.print("\"pm10\": ");
  Serial.print(_pm10);
  Serial.print(" ug/m3");
  Serial.println(" }");
}