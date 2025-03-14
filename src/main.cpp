#include <Arduino.h>

#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

#include "PMSensor.h"
// Create PMSensor instance with RX pin 16 and TX pin 17
PMSensor pmSensor(16, 17);

struct EnvironmentData {
  float temperature;
  float pressure;
  float humidity;
  float altitude;
  int pm1_0;
  int pm2_5;
  int pm10;
} environmentData;

void readBME280() {
  environmentData.temperature = bme.readTemperature();
  environmentData.pressure = bme.readPressure() / 100.0F;
  environmentData.humidity = bme.readHumidity();
  environmentData.altitude = bme.readAltitude(1013.25);
}

void readPMSensor() {
  if (!pmSensor.read()) {
    Serial.println("Could not read data from PMSensor");
    return;
  }
  environmentData.pm1_0 = pmSensor.getPM1();
  environmentData.pm2_5 = pmSensor.getPM2_5();
  environmentData.pm10 = pmSensor.getPM10();
}

void setup() {
  Serial.begin(9600);
  pmSensor.begin(9600);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  readBME280();
  readPMSensor();

  Serial.print("Temperature = ");
  Serial.print(environmentData.temperature);
  Serial.println(" *C");
  Serial.print("Humidity = ");
  Serial.print(environmentData.humidity);
  Serial.println(" %");
  Serial.print("Pressure = ");
  Serial.print(environmentData.pressure);
  Serial.println(" hPa");
  Serial.print("Altitude = ");
  Serial.print(environmentData.altitude);
  Serial.println(" m");
  Serial.print("PM1.0 = ");
  Serial.println(environmentData.pm1_0);
  Serial.print("PM2.5 = ");
  Serial.println(environmentData.pm2_5);
  Serial.print("PM10 = ");
  Serial.println(environmentData.pm10);
  Serial.println();

  delay(1000);
}