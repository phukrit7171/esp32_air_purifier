#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

#include "PMSensor.h"
// Create PMSensor instance with RX pin 16 and TX pin 17
PMSensor pmSensor(16, 17);

void readBME280();
void readPMSensor();
void debugToSerial();
void readData();
void displayData();
void displayBME280();
void displayPMSensor();

struct EnvironmentData
{
  float temperature;
  float pressure;
  float humidity;
  float altitude;
  int pm1_0;
  int pm2_5;
  int pm10;
} environmentData;



void setup()
{
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.home();
  lcd.print("Initializing...");
  Serial.begin(9600);
  pmSensor.begin(9600);
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  lcd.clear();
  lcd.home();
  lcd.print("Ready!");
  lcd.noAutoscroll();
}

void loop()
{
  readData();
  debugToSerial();
  displayData();
}


void debugToSerial()
{
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
  Serial.print(environmentData.pm1_0);
  Serial.println(" μg/m3");
  Serial.print("PM2.5 = ");
  Serial.print(environmentData.pm2_5);
  Serial.println(" μg/m3");
  Serial.print("PM10 = ");
  Serial.print(environmentData.pm10);
  Serial.println(" μg/m3");
  Serial.println();
}

void readBME280()
{
  environmentData.temperature = bme.readTemperature();
  environmentData.pressure = bme.readPressure() / 100.0F;
  environmentData.humidity = bme.readHumidity();
  environmentData.altitude = bme.readAltitude(1013.25);
}

void readPMSensor()
{
  if (!pmSensor.read())
  {
    Serial.println("Could not read data from PMSensor");
    return;
  }
  environmentData.pm1_0 = pmSensor.getPM1();
  environmentData.pm2_5 = pmSensor.getPM2_5();
  environmentData.pm10 = pmSensor.getPM10();
}

void readData()
{
  readBME280();
  readPMSensor();
}

void displayData(){
  displayBME280();
  delay(5000);
  displayPMSensor();
  delay(5000);
}

void displayBME280()
{
  lcd.clear();
  lcd.home();
  lcd.print("T:");
  lcd.print(environmentData.temperature);
  lcd.print(" H:");
  lcd.print(environmentData.humidity);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("P:");
  lcd.print(environmentData.pressure);
  lcd.print(" A:");
  lcd.print(environmentData.altitude);
  lcd.print(" m");
}

void displayPMSensor()
{
  lcd.clear();
  lcd.home();
  lcd.print("PM1.0:");
  lcd.print(environmentData.pm1_0);
  lcd.setCursor(0, 1);
  lcd.print("PM2.5:");
  lcd.print(environmentData.pm2_5);
  lcd.print("PM10:");
  lcd.print(environmentData.pm10);
}