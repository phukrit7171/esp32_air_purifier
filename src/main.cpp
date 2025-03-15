#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <Adafruit_BME280.h>
Adafruit_BME280 bme; //i2c mode

#include "PMSensor.h"
// Create PMSensor instance with RX pin 16 and TX pin 17
PMSensor pmSensor(16, 17);

const int rgbLed[] = {4,2,15}; // R, G, B

const int PM25_GOOD = 12;      // 0-12 μg/m3 - Good
const int PM25_MODERATE = 35;  // 13-35 μg/m3 - Moderate
const int PM25_BAD = 55;       // 36-55 μg/m3 - Unhealthy for Sensitive Groups
// Above 55 μg/m3 - Unhealthy

void readBME280();
void readPMSensor();
void debugToSerial();
void readData();
void displayData();
void displayBME280();
void displayPMSensor();
void updateAirQualityLed();

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

enum DisplayState {
    SHOW_BME280,
    SHOW_PM_SENSOR
};

DisplayState currentDisplay = SHOW_BME280;
unsigned long lastDisplayChange = 0;
const unsigned long DISPLAY_INTERVAL = 5000; // 5 seconds

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

  // Add LED pin setup
  for(int i = 0; i < 3; i++) {
      pinMode(rgbLed[i], OUTPUT);
      digitalWrite(rgbLed[i], HIGH); // Turn off LED (common anode)
  }
}

void loop()
{
  readData();
  debugToSerial();
  displayData();
  updateAirQualityLed();
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

void displayData() {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastDisplayChange >= DISPLAY_INTERVAL) {
        lastDisplayChange = currentMillis;
        
        switch (currentDisplay) {
            case SHOW_BME280:
                displayBME280();
                currentDisplay = SHOW_PM_SENSOR;
                break;
                
            case SHOW_PM_SENSOR:
                displayPMSensor();
                currentDisplay = SHOW_BME280;
                break;
        }
    }
}

void displayBME280()
{
  lcd.clear();
  lcd.home();
  lcd.setCursor(0, 0);
  lcd.print("Temp  Hum   Pres");
  lcd.setCursor(0, 1);
  lcd.print(environmentData.temperature);
  lcd.setCursor(6, 1);
  lcd.print(environmentData.humidity);
  lcd.setCursor(12, 1);
  lcd.print(environmentData.pressure);
  // lcd.print(environmentData.altitude);
}

void displayPMSensor()
{
  lcd.clear();
  lcd.home();
  lcd.setCursor(0, 0);
  lcd.print("PM1.0");
  lcd.setCursor(6, 0);
  lcd.print("PM2.5");
  lcd.setCursor(12, 0);
  lcd.print("PM10");
  lcd.setCursor(0, 1);
  lcd.print(environmentData.pm1_0);
  lcd.setCursor(6, 1);
  lcd.print(environmentData.pm2_5);
  lcd.setCursor(12, 1);
  lcd.print(environmentData.pm10);  
}

void updateAirQualityLed() {
    // Common anode RGB LED - LOW turns on the color, HIGH turns it off
    if(environmentData.pm2_5 <= PM25_GOOD) {
        // Green - Good
        digitalWrite(rgbLed[0], HIGH);  // R off
        digitalWrite(rgbLed[1], LOW);   // G on
        digitalWrite(rgbLed[2], HIGH);  // B off
    }
    else if(environmentData.pm2_5 <= PM25_MODERATE) {
        // Yellow - Moderate (Red + Green)
        digitalWrite(rgbLed[0], LOW);   // R on
        digitalWrite(rgbLed[1], LOW);   // G on
        digitalWrite(rgbLed[2], HIGH);  // B off
    }
    else if(environmentData.pm2_5 <= PM25_BAD) {
        // Orange - Unhealthy for Sensitive Groups
        digitalWrite(rgbLed[0], LOW);    // R on
        digitalWrite(rgbLed[1], HIGH);   // G off
        digitalWrite(rgbLed[2], HIGH);   // B off
    }
    else {
        // Red - Unhealthy
        digitalWrite(rgbLed[0], LOW);    // R on
        digitalWrite(rgbLed[1], HIGH);   // G off
        digitalWrite(rgbLed[2], HIGH);   // B off
    }
}