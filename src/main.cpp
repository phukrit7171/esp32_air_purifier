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

// Add these color mapping structs at the top with other constants
struct RGB {
    int r, g, b;
};

const RGB AQI_COLORS[] = {
    {0, 255, 0},    // Green (Good)
    {255, 255, 0},  // Yellow (Moderate)
    {255, 128, 0},  // Orange (Unhealthy for Sensitive Groups)
    {255, 0, 0}     // Red (Unhealthy)
};

const int AQI_LEVELS[] = {
    PM25_GOOD,      // 12
    PM25_MODERATE,  // 35
    PM25_BAD,       // 55
    999             // Max value
};

// Add these declarations after other constants
const int BUTTON_PIN = 5;
bool displayEnabled = true;
bool ledEnabled = true;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;
int lastButtonState = HIGH;
int buttonState = HIGH;

// Add these variables with other global variables
const unsigned long HOLD_DURATION = 2000; // 2 seconds hold to power on/off
unsigned long buttonPressStartTime = 0;
bool systemEnabled = true;

// Add these timing constants near other constants
const unsigned long SENSOR_READ_INTERVAL = 2000;    // Read sensors every 2 seconds
const unsigned long DEBUG_OUTPUT_INTERVAL = 1000;   // Debug output every 1 second
const unsigned long BUTTON_CHECK_INTERVAL = 50;     // Check button every 50ms

// Add these timing variables with other global variables
unsigned long lastSensorRead = 0;
unsigned long lastDebugOutput = 0;
unsigned long lastButtonCheck = 0;

void readBME280();
void readPMSensor();
void debugToSerial();
void readData();
void displayData();
void displayBME280();
void displayPMSensor();
void updateAirQualityLed();
// Add this function declaration with the others
void handleButton();

// Add these function declarations
void debugBME280();
void debugPMSensor();

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

  // Add button setup
  pinMode(BUTTON_PIN, INPUT);
    
  lcd.backlight();  // Ensure backlight is on initially
}

// Modify loop() for non-blocking operation
void loop() {
    unsigned long currentMillis = millis();
    
    // Check button state with interval
    if (currentMillis - lastButtonCheck >= BUTTON_CHECK_INTERVAL) {
        lastButtonCheck = currentMillis;
        handleButton();
    }
    
    if (systemEnabled) {
        // Read sensors with interval
        if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
            lastSensorRead = currentMillis;
            readData();
        }
        
        // Debug output with interval
        if (currentMillis - lastDebugOutput >= DEBUG_OUTPUT_INTERVAL) {
            lastDebugOutput = currentMillis;
            debugToSerial();
        }
        
        if (displayEnabled) {
            displayData();
        }
        if (ledEnabled) {
            updateAirQualityLed();
        }
    }
}

// Replace debugToSerial() with these three functions
void debugToSerial() {
    debugBME280();
    debugPMSensor();
    Serial.println();
}

void debugBME280() {
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
}

void debugPMSensor() {
    Serial.print("PM1.0 = ");
    Serial.print(environmentData.pm1_0);
    Serial.println(" μg/m3");
    Serial.print("PM2.5 = ");
    Serial.print(environmentData.pm2_5);
    Serial.println(" μg/m3");
    Serial.print("PM10 = ");
    Serial.print(environmentData.pm10);
    Serial.println(" μg/m3");
}

void readBME280()
{
  environmentData.temperature = bme.readTemperature();
  environmentData.pressure = bme.readPressure() / 100.0F;
  environmentData.humidity = bme.readHumidity();
  environmentData.altitude = bme.readAltitude(1013.25);
}

// Add calibration multiplier constant
const float PM_CALIBRATION_FACTOR = 3.0;  // 3x multiplier for PM sensor readings

// Modify readPMSensor() to handle timeouts and apply calibration
void readPMSensor() {
    unsigned long startTime = millis();
    bool readSuccess = false;
    
    while (millis() - startTime < 1000) {  // 1 second timeout
        if (pmSensor.read()) {
            // Apply calibration factor to all PM readings
            environmentData.pm1_0 = round(pmSensor.getPM1() * PM_CALIBRATION_FACTOR);
            environmentData.pm2_5 = round(pmSensor.getPM2_5() * PM_CALIBRATION_FACTOR);
            environmentData.pm10 = round(pmSensor.getPM10() * PM_CALIBRATION_FACTOR);
            readSuccess = true;
            break;
        }
        yield();  // Allow other tasks to run
    }
    
    if (!readSuccess) {
        Serial.println("PM Sensor read timeout");
    }
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

// Modify handleButton() to remove blocking delays
void handleButton() {
    int reading = digitalRead(BUTTON_PIN);
    
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
        
        if (reading == LOW) {
            buttonPressStartTime = millis();
        }
        else if (reading == HIGH) {
            unsigned long pressDuration = millis() - buttonPressStartTime;
            if (pressDuration < HOLD_DURATION) {
                // Short press - toggle display if system is enabled
                if (systemEnabled) {
                    displayEnabled = !displayEnabled;
                    if (displayEnabled) {
                        lcd.backlight();
                    } else {
                        lcd.noBacklight();
                    }
                }
            }
        }
    }
    
    // Check for long press while button is still being held
    if (reading == LOW) {
        unsigned long pressDuration = millis() - buttonPressStartTime;
        if (pressDuration >= HOLD_DURATION) {
            // Long press - toggle system state
            systemEnabled = !systemEnabled;
            
            if (!systemEnabled) {
                displayEnabled = false;
                ledEnabled = false;
                lcd.noBacklight();
                // Turn OFF the LEDs (set pins LOW for common anode)
                digitalWrite(rgbLed[0], LOW);
                digitalWrite(rgbLed[1], LOW);
                digitalWrite(rgbLed[2], LOW);
            } else {
                displayEnabled = true;
                ledEnabled = true;
                lcd.backlight();
            }
            
            // Wait for button release to prevent multiple toggles
            while (digitalRead(BUTTON_PIN) == LOW) {
                delay(10);
            }
        }
    }
    
    lastButtonState = reading;
}

// Modify updateAirQualityLed to respect ledEnabled state
void updateAirQualityLed() {
    if (!ledEnabled) {
        // Turn off all LEDs (LOW for common cathode/active-high)
        digitalWrite(rgbLed[0], LOW);
        digitalWrite(rgbLed[1], LOW);
        digitalWrite(rgbLed[2], LOW);
        return;
    }
    
    int pm25 = environmentData.pm2_5;
    
    // Find which AQI range we're in
    int i = 0;
    while (i < 3 && pm25 > AQI_LEVELS[i]) {
        i++;
    }
    
    RGB color;
    
    if (i == 0) {
        color = AQI_COLORS[0];
    } else {
        // Calculate how far we are between two levels (0.0 to 1.0)
        float ratio = (float)(pm25 - AQI_LEVELS[i-1]) / 
                     (float)(AQI_LEVELS[i] - AQI_LEVELS[i-1]);
        
        // Interpolate between colors
        color.r = AQI_COLORS[i-1].r + ratio * (AQI_COLORS[i].r - AQI_COLORS[i-1].r);
        color.g = AQI_COLORS[i-1].g + ratio * (AQI_COLORS[i].g - AQI_COLORS[i-1].g);
        color.b = AQI_COLORS[i-1].b + ratio * (AQI_COLORS[i].b - AQI_COLORS[i-1].b);
    }
    
    // For active-high LED:
    // - Input of 255 (full brightness) should give HIGH output (LED on)
    // - Input of 0 (no brightness) should give LOW output (LED off)
    digitalWrite(rgbLed[0], (color.r > 128) ? HIGH : LOW);  // R
    digitalWrite(rgbLed[1], (color.g > 128) ? HIGH : LOW);  // G
    digitalWrite(rgbLed[2], (color.b > 128) ? HIGH : LOW);  // B
}