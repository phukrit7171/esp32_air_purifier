# ESP32 Air Purifier Code Review

This document explains how the ESP32 Air Purifier codebase works, focusing on I/O operations and conditional logic.

## 1. Main System Structure

```cpp
// main.cpp
AirPurifier airPurifier;

void setup() {
  Serial.begin(115200);  // Opens serial port with 115200 baud rate
  airPurifier.begin();   // Initializes all subsystems
}

void loop() {
  airPurifier.update();  // Main loop calling all subsystem updates
}
```

The main program creates an AirPurifier instance and calls its update method in a continuous loop. This implements a simple task scheduler pattern.

## 2. Pin Configuration & Hardware Interface

```cpp
// AirPurifier.h
struct PinConfig {
    static const int RGB_LED[3];  // RGB LED pins for status indication
    static const int BUTTON;      // User input button 
    static const int FAN;         // Fan control pin
    static const int PM_SENSOR_RX;// Serial receive for PM sensor
    static const int PM_SENSOR_TX;// Serial transmit for PM sensor
};
```

This structure defines all hardware pins used in the system. Constants are used instead of hardcoded values to make the code more maintainable and readable.

## 3. Sensor Reading

### 3.1 PM Sensor Interface

```cpp
// PMSensor.cpp
bool PMSensor::read() {
    if (_serial.available() >= 32) {  // Checks if full data frame (32 bytes) is available
        uint8_t buffer[32];
        _serial.readBytes(buffer, 32);  // Reads 32 bytes from serial into buffer
        
        // Checks packet header (0x42, 0x4d) for valid data
        if (buffer[0] == 0x42 && buffer[1] == 0x4d) {
            // Extract PM values from specific buffer positions
            _pm1   = (buffer[10] << 8) | buffer[11];  // Combines high & low bytes
            _pm2_5 = (buffer[12] << 8) | buffer[13];
            _pm10  = (buffer[14] << 8) | buffer[15];
            return true;  // Valid reading obtained
        }
    }
    return false;  // No data or invalid data
}
```

This method:

1. Checks if a complete data frame is available (32 bytes)
2. Reads the data into a buffer
3. Verifies the data starts with the correct header (0x42, 0x4d)
4. Extracts 3 PM values by combining high and low bytes from specific positions
5. Returns success/failure status

### 3.2 Environmental Sensor Reading

```cpp
// AirPurifier.cpp
void AirPurifier::readSensors() {
    if (millis() - lastSensorRead > TimingConfig::SENSOR_READ_INTERVAL) {
        // Get data from BME280 sensor (temperature, humidity, pressure)
        envData.temperature = bme.readTemperature();
        envData.humidity = bme.readHumidity();
        envData.pressure = bme.readPressure() / 100.0F;
        envData.altitude = bme.readAltitude(1013.25);
        
        // Get PM sensor data if available
        if (pmSensor.read()) {
            // Apply calibration factor to raw PM values
            envData.pm1_0 = pmSensor.getPM1() * AirQualityConfig::PM_CALIBRATION_FACTOR;
            envData.pm2_5 = pmSensor.getPM2_5() * AirQualityConfig::PM_CALIBRATION_FACTOR;
            envData.pm10 = pmSensor.getPM10() * AirQualityConfig::PM_CALIBRATION_FACTOR;
        }
        
        lastSensorRead = millis();  // Update timestamp for interval tracking
    }
}
```

This method:

1. Uses time-based interval checking with `millis()` to avoid blocking delays
2. Reads environmental data from the BME280 sensor
3. Conditionally reads PM sensor data if available
4. Applies calibration factor to adjust raw PM values
5. Updates timestamp for next interval calculation

## 4. Fan Control System

```cpp
// FanConfig.h
enum class FanControlMethod {
    DIGITAL,  // Simple on/off control using digitalWrite
    PWM       // Variable speed control using PWM
};

enum class FanPolarity {
    ACTIVE_HIGH,  // Fan turns on with HIGH signal
    ACTIVE_LOW    // Fan turns on with LOW signal (inverted logic)
};
```

These enumerations define how the fan is controlled:

- Control method determines if simple digital or PWM control is used
- Polarity handles different types of fan circuits (regular or inverted logic)

```cpp
// AirPurifier.cpp
void AirPurifier::updateFan() {
    // Check if fan control method changed
    if (previousControlMethod != FanConfig::controlMethod) {
        if (FanConfig::controlMethod == FanControlMethod::PWM) {
            // Configure PWM for fan
            ledcSetup(FanConfig::PWM_CHANNEL, FanConfig::PWM_FREQ, FanConfig::PWM_RESOLUTION);
            ledcAttachPin(PinConfig::FAN, FanConfig::PWM_CHANNEL);
            isPwmAttached = true;
        } else {
            // Switch to digital mode
            if (isPwmAttached) {
                ledcDetachPin(PinConfig::FAN);
                isPwmAttached = false;
            }
            pinMode(PinConfig::FAN, OUTPUT);
        }
        previousControlMethod = FanConfig::controlMethod;
    }
    
    // Set fan speed based on current configuration
    if (systemEnabled) {
        if (autoFanControl) {
            // Auto mode: map PM2.5 level to fan speed
            currentFanSpeed = map(
                envData.pm2_5,
                AirQualityConfig::PM25_GOOD,
                AirQualityConfig::PM25_BAD,
                FanConfig::SPEED_MIN,
                FanConfig::SPEED_MAX
            );
        }
        
        // Apply fan speed based on control method
        if (FanConfig::controlMethod == FanControlMethod::PWM) {
            int pwmValue = currentFanSpeed;
            if (FanConfig::polarity == FanPolarity::ACTIVE_LOW) {
                pwmValue = FanConfig::SPEED_MAX - pwmValue;  // Invert PWM value for active low
            }
            ledcWrite(FanConfig::PWM_CHANNEL, pwmValue);
        } else {
            // Digital control (on/off)
            bool fanOn = currentFanSpeed > FanConfig::SPEED_MIN;
            if (FanConfig::polarity == FanPolarity::ACTIVE_LOW) {
                fanOn = !fanOn;  // Invert logic for active low
            }
            digitalWrite(PinConfig::FAN, fanOn ? HIGH : LOW);
        }
    } else {
        // System disabled - turn fan off
        if (FanConfig::controlMethod == FanControlMethod::PWM) {
            ledcWrite(FanConfig::PWM_CHANNEL, 
                     (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? 
                      FanConfig::SPEED_MAX : FanConfig::SPEED_MIN);
        } else {
            digitalWrite(PinConfig::FAN, 
                        (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? 
                         HIGH : LOW);
        }
    }
}
```

This method demonstrates:

1. Dynamic reconfiguration when fan control method changes
2. PWM setup and configuration using ESP32's LEDC peripheral
3. Conditional speed calculation based on air quality in auto mode
4. Polarity handling to support different fan driver circuits
5. System state awareness (enabled/disabled)

## 5. User Interface

### 5.1 Button Handling

```cpp
// AirPurifier.cpp
void AirPurifier::handleButton() {
    if (millis() - lastButtonCheck > TimingConfig::BUTTON_CHECK_INTERVAL) {
        int reading = digitalRead(PinConfig::BUTTON);  // Read current button state
        
        // If button state changed, reset debounce timer
        if (reading != lastButtonState) {
            lastDebounceTime = millis();
        }
        
        // Debounce: only process if stable for DEBOUNCE_DELAY
        if ((millis() - lastDebounceTime) > TimingConfig::DEBOUNCE_DELAY) {
            // Button pressed (assuming active LOW with pullup)
            if (reading == LOW) {
                // Check for long press
                if (!buttonPressStartTime) {
                    buttonPressStartTime = millis();
                }
                
                // Long press detected
                if (millis() - buttonPressStartTime > TimingConfig::HOLD_DURATION) {
                    // Toggle system on/off on long press
                    systemEnabled = !systemEnabled;
                    buttonPressStartTime = 0;  // Reset timer
                }
            } 
            // Button released
            else if (reading == HIGH && lastButtonState == LOW) {
                // Short press action (if not already handled as long press)
                if (buttonPressStartTime && 
                   (millis() - buttonPressStartTime < TimingConfig::HOLD_DURATION)) {
                    if (systemEnabled) {
                        // Cycle through modes on short press
                        autoFanControl = !autoFanControl;
                    }
                }
                buttonPressStartTime = 0;  // Reset timer
            }
        }
        
        lastButtonState = reading;  // Update button state
        lastButtonCheck = millis(); // Update timestamp
    }
}
```

This method demonstrates:

1. Non-blocking button polling with interval timing
2. Debounce implementation to filter out electrical noise
3. Both short and long press detection
4. State-dependent actions (different behavior when system on/off)
5. Timestamp management for timing operations

### 5.2 Display Management

```cpp
// AirPurifier.cpp
void AirPurifier::updateDisplay() {
    if (!displayEnabled) {
        if (lcd.isAttached()) {
            lcd.noDisplay();  // Turn off display
        }
        return;
    }
    
    // Display rotation logic
    if (millis() - lastDisplayChange > TimingConfig::DISPLAY_INTERVAL) {
        currentDisplay = (currentDisplay == DisplayState::BME280) ?
            DisplayState::PM_SENSOR : DisplayState::BME280;
        lastDisplayChange = millis();
    }
    
    // Display different info based on current display state
    if (currentDisplay == DisplayState::BME280) {
        displayBME280Data();
    } else {
        displayPMData();
    }
}

void AirPurifier::displayBME280Data() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(envData.temperature, 1);
    lcd.print("C");
    
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(envData.humidity, 1);
    lcd.print("%");
}

void AirPurifier::displayPMData() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PM2.5: ");
    lcd.print(envData.pm2_5);
    
    lcd.setCursor(0, 1);
    lcd.print("PM10: ");
    lcd.print(envData.pm10);
}
```

This code demonstrates:

1. State-based display with automatic rotation between screens
2. Time-based transitions without blocking delays
3. Conditional rendering based on current display state
4. Display enabling/disabling for power management

### 5.3 LED Status Indication

```cpp
// AirPurifier.cpp
void AirPurifier::updateLED() {
    if (!ledEnabled || !systemEnabled) {
        // Turn off LED
        analogWrite(PinConfig::RGB_LED[0], 0);
        analogWrite(PinConfig::RGB_LED[1], 0);
        analogWrite(PinConfig::RGB_LED[2], 0);
        return;
    }
    
    // Set LED color based on PM2.5 level
    if (envData.pm2_5 < AirQualityConfig::PM25_GOOD) {
        // Good air quality - Green
        analogWrite(PinConfig::RGB_LED[0], 0);    // R
        analogWrite(PinConfig::RGB_LED[1], 255);  // G
        analogWrite(PinConfig::RGB_LED[2], 0);    // B
    } 
    else if (envData.pm2_5 < AirQualityConfig::PM25_MODERATE) {
        // Moderate air quality - Yellow
        analogWrite(PinConfig::RGB_LED[0], 255);  // R
        analogWrite(PinConfig::RGB_LED[1], 255);  // G
        analogWrite(PinConfig::RGB_LED[2], 0);    // B
    } 
    else {
        // Bad air quality - Red
        analogWrite(PinConfig::RGB_LED[0], 255);  // R
        analogWrite(PinConfig::RGB_LED[1], 0);    // G
        analogWrite(PinConfig::RGB_LED[2], 0);    // B
    }
}
```

This method:

1. Uses PWM to control RGB LED brightness (analogWrite)
2. Changes color based on air quality conditions
3. Implements simple state awareness (enabled/disabled)

## 6. Timing System

```cpp
// AirPurifier.h
struct TimingConfig {
    static const unsigned long SENSOR_READ_INTERVAL;    // How often to read sensors
    static const unsigned long DEBUG_OUTPUT_INTERVAL;   // Serial debug message interval
    static const unsigned long BUTTON_CHECK_INTERVAL;   // Button polling rate
    static const unsigned long DISPLAY_INTERVAL;        // Screen rotation time
    static const unsigned long DEBOUNCE_DELAY;          // Button debounce time
    static const unsigned long HOLD_DURATION;           // Long-press detection time
};
```

The system uses non-blocking timing throughout:

1. Each subsystem has its own interval constants
2. The `millis()` function provides system uptime for timing calculations
3. Each operation tracks its last execution time
4. Comparison with current time and interval determines when to run again

This approach allows multiple operations to run concurrently without blocking each other.
