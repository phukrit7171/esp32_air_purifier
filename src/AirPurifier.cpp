#include "AirPurifier.h"

struct RGB
{
    int r;
    int g;
    int b;
};

const int PinConfig::RGB_LED[3] = {4, 2, 15};
const int PinConfig::BUTTON = 5;
const int PinConfig::FAN = 13;
const int PinConfig::PM_SENSOR_RX = 16;
const int PinConfig::PM_SENSOR_TX = 17;

const int AirQualityConfig::PM25_GOOD = 12;
const int AirQualityConfig::PM25_MODERATE = 35;
const int AirQualityConfig::PM25_BAD = 55;
const float AirQualityConfig::PM_CALIBRATION_FACTOR = 1.0;

const unsigned long TimingConfig::SENSOR_READ_INTERVAL = 2000;
const unsigned long TimingConfig::DEBUG_OUTPUT_INTERVAL = 1000;
const unsigned long TimingConfig::BUTTON_CHECK_INTERVAL = 50;
const unsigned long TimingConfig::DISPLAY_INTERVAL = 5000;
const unsigned long TimingConfig::DEBOUNCE_DELAY = 50;
const unsigned long TimingConfig::HOLD_DURATION = 2000;

AirPurifier::AirPurifier()
    : lcd(0x27, 16, 2), pmSensor(PinConfig::PM_SENSOR_RX, PinConfig::PM_SENSOR_TX),
      systemEnabled(true), displayEnabled(true), ledEnabled(true), autoFanControl(true),
      currentDisplay(DisplayState::BME280), currentFanSpeed(0), 
      isPwmAttached(false), previousControlMethod(FanControlMethod::DIGITAL),
      lastSensorRead(0), lastDebugOutput(0), lastButtonCheck(0), lastDisplayChange(0),
      lastButtonState(HIGH), buttonPressStartTime(0), lastDebounceTime(0) {}

void AirPurifier::begin()
{
    lcd.init();
    lcd.clear();
    lcd.home();
    lcd.print("Initializing...");
    lcd.backlight();
    pmSensor.begin(9600);
    if (!bme.begin(0x76))
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1)
            ;
    }
    lcd.clear();
    lcd.home();
    lcd.noAutoscroll();
    lcd.print("Ready!");

    for (int i = 0; i < 3; i++)
    {
        pinMode(PinConfig::RGB_LED[i], OUTPUT);
        digitalWrite(PinConfig::RGB_LED[i], LOW); // Set RGB LED to OFF (active-high wiring)
    }

    pinMode(PinConfig::BUTTON, INPUT);
    pinMode(PinConfig::FAN, OUTPUT);
    
    // Ensure fan is OFF at startup using the current control method.
    if (FanConfig::controlMethod == FanControlMethod::DIGITAL) {
        // Digital mode: set fan state based on polarity (active low: off = HIGH, active high: off = LOW)
        digitalWrite(PinConfig::FAN, (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? HIGH : LOW);
        isPwmAttached = false;
    } else {
        // PWM mode
        ledcSetup(FanConfig::PWM_CHANNEL, FanConfig::PWM_FREQ, FanConfig::PWM_RESOLUTION);
        ledcAttachPin(PinConfig::FAN, FanConfig::PWM_CHANNEL);
        // Initialize PWM to off state based on polarity
        int offValue = (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? 255 : 0;
        ledcWrite(FanConfig::PWM_CHANNEL, offValue);
        isPwmAttached = true;
    }
    
    // Track the current fan control method.
    previousControlMethod = FanConfig::controlMethod;
    
    // Initial debug output for fan configuration
    Serial.print("Fan initialized: ");
    Serial.print(FanConfig::controlMethod == FanControlMethod::DIGITAL ? "DIGITAL" : "PWM");
    Serial.print(", Polarity: ");
    Serial.println(FanConfig::polarity == FanPolarity::ACTIVE_LOW ? "ACTIVE_LOW" : "ACTIVE_HIGH");
}

void AirPurifier::update()
{
    const unsigned long currentMillis = millis();

    if ((currentMillis - lastButtonCheck) >= TimingConfig::BUTTON_CHECK_INTERVAL)
    {
        lastButtonCheck = currentMillis;
        handleButton();
    }

    if (systemEnabled)
    {
        if ((currentMillis - lastSensorRead) >= TimingConfig::SENSOR_READ_INTERVAL)
        {
            lastSensorRead = currentMillis;
            readSensors();
            updateFan();
        }

        if ((currentMillis - lastDebugOutput) >= TimingConfig::DEBUG_OUTPUT_INTERVAL)
        {
            lastDebugOutput = currentMillis;
            debugOutput();
        }

        if (displayEnabled)
        {
            updateDisplay();
        }
        if (ledEnabled)
        {
            updateLED();
        }
    }
}

void AirPurifier::readSensors()
{
    float temp = bme.readTemperature();
    float pres = bme.readPressure();
    float hum = bme.readHumidity();

    if (isnan(temp) || isnan(pres) || isnan(hum))
    {
        Serial.println("Failed to read from BME280 sensor!");
        return;
    }

    envData.temperature = temp;
    envData.pressure = pres / 100.0F;
    envData.humidity = hum;
    envData.altitude = bme.readAltitude(1013.25);

    unsigned long startTime = millis();
    bool readSuccess = false;

    while (millis() - startTime < 1000)
    {
        if (pmSensor.read())
        {
            float pm1 = pmSensor.getPM1();
            float pm25 = pmSensor.getPM2_5();
            float pm10 = pmSensor.getPM10();

            if (pm1 >= 0 && pm25 >= 0 && pm10 >= 0)
            {
                envData.pm1_0 = round(pm1 * AirQualityConfig::PM_CALIBRATION_FACTOR);
                envData.pm2_5 = round(pm25 * AirQualityConfig::PM_CALIBRATION_FACTOR);
                envData.pm10 = round(pm10 * AirQualityConfig::PM_CALIBRATION_FACTOR);
                readSuccess = true;
                break;
            }
            else
            {
                Serial.println("Invalid PM sensor readings");
            }
        }
        yield();
    }

    if (!readSuccess)
    {
        Serial.println("PM Sensor read timeout");
    }
}

void AirPurifier::updateDisplay()
{
    unsigned long currentMillis = millis();

    if (currentMillis - lastDisplayChange >= TimingConfig::DISPLAY_INTERVAL)
    {
        lastDisplayChange = currentMillis;

        switch (currentDisplay)
        {
        case DisplayState::BME280:
            displayBME280Data();
            currentDisplay = DisplayState::PM_SENSOR;
            break;

        case DisplayState::PM_SENSOR:
            displayPMData();
            currentDisplay = DisplayState::BME280;
            break;
        }
    }
}

void AirPurifier::handleButton()
{
    int reading = digitalRead(PinConfig::BUTTON);

    if (reading != lastButtonState)
    {
        lastDebounceTime = millis();

        if (reading == LOW)
        {
            buttonPressStartTime = millis();
        }
        else if (reading == HIGH)
        {
            unsigned long pressDuration = millis() - buttonPressStartTime;
            if (pressDuration < TimingConfig::HOLD_DURATION)
            {
                if (systemEnabled)
                {
                    displayEnabled = !displayEnabled;
                    if (displayEnabled)
                    {
                        lcd.backlight();
                        lastDisplayChange = 0; // Reset timer to force updateDisplay() to refresh
                        updateDisplay();       // Force an immediate update
                    }
                    else
                    {
                        lcd.clear(); // Clear display before turning off
                        lcd.noBacklight();
                    }
                }
            }
        }
    }

    if (reading == LOW)
    {
        unsigned long pressDuration = millis() - buttonPressStartTime;
        if (pressDuration >= TimingConfig::HOLD_DURATION)
        {
            systemEnabled = !systemEnabled;

            if (!systemEnabled)
            {
                displayEnabled = false;
                ledEnabled = false;
                lcd.clear(); // Clear display before turning off
                lcd.noBacklight();
                // Turn off LEDs (LOW for active-high)
                digitalWrite(PinConfig::RGB_LED[0], LOW);
                digitalWrite(PinConfig::RGB_LED[1], LOW);
                digitalWrite(PinConfig::RGB_LED[2], LOW);
                // Detach PWM and turn off fan based on polarity
                ledcDetachPin(PinConfig::FAN);
                digitalWrite(PinConfig::FAN, (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? HIGH : LOW);
            }
            else
            {
                displayEnabled = true;
                ledEnabled = true;
                lcd.backlight();
                lastDisplayChange = 0; // Force immediate update after re-enabling display
                updateDisplay();       // Force an immediate update
                // Reattach appropriate control method to the fan
                if (FanConfig::controlMethod == FanControlMethod::PWM) {
                    ledcAttachPin(PinConfig::FAN, FanConfig::PWM_CHANNEL);
                    // Initialize PWM to off value
                    int offValue = (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? 255 : 0;
                    ledcWrite(FanConfig::PWM_CHANNEL, offValue);
                }
            }

            while (digitalRead(PinConfig::BUTTON) == LOW)
            {
                delay(10);
            }
        }
    }

    lastButtonState = reading;
}

void AirPurifier::updateLED()
{
    int pm25 = envData.pm2_5;

    if (!ledEnabled)
    {
        // Turn off all LEDs (LOW for active-high)
        digitalWrite(PinConfig::RGB_LED[0], LOW);
        digitalWrite(PinConfig::RGB_LED[1], LOW);
        digitalWrite(PinConfig::RGB_LED[2], LOW);
        return;
    }

    RGB color;

    // Set colors based on PM2.5 levels
    if (pm25 <= AirQualityConfig::PM25_GOOD)
    {
        // Good - Green
        color = {0, 255, 0};
    }
    else if (pm25 <= AirQualityConfig::PM25_MODERATE)
    {
        // Moderate - Blue
        color = {0, 0, 255};
    }
    else
    {
        // Bad - Red
        color = {255, 0, 0};
    }

    // For active-high LED, write HIGH to turn ON, LOW to turn OFF
    digitalWrite(PinConfig::RGB_LED[0], (color.r > 128) ? HIGH : LOW); // R
    digitalWrite(PinConfig::RGB_LED[1], (color.g > 128) ? HIGH : LOW); // G
    digitalWrite(PinConfig::RGB_LED[2], (color.b > 128) ? HIGH : LOW); // B
}

// No need for any definitions or forward declarations here as they are at the top of the file

void AirPurifier::updateFan()
{
    // Detect change in fan control method and reinitialize if required.
    if (previousControlMethod != FanConfig::controlMethod) {
        if (isPwmAttached) {
            ledcDetachPin(PinConfig::FAN);
            pinMode(PinConfig::FAN, OUTPUT);
            isPwmAttached = false;
        }
        previousControlMethod = FanConfig::controlMethod;
    }

    if (!systemEnabled || !autoFanControl)
    {
        // Set fan OFF per configured control mode.
        if (FanConfig::controlMethod == FanControlMethod::PWM)
        {
            if (!isPwmAttached) {
                ledcAttachPin(PinConfig::FAN, FanConfig::PWM_CHANNEL);
                isPwmAttached = true;
            }
            int offValue = (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? 255 : 0;
            ledcWrite(FanConfig::PWM_CHANNEL, offValue);
        }
        else
        {
            if (isPwmAttached) {
                ledcDetachPin(PinConfig::FAN);
                pinMode(PinConfig::FAN, OUTPUT);
                isPwmAttached = false;
            }
            digitalWrite(PinConfig::FAN, (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? HIGH : LOW);
        }
        return;
    }

    int maxPM = envData.pm1_0;
    if (envData.pm2_5 > maxPM)
        maxPM = envData.pm2_5;
    if (envData.pm10 > maxPM)
        maxPM = envData.pm10;

    if (FanConfig::controlMethod == FanControlMethod::PWM)
    {
        if (!isPwmAttached) {
            ledcAttachPin(PinConfig::FAN, FanConfig::PWM_CHANNEL);
            isPwmAttached = true;
        }
        int pwmValue = 0;
        if (maxPM <= AirQualityConfig::PM25_GOOD)
        {
            pwmValue = (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? 255 : 0;
        }
        else
        {
            if (FanConfig::polarity == FanPolarity::ACTIVE_LOW)
            {
                pwmValue = map(maxPM, 0, 300, 0, 255);
            }
            else
            {
                pwmValue = map(maxPM, 0, 300, 255, 0);
            }
            pwmValue = constrain(pwmValue, 0, 255);
        }
        ledcWrite(FanConfig::PWM_CHANNEL, pwmValue);
    }
    else
    {
        if (isPwmAttached) {
            ledcDetachPin(PinConfig::FAN);
            pinMode(PinConfig::FAN, OUTPUT);
            isPwmAttached = false;
        }
        int digitalValue;
        if (maxPM <= AirQualityConfig::PM25_GOOD)
        {
            digitalValue = (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? HIGH : LOW;
        }
        else
        {
            digitalValue = (FanConfig::polarity == FanPolarity::ACTIVE_LOW) ? LOW : HIGH;
        }
        digitalWrite(PinConfig::FAN, digitalValue);
    }
}

void AirPurifier::debugOutput()
{
    Serial.print("Temperature = ");
    Serial.print(envData.temperature);
    Serial.println(" *C");
    Serial.print("Humidity = ");
    Serial.print(envData.humidity);
    Serial.println(" %");
    Serial.print("Pressure = ");
    Serial.print(envData.pressure);
    Serial.println(" hPa");
    Serial.print("Altitude = ");
    Serial.print(envData.altitude);
    Serial.println(" m");
    Serial.print("PM1.0 = ");
    Serial.print(envData.pm1_0);
    Serial.println(" μg/m3");
    Serial.print("PM2.5 = ");
    Serial.print(envData.pm2_5);
    Serial.println(" μg/m3");
    Serial.print("PM10 = ");
    Serial.print(envData.pm10);
    Serial.println(" μg/m3");
    Serial.println();
}

void AirPurifier::displayBME280Data()
{
    lcd.clear();
    lcd.home();
    lcd.setCursor(0, 0);
    lcd.print("Temp  Hum   Pres");
    lcd.setCursor(0, 1);
    lcd.print(envData.temperature);
    lcd.setCursor(6, 1);
    lcd.print(envData.humidity);
    lcd.setCursor(12, 1);
    lcd.print(envData.pressure);
}

void AirPurifier::displayPMData()
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
    lcd.print(envData.pm1_0);
    lcd.setCursor(6, 1);
    lcd.print(envData.pm2_5);
    lcd.setCursor(12, 1);
    lcd.print(envData.pm10);
}
