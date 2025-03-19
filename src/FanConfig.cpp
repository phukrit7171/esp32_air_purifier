// Initialize static fan configuration parameters.
#include "FanConfig.h"

// Initialize all static members
const int FanConfig::SPEED_MIN = 0;     // Minimum PWM value (OFF state) or digital off depending on polarity
const int FanConfig::SPEED_MAX = 255;   // Maximum PWM value (full speed) or digital on depending on polarity
const int FanConfig::PWM_CHANNEL = 0;
const int FanConfig::PWM_FREQ = 25000;
const int FanConfig::PWM_RESOLUTION = 8;
FanControlMethod FanConfig::controlMethod = FanControlMethod::DIGITAL;  // Use DIGITAL or PWM control as required
FanPolarity FanConfig::polarity = FanPolarity::ACTIVE_LOW;             // ACTIVE_LOW or ACTIVE_HIGH
