#include "FanConfig.h"

// Initialize all static members
const int FanConfig::SPEED_MIN = 0;     // for PWM: off state (or digital off) depending on polarity
const int FanConfig::SPEED_MAX = 255;   // for PWM: full speed (or digital on) depending on polarity
const int FanConfig::PWM_CHANNEL = 0;
const int FanConfig::PWM_FREQ = 25000;
const int FanConfig::PWM_RESOLUTION = 8;
FanControlMethod FanConfig::controlMethod = FanControlMethod::DIGITAL;  // set to PWM or DIGITAL as required
FanPolarity FanConfig::polarity = FanPolarity::ACTIVE_LOW;             // set to ACTIVE_LOW or ACTIVE_HIGH
