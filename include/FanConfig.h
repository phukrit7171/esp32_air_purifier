enum class FanControlMethod {
    DIGITAL,
    PWM
};

enum class FanPolarity {
    ACTIVE_HIGH,
    ACTIVE_LOW
};

class FanConfig {
public:
    static const int SPEED_MIN; // for PWM: off state (or digital off) depending on polarity
    static const int SPEED_MAX; // for PWM: full speed (or digital on) depending on polarity
    static const int PWM_CHANNEL;
    static const int PWM_FREQ;
    static const int PWM_RESOLUTION;
    static FanControlMethod controlMethod;
    static FanPolarity polarity;
};