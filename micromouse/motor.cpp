#include "motor.h"
#include "config.h"

// ── LEDC channel numbers (ESP32-S3 Arduino Core v2.x) ───
// If using ESP32 Arduino Core v3.x, the API changes:
//   ledcAttach(pin, freq, resolution)  instead of ledcSetup + ledcAttachPin
//   ledcWrite(pin, duty)               instead of ledcWrite(channel, duty)
// A compile-time check is included below.

#define LEDC_CH_LEFT   0
#define LEDC_CH_RIGHT  1

void motor_init() {
    // Direction pins
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY,  OUTPUT);

    // LEDC PWM setup
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    // ESP32 Arduino Core v3.x API
    ledcAttach(PIN_MOTOR_L_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_R_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
    // ESP32 Arduino Core v2.x API
    ledcSetup(LEDC_CH_LEFT,  PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_RIGHT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_L_PWM, LEDC_CH_LEFT);
    ledcAttachPin(PIN_MOTOR_R_PWM, LEDC_CH_RIGHT);
#endif

    // Start with motors stopped, driver enabled
    motor_brake();
    digitalWrite(PIN_MOTOR_STBY, HIGH);
}

// Helper: write duty to the correct LEDC output
static inline void _pwm_write_left(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_MOTOR_L_PWM, duty);
#else
    ledcWrite(LEDC_CH_LEFT, duty);
#endif
}

static inline void _pwm_write_right(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_MOTOR_R_PWM, duty);
#else
    ledcWrite(LEDC_CH_RIGHT, duty);
#endif
}

void motor_set(int pwm_left, int pwm_right) {
    // Motors are physically wired in reverse — negate to correct
    pwm_left  = -pwm_left;
    pwm_right = -pwm_right;

    // ── Left motor ──
    if (pwm_left > 0) {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
    } else if (pwm_left < 0) {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
    } else {
        // Brake: both LOW → TB6612 short-brake
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
    }
    _pwm_write_left(constrain(abs(pwm_left), 0, PWM_MAX));

    // ── Right motor ──
    if (pwm_right > 0) {
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
    } else if (pwm_right < 0) {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
    } else {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
    }
    _pwm_write_right(constrain(abs(pwm_right), 0, PWM_MAX));
}

void motor_brake() {
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);
    _pwm_write_left(0);
    _pwm_write_right(0);
}

void motor_coast() {
    // Pull STBY LOW — driver outputs go high-Z, motors coast
    digitalWrite(PIN_MOTOR_STBY, LOW);
}
