#include "encoder.h"
#include "config.h"
#include <ESP32Encoder.h>

static ESP32Encoder encLeft;
static ESP32Encoder encRight;

void encoder_init() {
    // Use full quadrature decoding (count all 4 edges per cycle)
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    encLeft.attachFullQuad(PIN_ENC_L_A, PIN_ENC_L_B);
    encRight.attachFullQuad(PIN_ENC_R_A, PIN_ENC_R_B);

    encoder_reset();

    Serial.println(F("[ENC] Encoders initialised (PCNT full-quad)"));
    Serial.print(F("[ENC] COUNTS_PER_WHEEL_REV = "));
    Serial.println(COUNTS_PER_WHEEL_REV);
    Serial.print(F("[ENC] MM_PER_COUNT          = "));
    Serial.println(MM_PER_COUNT, 5);
}

int64_t encoder_get_left() {
    return encLeft.getCount();
}

int64_t encoder_get_right() {
    return encRight.getCount();
}

void encoder_reset() {
    encLeft.clearCount();
    encRight.clearCount();
}

float encoder_to_mm(int64_t counts) {
    return (float)counts * MM_PER_COUNT;
}
