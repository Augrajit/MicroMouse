#include "led.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel pixel(1, PIN_LED, NEO_GRB + NEO_KHZ800);

void led_init() {
    pixel.begin();
    pixel.setBrightness(40);   // keep brightness modest to save power
    pixel.setPixelColor(0, 0);
    pixel.show();
    Serial.println(F("[LED] NeoPixel initialised"));
}

void led_set(uint8_t r, uint8_t g, uint8_t b) {
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

// ── Calibration test index (set by solver) ───────────────
static uint8_t cal_test_index = 0;

void led_set_cal_test(uint8_t idx) { cal_test_index = idx; }

// ── Flash pattern to show which walls are detected ───────
// Called once — blocks briefly to blink the LED.
//   Left wall  → 1 blue flash
//   Front wall → 1 green flash
//   Right wall → 1 red flash
// Absent walls get a dim short blink instead.
void led_flash_walls(bool wl, bool wf, bool wr) {
    // Left
    led_set(0, 0, wl ? 200 : 20); delay(300);
    led_set(0, 0, 0);              delay(150);
    // Front
    led_set(0, wf ? 200 : 20, 0); delay(300);
    led_set(0, 0, 0);              delay(150);
    // Right
    led_set(wr ? 200 : 20, 0, 0); delay(300);
    led_set(0, 0, 0);              delay(150);
}

// ── Animated LED state indicator ─────────────────────────
// Called from loop() — uses millis() for non-blocking animation.
void led_update(RobotState state) {
    uint32_t now = millis();

    switch (state) {
        case STATE_IDLE: {
            // Slow white pulse (breathe)
            uint8_t brightness = (uint8_t)((sin(now / 1000.0f * PI) + 1.0f) * 80);
            led_set(brightness, brightness, brightness);
            break;
        }
        case STATE_CALIBRATE: {
            // Pulse the colour that matches the selected test
            uint8_t b = (uint8_t)((sin(now / 600.0f * PI) + 1.0f) * 90);
            switch (cal_test_index) {
                case 0: led_set(0,   0,   b);  break;  // Blue   — Forward 1 cell
                case 1: led_set(b,   b/2, 0);  break;  // Yellow — 360° rotation
                case 2: led_set(0,   b,   0);  break;  // Green  — ToF wall check
                case 3: led_set(b,   0,   0);  break;  // Red    — Square-up test
                case 4: led_set(0,   b,   b);  break;  // Cyan   — Motor self-test
                default: led_set(b,  0,   b);  break;  // Magenta fallback
            }
            break;
        }
        case STATE_EXPLORE:
            // Solid blue
            led_set(0, 0, 180);
            break;

        case STATE_RETURN:
            // Solid yellow
            led_set(180, 140, 0);
            break;

        case STATE_SPEEDRUN:
            // Solid green
            led_set(0, 180, 0);
            break;

        case STATE_DONE: {
            // Fast rainbow cycle then solid green
            static bool done_flash_complete = false;
            static uint32_t done_start = 0;

            if (!done_flash_complete) {
                if (done_start == 0) done_start = now;

                uint32_t elapsed = now - done_start;
                if (elapsed < 2500) {
                    // Rainbow flash — cycle hue rapidly (5 cycles in 2.5s)
                    uint16_t hue = (uint16_t)((elapsed * 13) % 65536);
                    uint32_t c = pixel.ColorHSV(hue, 255, 180);
                    pixel.setPixelColor(0, c);
                    pixel.show();
                } else {
                    done_flash_complete = true;
                }
            } else {
                led_set(0, 180, 0);   // solid green after celebration
            }
            break;
        }
    }
}

