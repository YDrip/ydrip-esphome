#include "driver/rmt.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <math.h>

#include "StatusLED.h"

#define RMT_CHANNEL RMT_CHANNEL_0

namespace esphome::ydrip {

static const char* TAG = "YDrip.StatusLED";

void StatusLED::init(gpio_num_t pin) {
    rmt_config_t config = { };
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL;
    config.gpio_num = pin;
    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.clk_div = 8;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // Send initial state to the LED (turn it off)
    this->send_rgb(0, 0, 0);
}

void StatusLED::send_rgb(uint8_t r, uint8_t g, uint8_t b) {
    rmt_item32_t items[24];

    // WS2812B LEDs expect GRB color order
    uint32_t rgb = (g << 16) | (r << 8) | b;

    for (int i = 0; i < 24; ++i) {
        if (rgb & (1 << (23 - i))) {
            // Bit is set, send T1H
            items[i].level0 = 1;
            items[i].duration0 = 9;
            items[i].level1 = 0;
            items[i].duration1 = 3;
        } else {
            // Bit is unset, send T0H
            items[i].level0 = 1;
            items[i].duration0 = 3;
            items[i].level1 = 0;
            items[i].duration1 = 9;
        }
    }


    rmt_write_items(RMT_CHANNEL, items, 24, true);
    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
}

#include <cmath>  // for fmod, floor

void hsv_to_rgb(float h, float s, float v, float& r, float& g, float& b) {
    float c = v * s;  // chroma
    float hp = h / 60.0;  // hue prime
    float x = c * (1 - std::abs(fmod(hp, 2) - 1));

    float r1, g1, b1;
    if (hp <= 1) {
        r1 = c;
        g1 = x;
        b1 = 0;
    } else if (hp <= 2) {
        r1 = x;
        g1 = c;
        b1 = 0;
    } else if (hp <= 3) {
        r1 = 0;
        g1 = c;
        b1 = x;
    } else if (hp <= 4) {
        r1 = 0;
        g1 = x;
        b1 = c;
    } else if (hp <= 5) {
        r1 = x;
        g1 = 0;
        b1 = c;
    } else {
        r1 = c;
        g1 = 0;
        b1 = x;
    }

    float m = v - c;

    r = r1 + m;
    g = g1 + m;
    b = b1 + m;
}

void StatusLED::breathe(int base_r, int base_g, int base_b) {
    static float intensity = 0.0f;
    static float direction = 0.01f;

    // Cycle the intensity between 0.0 and 1.0
    intensity += direction;
    if (intensity > 1.0f || intensity < 0.0f) {
        direction = -direction;
        intensity = std::max(0.0f, std::min(intensity, 1.0f));
        if (intensity == 0.0f) {
            cycle_complete = true;
        }
    }

    // Adjust the RGB values based on the intensity
    float r = base_r / 255.0f * intensity;
    float g = base_g / 255.0f * intensity;
    float b = base_b / 255.0f * intensity;

    this->send_rgb(r * 255, g * 255, b * 255);
}

bool StatusLED::is_cycle_complete() {
    if (cycle_complete) {
        cycle_complete = false;
        return true;
    } else {
        return false;
    }
}

void StatusLED::show() {
    static int hue = 0;
    hue = (hue + 1) % 360;
    float r, g, b;
    esphome::hsv_to_rgb(hue, 1.0f, 1.0f, r, g, b);
    this->send_rgb(r * 255, g * 255, b * 255);
}
}


