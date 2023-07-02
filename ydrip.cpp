//#include <Arduino.h>
#include <Wire.h>
#include <random>
#include <time.h>
#include <WiFi.h>

#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esp_sleep.h"
#include "esphome.h"
#include "driver/gpio.h"

#include "common.h"
#include "ExtRtc.h"
#include "Fram.h"
#include "StatusLED.h"
#include "WaterMeterReader.h"
#include "ydrip.h"

namespace esphome::ydrip {

#define WAKEUP_PIN GPIO_NUM_8
#define LED_PWR_PIN GPIO_NUM_1
#define LED_DATA_PIN GPIO_NUM_2
#define NUM_LEDS 1

ExtRTC rtc;
StatusLED led;
WaterMeterReader water_meter;

static const char* const TAG = "YDrip.Sensor";

Ydrip::Ydrip(uint16_t wakeup_pulses) : PollingComponent(1000) {
    this->wakeup_pulses = wakeup_pulses;
}

volatile bool interrupt_fired = false;
void IRAM_ATTR handle_interrupt() {
    interrupt_fired = true;
}

void Ydrip::setup() {
    //esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 1);
    attachInterrupt(digitalPinToInterrupt(WAKEUP_PIN), handle_interrupt, RISING);

    Wire.begin();
    FRAM mem;
    mem.init();

    uint16_t manufacture_id;
    uint8_t density_code;
    uint16_t product_id;

    // Call get_device_id() and retrieve the device ID values
    mem.get_device_id(manufacture_id, density_code, product_id);

    // Check if the retrieved values match the expected values
    if (manufacture_id != FRAM::FUJITSU_MANUF_ID ||
        density_code != FRAM::FUJITSU_DENSITY_CODE ||
        product_id != FRAM::MB85RC64TA_PROD_ID) {
        ESP_LOGE(TAG, "Unrecognized FRAM");
        ESP_LOGE(TAG, "  manufacture_id 0x%x", manufacture_id);
        ESP_LOGE(TAG, "  density_code 0x%x", density_code);
        ESP_LOGE(TAG, "  product_id 0x%x", product_id);
    }

    rtc.init();
    rtc.ext_rtc_set_ext_clk_freq(PCF8563_32768HZ);
    rtc.enable_ext_clock(true);

    ESP_LOGI(TAG, "Set wakeup_pulses to %d", this->wakeup_pulses);
    water_meter.set_wake_up_count(this->wakeup_pulses);

    gpio_set_direction(LED_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PWR_PIN, 0); // active low
    led.init(LED_DATA_PIN);
}

void Ydrip::loop() {
    if (interrupt_fired) {
        // Fix: there is a risk another interrupt can set this to true before
        // setting it to flase here resulting in a missed interrupt
        interrupt_fired = false;

        publish_state(this->wakeup_pulses);
    }
}

float Ydrip::get_setup_priority() const {
    return setup_priority::DATA;
}

void Ydrip::update() {
    ESP_LOGI(TAG, "Number: %d", water_meter.get_pulse_count());
}

void Ydrip::dump_config() {
    ESP_LOGCONFIG(TAG, "Empty custom sensor");
}

}  // namespace esphome::ydrip
