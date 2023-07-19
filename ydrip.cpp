//#include <Arduino.h>
#include <Wire.h>
#include <random>
#include <time.h>
#include <WiFi.h>

#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esp_sleep.h"
#include "esphome.h"
#include "driver/gpio.h"
#include "esphome/components/time/real_time_clock.h"

#include "common.h"
#include "ExtRtc.h"
#include "Fram.h"
#include "StatusLED.h"
#include "WaterMeterReader.h"
#include "ydrip.h"

namespace esphome::ydrip {

#define PULSE_PIN GPIO_NUM_8
#define LEAK_PIN GPIO_NUM_18
#define LED_PWR_PIN GPIO_NUM_1
#define LED_DATA_PIN GPIO_NUM_2
#define NUM_LEDS 1

#define PULSE_INTERRUPT_TYPE 1
#define LEAK_INTERRUPT_TYPE 2

#define MAX_LEAK_RESET_COUNT 5

// FRAM memory address
#define CONFIG_ADDR      1

#define SPLD_CLK_FREQ 32768
#define SPLD_CLK_DIV 512

#define LEAK_RESET_TIME 3600

static const char* const TAG = "YDrip.Sensor";
static QueueHandle_t interruptQueue;

Ydrip::Ydrip(uint16_t wakeup_pulse_count,
             uint8_t leak_alert_count,
             float low_freq_detect_thesh,
             sensor::Sensor* pulse_sensor,
             binary_sensor::BinarySensor* leak_sensor) :
    PollingComponent(1000),
    water_meter(SPLD_CLK_FREQ) {
    this->leak_sensor = leak_sensor;
    this->pulse_sensor = pulse_sensor;

    this->leak_alert_count = leak_alert_count;
    this->low_freq_detect_thesh = low_freq_detect_thesh;
    this->wakeup_pulse_count = wakeup_pulse_count;
}

void IRAM_ATTR handle_pulse_interrupt() {
  uint8_t interruptType = PULSE_INTERRUPT_TYPE;
  xQueueSendFromISR(interruptQueue, &interruptType, NULL);
}

void IRAM_ATTR handle_leak_interrupt() {
  uint8_t interruptType = LEAK_INTERRUPT_TYPE;
  xQueueSendFromISR(interruptQueue, &interruptType, NULL);
}

void Ydrip::setup() {
    wakeup_reason = esp_sleep_get_wakeup_cause();

    interruptQueue = xQueueCreate(10, sizeof(uint8_t));

    Wire.begin();
    fram.init();

    uint16_t manufacture_id;
    uint8_t density_code;
    uint16_t product_id;

    // Call get_device_id() and retrieve the device ID values
    fram.get_device_id(manufacture_id, density_code, product_id);

    // Check if the retrieved values match the expected values
    if (manufacture_id != FRAM::FUJITSU_MANUF_ID ||
        density_code != FRAM::FUJITSU_DENSITY_CODE ||
        product_id != FRAM::MB85RC64TA_PROD_ID) {
        ESP_LOGE(TAG, "Unrecognized FRAM");
        ESP_LOGE(TAG, "  manufacture_id 0x%x", manufacture_id);
        ESP_LOGE(TAG, "  density_code 0x%x", density_code);
        ESP_LOGE(TAG, "  product_id 0x%x", product_id);
    }

    ext_rtc.init();
    ext_rtc.ext_rtc_set_ext_clk_freq(PCF8563_32768HZ);
    ext_rtc.enable_ext_clock(true);

    water_meter.init();

    bool woke_from_sleep = wakeup_reason == ESP_SLEEP_WAKEUP_EXT1;
    ESP_LOGD(TAG, "wakeup reason %x", wakeup_reason);

    if (!woke_from_sleep) {
        water_meter.reset_usage_counter();
        water_meter.reset_leak_counter();
        water_meter.set_wake_up_count(this->wakeup_pulse_count);
        water_meter.set_leak_alert_count(this->leak_alert_count);
        water_meter.set_low_freq_thresh(this->low_freq_detect_thesh);
        water_meter.set_clock_div(SPLD_CLK_DIV);

        // Initialize default state for stored values after power up
        pulse_sensor->publish_state(0);
        leak_sensor->publish_initial_state(false);
        meter_state.total_pulses = 0;
        meter_state.leak_state = false;
        meter_state.leak_time = rtc->utcnow().timestamp;
        meter_state.update_time = rtc->utcnow().timestamp;
        save_state();
    } else {
        // Restore state after deep sleep
        restore_state();
        ESP_LOGD(TAG, "Restore state, pulse: %d  leak: %d",
                 meter_state.total_pulses, meter_state.leak_state);

        struct tm timeinfo;
        char time_string[64];
        gmtime_r(&meter_state.update_time, &timeinfo);
        strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", &timeinfo);
        ESP_LOGD(TAG, "Config timestamp: %s", time_string);

        gmtime_r(&meter_state.leak_time, &timeinfo);
        strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", &timeinfo);
        ESP_LOGD(TAG, "Last leak time: %s", time_string);

        time_t now = rtc->utcnow().timestamp;
        gmtime_r(&now, &timeinfo);
        strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", &timeinfo);
        ESP_LOGD(TAG, "Current time: %s", time_string);

        // Update state based on wakeup reason
        uint64_t wake_gpio = esp_sleep_get_ext1_wakeup_status();
        if (wake_gpio & BIT(PULSE_PIN)) {
            meter_state.total_pulses += this->wakeup_pulse_count;
            ESP_LOGD(TAG, "Pulse pin wakeup");
        }
        if (wake_gpio & BIT(LEAK_PIN)) {
            meter_state.leak_state = true;
            ESP_LOGD(TAG, "Leak pin wakeup");
        } else if (now != 0 &&\
                   (now - meter_state.leak_time) > LEAK_RESET_TIME)  {
            meter_state.leak_state = false;
            ESP_LOGD(TAG, "Reset leak status");
        }
        publish_leak_state();
        publish_pulse();
    }

    led.init(LED_DATA_PIN);
    gpio_set_direction(LED_PWR_PIN, GPIO_MODE_OUTPUT);
    // Power off LED
    gpio_set_level(LED_PWR_PIN, 0);

    // Initialize interrupts at the end to avoid false trigger when configuring digital logic
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handle_pulse_interrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(LEAK_PIN), handle_leak_interrupt, RISING);
}

uint32_t Ydrip::get_pulse_count() {
    return meter_state.total_pulses;
}

void Ydrip::save_state() {
    meter_state.update_time = rtc->utcnow().timestamp;
    fram.page_write(CONFIG_ADDR, (uint8_t*)&meter_state, sizeof(struct MeterState));
}

void Ydrip::restore_state() {
    fram.sequential_read(CONFIG_ADDR, (uint8_t*)&meter_state, sizeof(struct MeterState));
}

void Ydrip::publish_pulse() {
    pulse_sensor->publish_state(meter_state.total_pulses);
    save_state();
}

void Ydrip::publish_leak_state() {
    meter_state.leak_time = rtc->utcnow().timestamp;
    leak_sensor->publish_state(meter_state.leak_state);
    save_state();
    ESP_LOGD(TAG, "Publishing leak state: %d", meter_state.leak_state);
}

void Ydrip::loop() {
    // Publish water usage if notified over GPIO while ESP32 is awake
    uint8_t interruptType;
    if (xQueueReceive(interruptQueue, &interruptType, 0) == pdPASS) {
        switch (interruptType) {
          case PULSE_INTERRUPT_TYPE:
              meter_state.total_pulses += this->wakeup_pulse_count;
              publish_pulse();
              break;
          case LEAK_INTERRUPT_TYPE:
              meter_state.leak_state = true;
              publish_leak_state();
              break;
        }
    }
}

void Ydrip::register_write(int reg, int val) {
    ESP_LOGD(TAG, "reg write %x %x", reg, val);
    uint8_t value = (uint8_t)val;
    water_meter.write_bytes((uint8_t)reg, &value, sizeof(value));
}

void Ydrip::reset_leak_state() {
    meter_state.leak_state = false;
    publish_leak_state();
}

void Ydrip::reset_counter() {
    meter_state.total_pulses = 0;
    publish_pulse();
}

float Ydrip::get_setup_priority() const {
    return setup_priority::DATA;
}

void Ydrip::update() {
    //ESP_LOGD(TAG, "Number: %d", water_meter.get_pulse_count());
}

void Ydrip::dump_config() {
    water_meter.dump_config();
}

}  // namespace esphome::ydrip
