#ifndef WM_YDRIP_H
#define WM_YDRIP_H

#include <time.h>
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esp_sleep.h"

#include "ExtRtc.h"
#include "Fram.h"
#include "StatusLED.h"
#include "WaterMeterReader.h"

namespace esphome::ydrip {
struct MeterState {
    uint32_t total_pulses;

    uint8_t leak_state;
    time_t leak_time;
    time_t update_time;
}__attribute__((packed));

class Ydrip : public esphome::PollingComponent {
 public:
    Ydrip(uint16_t wakeup_pulse_count,
          uint8_t leak_alert_count,
          float low_freq_detect_thesh,
          sensor::Sensor* pulse_sensor,
          binary_sensor::BinarySensor* leak_sensor);
    void setup() override;
    void loop() override;
    void update() override;
    float get_setup_priority() const override;
    void dump_config() override;
    void reset_counter();
    void reset_leak_state();
    void start_calibration();
    void stop_calibration();
    uint32_t get_pulse_count();
    void register_write(int reg, int val);

    sensor::Sensor* pulse_sensor;
    binary_sensor::BinarySensor* leak_sensor;
    time::RealTimeClock *rtc;

 protected:
    void publish_pulse();
    void publish_leak_state();
    void save_state();
    void restore_state();

    uint16_t wakeup_pulse_count;
    uint16_t leak_alert_count;
    float low_freq_detect_thesh;

    FRAM fram;
    ExtRTC ext_rtc;
    StatusLED led;
    WaterMeterReader water_meter;
    esp_sleep_wakeup_cause_t wakeup_reason;

    MeterState meter_state;
};

} //namespace esphome::ydrip

#endif
