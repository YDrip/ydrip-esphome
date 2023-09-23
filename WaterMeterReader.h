#ifndef WM_WATER_METER_READER_H
#define WM_WATER_METER_READER_H

#include "esp_err.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome::ydrip {

class WaterMeterReader {
 public:

    static const uint8_t WATER_METER_READER_ADDR = ((0x02 << 3) | 1);

    WaterMeterReader(uint32_t main_clock_freq);
    esp_err_t init();
    int get_pulse_count();
    void set_wake_up_count(uint16_t count);
    void set_leak_alert_count(uint16_t count);
    void set_low_freq_thresh(float freq_thesh);
    void set_clock_div(uint32_t div);
    esp_err_t reset_leak_counter();
    esp_err_t reset_usage_counter();
    esp_err_t start_calibration();
    esp_err_t stop_calibration();
    void dump_config();

    static size_t write_bytes(uint8_t reg, const uint8_t* data, size_t len);

 private:
    static esp_err_t update_register(uint8_t reg, uint8_t mask, uint8_t val);

    static size_t read_bytes(uint8_t reg, uint8_t* data, size_t len);
    int read_16bit_number(uint8_t reg);

    uint16_t wakeup_count;
    uint16_t leak_alert_count;
    float freq_thesh;
    uint32_t main_clock_freq;
    uint32_t main_clock_div;
};
} //namespace esphome::ydrip
#endif // WATER_METER_READER


