#ifndef WM_WATER_METER_READER_H
#define WM_WATER_METER_READER_H

#include "esp_err.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome::ydrip {

class WaterMeterReader {
 public:

    static const uint8_t WATER_METER_READER_ADDR = ((0x02 << 3) | 1);

    bool init();
    int get_pulse_count();
    void set_wake_up_count(uint16_t count);

 private:
    static esp_err_t update_register(uint8_t reg, uint8_t mask, uint8_t val);
    static size_t  write_bytes(uint8_t reg, const uint8_t* data, size_t len);
    static size_t read_bytes(uint8_t reg, uint8_t* data, size_t len);
    int is_big_endian();
    int read_16bit_number(uint8_t reg);
};
} //namespace esphome::ydrip
#endif // WATER_METER_READER


