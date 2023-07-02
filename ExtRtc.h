#ifndef EXTRTC_H
#define EXTRTC_H

namespace esphome::ydrip {
enum pcf8563_ext_clk_freq_t {
    PCF8563_32768HZ = 0,
    PCF8563_1024HZ,
    PCF8563_32HZ,
    PCF8563_1HZ,
};

class ExtRTC {
 public:

    static const uint8_t EXT_RTC_I2C_ADDR = 0x51;

    bool init();
    esp_err_t enable_ext_clock(bool enable);
    esp_err_t ext_rtc_set_ext_clk_freq(enum pcf8563_ext_clk_freq_t freq);
    esp_err_t set_time(const struct tm* time);
    esp_err_t get_time(struct tm* time);

 private:
    static uint8_t bcd2dec(uint8_t val);
    static uint8_t dec2bcd(uint8_t val);
    static esp_err_t update_register(uint8_t reg, uint8_t mask, uint8_t val);
    static size_t  write_bytes(uint8_t reg, const uint8_t* data, size_t len);
    static size_t read_bytes(uint8_t reg, uint8_t* data, size_t len);
};
} //namespace esphome::ydrip
#endif // EXTRTC_H


