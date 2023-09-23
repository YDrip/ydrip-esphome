#include <Wire.h>
#include <time.h>
#include "esp_err.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

#include "ExtRtc.h"

namespace esphome::ydrip {

#define PCF8563TS_CONTROL_STATUS1_REG 0x00
#define PCF8563TS_CONTROL_STATUS2_REG 0x01
#define PCF8563TS_SECONDS_REG         0x02
#define PCF8563TS_MINUTES_REG         0x03
#define PCF8563TS_HOURS_REG           0x04
#define PCF8563TS_DAYS_REG            0x05
#define PCF8563TS_WEEKDAYS_REG        0x06
#define PCF8563TS_MONTHS_REG          0x07
#define PCF8563TS_YEARS_REG           0x08

#define PCF8563TS_ALARM_MINUTES_REG   0x09
#define PCF8563TS_ALARM_HOURS_REG     0x0A
#define PCF8563TS_ALARM_DAYS_REG      0x0B
#define PCF8563TS_ALARM_WEEKDAYS_REG  0x0C
#define PCF8563TS_CLKOUT_CONTROL_REG  0x0D
#define PCF8563TS_TIMER_CONTROL_REG   0x0E
#define PCF8563TS_TIMER_VALUE_REG     0x0F

#define RAM_SIZE 56

#define TIME_REG    0
#define CONTROL_REG 7
#define RAM_REG     8

#define CH_BIT      (1 << 7)

#define CH_MASK       0x7f
#define SECONDS_MASK  0x7f
#define MINUTES_MASK  0x7f
#define HOUR24_MASK   0x3f
#define DAYS_MASK     0x3f
#define WEEKDAYS_MASK 0x07
#define MONTHS_MASK   0x1f

#define CLKOUT_BIT    (1 << 7)
#define CLKOUT_MASK   0x7f

#define CLKFREQ_MASK  0xfc

static const char* const TAG = "YDrip.ExtRTC";

uint8_t ExtRTC::bcd2dec(uint8_t val) {
    return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t ExtRTC::dec2bcd(uint8_t val) {
    return ((val / 10) << 4) + (val % 10);
}

bool ExtRTC::init() {
    //this->set_i2c_address(EXT_RTC_I2C_ADDR);

    return true;
}

esp_err_t ExtRTC::enable_ext_clock(bool enable) {
    return update_register(PCF8563TS_CLKOUT_CONTROL_REG,
                           CLKOUT_MASK,
                           enable ? CLKOUT_BIT : 0);
}

esp_err_t ExtRTC::ext_rtc_set_ext_clk_freq(enum pcf8563_ext_clk_freq_t freq) {
    return update_register(PCF8563TS_CLKOUT_CONTROL_REG,
                           CLKFREQ_MASK,
                           freq);
}

esp_err_t ExtRTC::set_time(const struct tm* time) {
    if (!time) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[9] = { 0 };
    buf[PCF8563TS_SECONDS_REG] = dec2bcd(time->tm_sec) & SECONDS_MASK;
    buf[PCF8563TS_MINUTES_REG] = dec2bcd(time->tm_min) & MINUTES_MASK;
    buf[PCF8563TS_HOURS_REG] = dec2bcd(time->tm_hour) & HOUR24_MASK;
    buf[PCF8563TS_DAYS_REG] = dec2bcd(time->tm_mday) & DAYS_MASK;
    buf[PCF8563TS_WEEKDAYS_REG] = dec2bcd(time->tm_wday) & WEEKDAYS_MASK;
    buf[PCF8563TS_MONTHS_REG] = dec2bcd(time->tm_mon + 1) & MONTHS_MASK;
    buf[PCF8563TS_YEARS_REG] = dec2bcd(time->tm_year % 100); // 0 - 99

    // Set the century bit if year >= 2000
    buf[PCF8563TS_MONTHS_REG] &= ~CH_BIT;
    if (time->tm_year >= 100) {
        buf[PCF8563TS_MONTHS_REG] |= CH_BIT;
    }

    size_t bytes_to_write = sizeof(buf) - PCF8563TS_SECONDS_REG;
    size_t bytes_written = write_bytes(PCF8563TS_SECONDS_REG,
                                       buf + PCF8563TS_SECONDS_REG,
                                       bytes_to_write
                                      );
    if (bytes_written < bytes_to_write) {
        ESP_LOGE(TAG, "set_time failed - bytes_written: %d", bytes_written);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ExtRTC::get_time(struct tm* time) {
    if (!time) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[9] = { 0 };

    size_t bytes_to_read = sizeof(buf) - PCF8563TS_SECONDS_REG;
    size_t bytes_read = read_bytes(PCF8563TS_SECONDS_REG,
                                   buf + PCF8563TS_SECONDS_REG,
                                   bytes_to_read);
    if (bytes_read < bytes_to_read) {
        return ESP_FAIL;
    }

    time->tm_sec = bcd2dec(buf[PCF8563TS_SECONDS_REG] & SECONDS_MASK);
    time->tm_min = bcd2dec(buf[PCF8563TS_MINUTES_REG] & MINUTES_MASK);
    time->tm_hour = bcd2dec(buf[PCF8563TS_HOURS_REG] & HOUR24_MASK);
    time->tm_wday = bcd2dec(buf[PCF8563TS_WEEKDAYS_REG] & WEEKDAYS_MASK);
    time->tm_mday = bcd2dec(buf[PCF8563TS_DAYS_REG] & DAYS_MASK);
    time->tm_mon = bcd2dec(buf[PCF8563TS_MONTHS_REG] & MONTHS_MASK) - 1;

    // Assume it is 2000 if century bit is set
    uint16_t century = (buf[PCF8563TS_MONTHS_REG] & CH_BIT) ? 100 : 0;
    time->tm_year = bcd2dec(buf[PCF8563TS_YEARS_REG]) + century;

    return ESP_OK;
}

size_t ExtRTC::write_bytes(uint8_t reg, const uint8_t* data, size_t size) {
    // Maximum buffer size supported by the Wire library.
    // It could be larger on some platforms, but play it safe
    const uint8_t bufferSize = 32;
    size_t bytes_writen = 0;

    while (size > 0) {
        uint8_t chunkSize = size < bufferSize ? size : bufferSize;

        Wire.beginTransmission(EXT_RTC_I2C_ADDR);
        Wire.write(reg);   // High byte of address

        for (uint8_t i = 0; i < chunkSize; i++) {
            Wire.write(data[i]);
        }

        int ret = Wire.endTransmission();
        if (ret != 0) {
            // error
            ESP_LOGD(TAG, "write failed: %d", ret);
            break;
        }
        bytes_writen += chunkSize;

        reg += chunkSize;
        data += chunkSize;
        size -= chunkSize;
    }

    return bytes_writen;
}

size_t ExtRTC::read_bytes(uint8_t reg, uint8_t* data, size_t size) {
    Wire.beginTransmission(EXT_RTC_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    // Maximum buffer size supported by the Wire library.
    // It could be larger on some platforms, but play it safe
    const uint8_t maxChunkSize = 32;

    size_t bytesRead = 0;
    while (bytesRead < size) {
        size_t remainingSize = size - bytesRead;
        size_t chunkSize = (remainingSize < maxChunkSize) ? remainingSize : maxChunkSize;

        Wire.requestFrom(EXT_RTC_I2C_ADDR, chunkSize);

        for (size_t i = 0; i < chunkSize; i++) {
            if (Wire.available()) {
                data[bytesRead] = Wire.read();
                bytesRead++;
            } else {
                // Handle error or unexpected end of data
                // Break out of the loop or perform error handling as needed
                ESP_LOGE(TAG, "Expected more bytes %d", i);
                break;
            }
        }
    }
    return bytesRead;
}

esp_err_t ExtRTC::update_register(uint8_t reg, uint8_t mask, uint8_t val) {
    Wire.beginTransmission(EXT_RTC_I2C_ADDR);
    Wire.write(reg);
    esp_err_t result = Wire.endTransmission(false);
    if (result != ESP_OK) {
        return result;
    }

    Wire.requestFrom(EXT_RTC_I2C_ADDR, (uint8_t)1);
    if (Wire.available()) {
        uint8_t oldValue = Wire.read();
        uint8_t newValue = (oldValue & mask) | val;

        Wire.beginTransmission(EXT_RTC_I2C_ADDR);
        Wire.write(reg);
        Wire.write(newValue);
        result = Wire.endTransmission();
        if (result != ESP_OK) {
            return result;
        }
    } else {
        return ESP_ERR_TIMEOUT;  // Return timeout error if unable to read old value
    }

    return ESP_OK;
}
}
