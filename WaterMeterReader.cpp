#include <Wire.h>
#include <time.h>
#include "esp_err.h"
#include "esphome/core/log.h"
#include "WaterMeterReader.h"

namespace esphome::ydrip {

static const char* const TAG = "YDrip.WaterMeterReader";

#define CNT2_COUNTED_DATA_ADDR 0x7E
#define CNT4_COUNTED_DATA_ADDR 0x7F

// Max value of counter before reset
#define CNT2_DATA_LIMIT_ADDR 0xAF
#define CNT3_DATA_LIMIT_ADDR 0xB3
#define CNT4_DATA_LIMIT_ADDR 0xB8
#define CNT5_DATA_LIMIT_ADDR 0xBC
#define CNT6_DATA_LIMIT_ADDR 0xC1
#define CNT7_DATA_LIMIT_ADDR 0xC6

// Acts as another counter
#define PIPE_DELAY_SEL 0x9C

#define I2C_GPIO_ADDR 0x7A

#define SLOW_CNT_RESET_MASK       (1<<0)
#define USAGE_CNT_RESET_MASK      (1<<1)
#define PIPE_DELAY_OUT0_SEL_MASK  (0x0F)
#define PIPE_DELAY_OUT1_SEL_MASK  (0xF0)

#define SLOW_CNT_RESET_OFFSET      (0)
#define USAGE_CNT_RESET_OFFSET     (1)
#define PIPE_DELAY_OUT0_SEL_OFFSET (0)
#define PIPE_DELAY_OUT1_SEL_OFFSET (4)

#define DEFAULT_MAIN_CLK_DIV (512)

 // 5 flip flots = 2^5
 #define FLIP_FLOP_DIV 32;

#define TO_MS(value) ((value) * 1000)

WaterMeterReader::WaterMeterReader(uint32_t main_clock_freq) {
    this->main_clock_freq = main_clock_freq;
}

esp_err_t WaterMeterReader::init() {
    esp_err_t err = ESP_OK;
    this->main_clock_div = DEFAULT_MAIN_CLK_DIV;
    set_clock_div(DEFAULT_MAIN_CLK_DIV);
    return err;
}


// Function to read a 16-bit number from two consecutive addresses
int WaterMeterReader::read_16bit_number(uint8_t reg) {
    uint8_t data[2];

    this->read_bytes(reg, &data[0], sizeof(uint8_t));
    this->read_bytes(reg + 1, &data[1], sizeof(uint8_t));

    ESP_LOGD(TAG, "%d %d", data[0], data[1]);

    uint16_t number = ((data[0] + 0)) + (10 * (data[1] + 0));
    return number;
}

esp_err_t WaterMeterReader::reset_usage_counter() {
    esp_err_t err = ESP_OK;

    err |= update_register(I2C_GPIO_ADDR, USAGE_CNT_RESET_MASK, 0<<USAGE_CNT_RESET_OFFSET);
    err |= update_register(I2C_GPIO_ADDR, USAGE_CNT_RESET_MASK, 1<<USAGE_CNT_RESET_OFFSET);
    err |= update_register(I2C_GPIO_ADDR, USAGE_CNT_RESET_MASK, 0<<USAGE_CNT_RESET_OFFSET);

    return err;
}

esp_err_t WaterMeterReader::reset_leak_counter() {
    esp_err_t err = ESP_OK;

    err |= update_register(I2C_GPIO_ADDR, SLOW_CNT_RESET_MASK, 0<<SLOW_CNT_RESET_OFFSET);
    err |= update_register(I2C_GPIO_ADDR, SLOW_CNT_RESET_MASK, 1<<SLOW_CNT_RESET_OFFSET);
    err |= update_register(I2C_GPIO_ADDR, SLOW_CNT_RESET_MASK, 0<<SLOW_CNT_RESET_OFFSET);

    return err;
}

void WaterMeterReader::set_wake_up_count(uint16_t count) {
    uint8_t cnt_2_value = (count / 2) - 1;
    if (cnt_2_value == 0) {
      cnt_2_value = 1;
    }

    uint8_t cnt_4_value = (count / (cnt_2_value + 1)) - 1;
    if (cnt_4_value == 0) {
      cnt_4_value = 1;
    }
    this->wakeup_count = (cnt_2_value+1) * (cnt_4_value+1);

    write_bytes(CNT2_DATA_LIMIT_ADDR, &cnt_2_value, sizeof(cnt_2_value));
    write_bytes(CNT4_DATA_LIMIT_ADDR, &cnt_4_value, sizeof(cnt_4_value));
    ESP_LOGD(TAG, "Set wake up count: %d", count);
    ESP_LOGD(TAG, "Actual wake up count: %d", this->wakeup_count);
}

void WaterMeterReader::set_leak_alert_count(uint16_t count) {
    this->leak_alert_count = count;
    uint8_t cnt_6_value = (count / 2) - 1;
    if (cnt_6_value == 0) {
      cnt_6_value = 1;
    }

    uint8_t cnt_7_value = (count / (cnt_6_value + 1)) - 1;
    if (cnt_7_value == 0) {
      cnt_7_value = 1;
    }
    this->leak_alert_count = (cnt_6_value+1) * (cnt_7_value+1);

    write_bytes(CNT6_DATA_LIMIT_ADDR, &cnt_6_value, sizeof(cnt_6_value));
    write_bytes(CNT7_DATA_LIMIT_ADDR, &cnt_7_value, sizeof(cnt_7_value));
    ESP_LOGD(TAG, "Set leak alert count: %d", count);
    ESP_LOGD(TAG, "Actual leak alert count: %d", this->leak_alert_count);
}

void WaterMeterReader::set_low_freq_thresh(float freq_thesh) {
    uint32_t slow_clk = main_clock_freq / main_clock_div;
    uint8_t val = ((1/(freq_thesh*2.0)) * slow_clk) - 1;

    //float low_freq_detect = slow_clk_freq / (2.0*(val+1));

    write_bytes(CNT3_DATA_LIMIT_ADDR, &val, sizeof(val));

    this->freq_thesh = slow_clk / (2.0*(val+1));
    ESP_LOGD(TAG, "set_low_freq_thresh: %f val %d", this->freq_thesh, val);
}

void WaterMeterReader::set_clock_div(uint32_t div) {
    uint8_t val = 0;
    div /= FLIP_FLOP_DIV;
    val = ((div-1) << PIPE_DELAY_OUT1_SEL_OFFSET) & PIPE_DELAY_OUT1_SEL_MASK;
    val |= (div/2-1) & PIPE_DELAY_OUT0_SEL_MASK;
    write_bytes(PIPE_DELAY_SEL, &val, sizeof(val));
    ESP_LOGD(TAG, "set_clock_div: %d", val);
}

void WaterMeterReader::dump_config() {
    uint8_t wakeup_count_1;
    uint8_t wakeup_count_2;
    uint8_t leak_alert_count_1;
    uint8_t leak_alert_count_2;
    uint8_t slow_clock_limit;
    uint8_t pipe_delay_reg;
    uint8_t low_freq_detect_reg;

    ESP_LOGD(TAG, "Water Meter Reader Config");

    read_bytes(CNT2_DATA_LIMIT_ADDR, &wakeup_count_1, sizeof(wakeup_count_1));
    ESP_LOGD(TAG, "Usage counter 1 limit register val: %d", wakeup_count_1);

    read_bytes(CNT4_DATA_LIMIT_ADDR, &wakeup_count_2, sizeof(wakeup_count_2));
    ESP_LOGD(TAG, "Usage counter 2 limit register val: %d", wakeup_count_2);

    read_bytes(CNT6_DATA_LIMIT_ADDR, &leak_alert_count_1, sizeof(leak_alert_count_1));
    ESP_LOGD(TAG, "Leak alert counter 1 limit register val: %d", leak_alert_count_1);

    read_bytes(CNT7_DATA_LIMIT_ADDR, &leak_alert_count_2, sizeof(leak_alert_count_2));
    ESP_LOGD(TAG, "Leak alert counter 2 limit register val: %d", leak_alert_count_2);

    read_bytes(PIPE_DELAY_SEL, &pipe_delay_reg, sizeof(pipe_delay_reg));
    ESP_LOGD(TAG, "Slow clock register val: %d", pipe_delay_reg);

    read_bytes(CNT3_DATA_LIMIT_ADDR, &low_freq_detect_reg, sizeof(low_freq_detect_reg));
    ESP_LOGD(TAG, "Low freq detect register val: %d", low_freq_detect_reg);

    ESP_LOGD(TAG, "");

    uint32_t slow_clock_div = pipe_delay_reg & PIPE_DELAY_OUT0_SEL_MASK;
    slow_clock_div = 2*(slow_clock_div+1);
    slow_clock_div *= FLIP_FLOP_DIV;

    uint32_t slow_clk_freq = main_clock_freq/slow_clock_div;

    float low_freq_detect = slow_clk_freq / (2.0*(low_freq_detect_reg+1));

    ESP_LOGD(TAG, "Slow clock freq: %d Hz", slow_clk_freq);
    ESP_LOGD(TAG, "Usage wakeup count: %d", (wakeup_count_1+1)*(wakeup_count_2+1));
    ESP_LOGD(TAG, "Leak alert wake up count: %d", (leak_alert_count_1+1)*(leak_alert_count_2+1));
    ESP_LOGD(TAG, "Low freq detect threshold: %.2f Hz", low_freq_detect);
}

int WaterMeterReader::get_pulse_count() {

    return this->read_16bit_number(CNT2_COUNTED_DATA_ADDR);
}

size_t WaterMeterReader::write_bytes(uint8_t reg, const uint8_t* data, size_t size) {
    // Maximum buffer size supported by the Wire library.
    // It could be larger on some platforms, but play it safe
    const uint8_t bufferSize = 32;
    size_t bytes_writen = 0;

    while (size > 0) {
        uint8_t chunkSize = size < bufferSize ? size : bufferSize;

        Wire.beginTransmission(WATER_METER_READER_ADDR);
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

#define TIMEOUT_DURATION 500
size_t WaterMeterReader::read_bytes(uint8_t reg, uint8_t* data, size_t size) {
    Wire.beginTransmission(WATER_METER_READER_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    // Maximum buffer size supported by the Wire library.
    // It could be larger on some platforms, but play it safe
    const uint8_t maxChunkSize = 32;

    size_t bytesRead = 0;
    unsigned long startTime = millis();  // Start time for timeout
    bool requestError = false;  // Flag to track requestFrom() error
    while (bytesRead < size) {
        size_t remainingSize = size - bytesRead;
        size_t chunkSize = (remainingSize < maxChunkSize) ? remainingSize : maxChunkSize;

        if (Wire.requestFrom(WATER_METER_READER_ADDR, chunkSize) != chunkSize) {
            // Handle requestFrom() error
            ESP_LOGE(TAG, "Error in requestFrom()");
            requestError = true;
            break;
        }

        for (size_t i = 0; i < chunkSize; i++) {
            if (Wire.available()) {
                data[bytesRead] = Wire.read();
                bytesRead++;
                startTime = millis();  // Reset the timeout start time
            } else {
                // Handle error or unexpected end of data
                // Break out of the loop or perform error handling as needed
                ESP_LOGE(TAG, "Expected more bytes %d", i);
                break;
            }
        }

        // Check for timeout and exit the loop if necessary
        if (millis() - startTime > TIMEOUT_DURATION) {
            ESP_LOGE(TAG, "Timeout occurred");
            break;
        }
    }

    // Check for requestFrom() error and return 0 if an error occurred
    if (requestError) {
        return 0;
    }

    return bytesRead;
}

esp_err_t WaterMeterReader::update_register(uint8_t reg, uint8_t mask, uint8_t val) {
    Wire.beginTransmission(WATER_METER_READER_ADDR);
    Wire.write(reg);
    esp_err_t result = Wire.endTransmission(false);
    if (result != ESP_OK) {
        return result;
    }

    Wire.requestFrom(WATER_METER_READER_ADDR, (uint8_t)1);
    if (Wire.available()) {
        uint8_t oldValue = Wire.read();
        uint8_t newValue = (oldValue & mask) | val;

        Wire.beginTransmission(WATER_METER_READER_ADDR);
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
