#include <Wire.h>
#include <time.h>
#include "esp_err.h"
#include "esphome/core/log.h"
#include "WaterMeterReader.h"

namespace esphome::ydrip {

static const char* const TAG = "YDrip.WaterMeterReader";

#define PULSE_COUNT_REG 0x7E

bool WaterMeterReader::init() {
    return true;
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

// Helper function to check if the system is big-endian
int WaterMeterReader::is_big_endian() {
    union {
        uint32_t i;
        uint8_t c[4];
    } test = { 0x01020304 };

    return test.c[0] == 0x01;
}

#define CNT2_ADDR 0xAF
#define CNT4_ADDR 0xB8

void WaterMeterReader::set_wake_up_count(uint16_t count) {
    // Extract the lower 8 bits and set Counter 2 value
    uint8_t cnt_2_value = (count & 0xFF);  // Extract lower 8 bits
    write_bytes(CNT2_ADDR, &cnt_2_value, sizeof(cnt_2_value));

    // TODO: interrupts don't generate when cnt_4_value = 0
    // Extract the upper 8 bits and set Counter 4 value
    uint8_t cnt_4_value = 1; //((count >> 8) & 0xFF);  // Shift and extract upper 8 bits
    write_bytes(CNT4_ADDR, &cnt_4_value, sizeof(cnt_4_value));
}

int WaterMeterReader::get_pulse_count() {

    return this->read_16bit_number(PULSE_COUNT_REG);
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
