#include <Wire.h>
#include <time.h>

#include "esp_err.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

#include "Fram.h"

namespace esphome::ydrip {

static const char* const TAG = "YDrip.fram";

bool FRAM::init() {
    return true;
}

void FRAM::write_byte(uint16_t address, uint8_t data) {
    Wire.beginTransmission(FRAM::MB85RC64TA_I2C_ADDR);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    Wire.write(data);
    Wire.endTransmission();
}

void FRAM::page_write(uint16_t address, const uint8_t* data, size_t size) {
    if (!validate_addr_size(address, size)) {
        return;
    }

    // Maximum buffer size supported by the Wire library.
    // It could be larger on some platforms, but play it safe
    const uint8_t bufferSize = 32;

    while (size > 0) {
        uint8_t chunkSize = size < bufferSize ? size : bufferSize;

        Wire.beginTransmission(FRAM::MB85RC64TA_I2C_ADDR);
        Wire.write(address >> 8);   // High byte of address
        Wire.write(address & 0xFF); // Low byte of address

        for (uint8_t i = 0; i < chunkSize; i++) {
            Wire.write(data[i]);
        }

        Wire.endTransmission();

        address += chunkSize;
        data += chunkSize;
        size -= chunkSize;
    }
}

void FRAM::erase(uint16_t address, size_t size) {
    if (!validate_addr_size(address, size)) {
        return;
    }

    const uint8_t eraseValue = 0x00; // Value to erase the memory cells

    while (size > 0) {
        uint8_t chunkSize = size < 32 ? size : 32;
        Wire.beginTransmission(FRAM::MB85RC64TA_I2C_ADDR);
        Wire.write(address >> 8);   // High byte of address
        Wire.write(address & 0xFF); // Low byte of address
        for (uint8_t i = 0; i < chunkSize; i++) {
            Wire.write(eraseValue);
        }
        Wire.endTransmission();

        address += chunkSize;
        size -= chunkSize;
    }
}

void FRAM::current_address_read(uint8_t* data, size_t size) {
    Wire.requestFrom(FRAM::MB85RC64TA_I2C_ADDR, size);

    // Read data bytes
    for (size_t i = 0; i < size; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        } else {
            // Handle error or unexpected end of data
            // Break out of the loop or perform error handling as needed
            break;
        }
    }
}

uint8_t FRAM::random_read(uint16_t address) {
    Wire.beginTransmission(FRAM::MB85RC64TA_I2C_ADDR);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    Wire.endTransmission(false);

    uint8_t size = 0x01;
    Wire.requestFrom(FRAM::MB85RC64TA_I2C_ADDR, size);

    if (Wire.available()) {
        return Wire.read();
    } else {
        // Handle error or unexpected end of data
        // Return an appropriate default value or perform error handling as needed
        return 0;
    }
}

size_t FRAM::sequential_read(uint16_t address, uint8_t* data, size_t size) {
    Wire.beginTransmission(FRAM::MB85RC64TA_I2C_ADDR);
    Wire.write(address >> 8);   // High byte of address
    Wire.write(address & 0xFF); // Low byte of address
    Wire.endTransmission(false);

    // Maximum buffer size supported by the Wire library.
    // It could be larger on some platforms, but play it safe
    const uint8_t maxChunkSize = 32;

    size_t bytesRead = 0;
    while (bytesRead < size) {
        size_t remainingSize = size - bytesRead;
        size_t chunkSize = (remainingSize < maxChunkSize) ? remainingSize : maxChunkSize;

        Wire.requestFrom(FRAM::MB85RC64TA_I2C_ADDR, chunkSize);

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

bool FRAM::get_device_id(uint16_t& manufacture_id, uint8_t& density_code, uint16_t& product_id) {
    uint8_t dev_id[FRAM::DEVICE_ID_SIZE] = { 0 };
    Wire.beginTransmission(FRAM::RESERVED_SLAVE_ID >> 1);
    Wire.write((uint8_t)(FRAM::MB85RC64TA_I2C_ADDR << 1));
    uint8_t result = Wire.endTransmission(false);

    Wire.requestFrom(FRAM::RESERVED_SLAVE_ID >> 1, 3);
    for (size_t i = 0; i < FRAM::DEVICE_ID_SIZE; i++) {
        dev_id[i] = Wire.read();
        ESP_LOGD(TAG, "dev id %x", dev_id[i]);
    }

    manufacture_id = (dev_id[0] << 4) | (dev_id[1] >> 4);
    density_code = dev_id[1] & 0x0f;
    product_id = (density_code << 8) | (dev_id[2]);

    return true;
}

/* 
   NOT WORKING - Throws the following error. This could save 4uA.
   beginTransmission(): Unfinished Repeated Start transaction! Expected requestFrom, not beginTransmission! Clearing...
 */
bool FRAM::sleep() {
    Wire.beginTransmission(FRAM::RESERVED_SLAVE_ID >> 1);
    Wire.write((FRAM::MB85RC64TA_I2C_ADDR << 1) & 0xff);
    uint8_t result = Wire.endTransmission(false);
    if (result) {
        ESP_LOGE(TAG, "Failed to sleep FRAM %d", result);
        return false;
    }

    Wire.beginTransmission(FRAM::SLEEP_CMD >> 1);
    result = Wire.endTransmission(true);
    if (result) {
        ESP_LOGE(TAG, "Failed to sleep FRAM %d", result);
        return false;
    }

    return true;
}

bool FRAM::wakeup() {
    return true;
}

bool FRAM::validate_addr_size(uint16_t address, size_t size) {
    // Validate the address
    if (address >= FRAM::MEMORY_SIZE) {
        ESP_LOGE(TAG, "Invalid address: %04X", address);
        return false;
    }

    // Validate the size
    if (size == 0 || (address + size) > FRAM::MEMORY_SIZE) {
        ESP_LOGE(TAG, "Invalid size: %zu", size);
        return false;
    }

    return true;
}
}
