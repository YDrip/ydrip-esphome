#ifndef WM_FRAM_H
#define WM_FRAM_H

#include <stdint.h>
#include <stddef.h>

namespace esphome::ydrip {

class FRAM {
 public:
    static const uint8_t MB85RC64TA_I2C_ADDR = 0x50;
    static const size_t DEVICE_ID_SIZE = 3;
    static const uint8_t RESERVED_SLAVE_ID = 0xF8;
    static const uint8_t SLEEP_CMD = 0x86;

    static const uint16_t FUJITSU_MANUF_ID = 0xA;
    static const uint8_t FUJITSU_DENSITY_CODE = 0x03;
    static const uint16_t MB85RC64TA_PROD_ID = 0x358;

    static const uint16_t MEMORY_SIZE = 8192;

    bool init();
    void write_byte(uint16_t address, uint8_t data);
    void page_write(uint16_t address, const uint8_t* data, size_t size);
    void erase(uint16_t address, size_t size);
    void current_address_read(uint8_t* data, size_t size);
    uint8_t random_read(uint16_t address);
    size_t sequential_read(uint16_t address, uint8_t* data, size_t size);
    bool sleep();
    bool wakeup();
    bool reset();
    bool get_device_id(uint16_t& manufacture_id, uint8_t& density_code, uint16_t& product_id);

 private:
    bool validate_addr_size(uint16_t address, size_t size);
};
} //namespace esphome::ydrip

#endif /* FRAM_H */


