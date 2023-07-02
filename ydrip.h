#ifndef WM_YDRIP_H
#define WM_YDRIP_H

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome::ydrip {
class Ydrip : public esphome::PollingComponent, public sensor::Sensor {
 public:
    Ydrip(uint16_t wakeup_pulses);
    void setup() override;
    void loop() override;
    void update() override;
    float get_setup_priority() const override;
    void dump_config() override;
 private:
    uint16_t wakeup_pulses;
};

} //namespace esphome::ydrip

#endif
