#ifndef WM_STATUS_LED_H
#define WM_STATUS_LED_H

namespace esphome::ydrip {

struct LEDColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

class StatusLED {
 public:
    void init(gpio_num_t pin);
    void send_rgb(uint8_t r, uint8_t g, uint8_t b);
    void set_color(const LEDColor& color);
    void breathe(const LEDColor& color);
    void show();
    bool is_cycle_complete();
 private:
    bool cycle_complete = false;
};
}

#endif
