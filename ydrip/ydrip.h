#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_adc_cal.h"
#include "status_led.h"

namespace esphome {
namespace ydrip {

typedef enum {
    WATER_METER_PULSE_EVENT,
    WATER_METER_LEAK_EVENT,
    WATER_METER_CAL_DONE,
    WATER_METER_CAL_STEP,
    WATER_METER_USER_INPUT,
    WATER_METER_BOOT,
} water_meter_event_t;

typedef enum {
    STATE_UNCALIBRATED,
    STATE_CALIBRATED,
    STATE_CALIBRATING,
    STATE_FAILED_CALIBRATION,
    STATE_LEAK_DETECTED,
} water_meter_state_t;

typedef enum {
    SLOW_CLOCK_1024Hz = 1024,
    SLOW_CLOCK_512Hz = 512,
    SLOW_CLOCK_256Hz = 256,
    SLOW_CLOCK_128Hz = 128,
    SLOW_CLOCK_64Hz = 64,
    SLOW_CLOCK_32Hz = 32,
    SLOW_CLOCK_16Hz = 16,
    SLOW_CLOCK_8Hz = 8,
    SLOW_CLOCK_4Hz = 4,
    SLOW_CLOCK_2Hz = 2,
    SLOW_CLOCK_1Hz = 1,
} slow_clock_frequency_t;

struct app_config {
    //char ota_url[64];
    uint16_t valid_flag; // Validation flag to identify valid data
    uint8_t version;

    uint16_t pulse_alert_limit; 
    uint16_t leak_alert_limit;
    uint16_t slow_clock_freq;
    double low_freq_detect_thresh;

    // Calibration info
    bool is_calibrated;
    uint16_t min_voltage;
    uint16_t max_voltage;
    uint16_t target_voltage;

    // Register values
    uint16_t rheostat_value_non_inv_reg;
    uint16_t rheostat_value_inv_reg;
    uint8_t usage_count_1_reg;
    uint8_t usage_count_2_reg;
    uint8_t leak_alert_count_reg;
    uint8_t low_freq_detect_reg;

    // Analog comparator voltage thesholds
    uint8_t low_to_high_thresh;
    uint8_t high_to_low_thresh;

}__attribute__((packed));

struct CalibrationState {
    int target_voltage;
    int best_error;
    int min_voltage;
    int max_voltage;
    int low_non_inv;
    int low_inv;
    int high_non_inv;
    int high_inv;
    int current_sample;
    int attempts;
    bool active;
    int direction;
};

class YDripComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  // Constructor
  YDripComponent();

  void set_usage_sensor(sensor::Sensor *sensor) { this->usage_sensor_ = sensor; }
  void set_leak_sensor(binary_sensor::BinarySensor *sensor) { this->leak_sensor_ = sensor; }
  void set_battery_sensor(sensor::Sensor *sensor) { battery_sensor_ = sensor; }

  // Methods
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Configuration methods
  void set_default_sensor_values();
  esp_err_t load_calibration_data();
  void set_usage_alert_count(uint16_t count);
  void set_leak_alert_count(uint16_t count);
  void set_low_freq_leak_thresh(double threshold);
  void api_callback(float state);
  uint32_t read_adc_battery_voltage();

  // Methods for controlling slow clock frequency
  esp_err_t wm_set_slow_clock(slow_clock_frequency_t desired_freq);
  esp_err_t wm_get_slow_clock(slow_clock_frequency_t *slow_clock_freq);

  // Pulse and leak alert count methods
  esp_err_t wm_set_pulse_alert_count(uint16_t count);
  uint16_t wm_get_valid_pulse_alert_count(uint16_t count);
  esp_err_t wm_get_pulse_alert_count(uint16_t *value);
  esp_err_t wm_set_leak_alert_count(uint16_t count);
  esp_err_t wm_get_leak_alert_count(uint16_t *value);
  
  // Low frequency threshold methods
  esp_err_t wm_set_low_freq_thresh(double freq_thesh);
  esp_err_t wm_get_low_freq_thresh(double *freq_thresh);

  esp_err_t wm_reset_leak_counter();
  esp_err_t wm_reset_usage_counter();
  esp_err_t wm_set_power_duty_cycle(bool enable);

  esp_err_t wm_set_comparator_threshold(int high_mv, int low_mv);
  esp_err_t wm_set_comparator_threshold_reg(uint8_t high_thresh, uint8_t low_thresh);
  esp_err_t wm_get_comparator_threshold(int* high_mv, int* low_mv);
  esp_err_t wm_get_comparator_threshold_reg(uint8_t* high_thresh, uint8_t* low_thresh);
  uint16_t wm_comparator_threshold_reg_to_mv(uint8_t thresh);
  void test_set_get_comparator_threshold(int high_mv, int low_mv);

  void wm_update_app_config_from_device(void);

 protected:
  // Helper function to read data from I2C
  esp_err_t wm_write_reg16(uint8_t reg, uint16_t value);
  esp_err_t wm_write_reg(uint8_t reg, uint8_t* data, size_t size);
  esp_err_t wm_read_reg16(uint8_t reg, uint16_t* value);
  esp_err_t wm_read_reg(uint8_t reg, uint8_t* data, size_t size);
  esp_err_t update_register(uint8_t reg, uint8_t mask, uint8_t value);

  esp_err_t write_bytes_(uint8_t reg, const uint8_t* data, size_t size);
  esp_err_t read_bytes_(uint8_t reg, uint8_t* data, size_t size);

  esp_err_t save_app_config_to_fram();
  esp_err_t load_app_config_from_fram();
  esp_err_t invalidate_fram_app_config();

  esp_err_t fram_init();
  esp_err_t fram_page_write(uint16_t address, const uint8_t* data, size_t size);
  esp_err_t fram_sequential_read(uint16_t address, uint8_t* data, size_t size);

  // Sensors for reporting data
  sensor::Sensor *usage_sensor_{nullptr};
  binary_sensor::BinarySensor *leak_sensor_{nullptr};
  sensor::Sensor *ha_usage_sensor{nullptr};
  sensor::Sensor *battery_sensor_{nullptr};

  // Device configuration parameters
  uint16_t usage_alert_count_{0};
  uint16_t leak_alert_count_{0};
  double low_freq_leak_thresh_{0.0};

  esp_sleep_wakeup_cause_t wakeup_reason;

 private:
    static void water_meter_event_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);
    static void IRAM_ATTR pulse_isr_handler(void* arg);
    static void IRAM_ATTR leak_isr_handler(void* arg);
    static void IRAM_ATTR input_isr_handler(void* arg);
    static void sleep_timer_callback(void* arg);

    void publish_sensor_state();
    void publish_debug_state();
    void publish_battery_state();

    void on_api_connected();
    void update_led();

    void send_event(water_meter_event_t event);
    void process_events();
    void handle_event(water_meter_event_t event);
    void handle_uncalibrated_state(water_meter_event_t event);
    void handle_calibrating_state(water_meter_event_t event);
    void handle_calibrated_state(water_meter_event_t event);

    esp_err_t wm_start_calibration(int target_voltage);
    void calibration_init();
    void calibration_step();
    void calibration_step_work(int direction);
    esp_err_t calibration_done();
    
    void start_sleep_timer(int seconds);
    
    esp_err_t adc_init();

    struct app_config app_settings;
    esp_event_loop_handle_t water_meter_event_loop;
    water_meter_state_t current_state;

    esp_adc_cal_characteristics_t adc1_chars;

    float battery_percentage;

    bool usage_state_sent;
    bool leak_state_sent;
    bool sleep_scheduled;

    StatusLED status_led;
    CalibrationState calibration;
    QueueHandle_t event_queue;
};

}  // namespace ydrip
}  // namespace esphome

