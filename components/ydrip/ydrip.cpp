#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esphome.h"
#include "esphome/core/log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_event.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "status_led.h"
#include "ydrip.h"

namespace esphome {
namespace ydrip {

static const char* const TAG = "ydrip";

#define SW_VERSION                   "v0.0.2"

#define WATER_METER_READER_ADDR      ((0x02 << 3) | 1)

// I2C register addresses
#define CNT0_DATA_LIMIT_ADDR         (0xAB)
#define CNT1_DATA_LIMIT_ADDR         (0xAE)
#define CNT2_DATA_LIMIT_ADDR         (0xB0)
#define CNT3_DATA_LIMIT_ADDR         (0xB2)
#define CNT4_DATA_LIMIT_ADDR         (0xB4)
#define CNT5_DATA_LIMIT_ADDR         (0xB6)
#define CNT6_DATA_LIMIT_ADDR         (0xB8)

#define PIPE_DELAY_CNT_ADDR          (0x6A)
#define PIPE_DELAY_CNT_MASK          (0xF0)
#define PIPE_DELAY_CNT_OFFSET        (0x04)

#define OSC0_OUT1_CLOCK_ADDR         (0x5B)
#define OSC0_OUT1_CLOCK_MASK         (0x38)
#define OSC0_OUT1_CLOCK_OFFSET       (0x03)

#define RHEOSTAT0_SET_VALUE_ADDR     (0xC0)
#define RHEOSTAT0_CURRENT_VALUE_ADDR (0xC2)
#define RHEOSTAT1_SET_VALUE_ADDR     (0xD0)
#define RHEOSTAT1_CURRENT_VALUE_ADDR (0xD2)

#define ACMP0_LOW_TO_HIGH_ADDR       (0x4D)
#define ACMP0_LOW_TO_HIGH_MASK       (0xFC)
#define ACMP0_LOW_TO_HIGH_OFFSET     (0x02)

#define ACMP0_HIGH_TO_LOW_ADDR       (0x4E)
#define ACMP0_HIGH_TO_LOW_MASK       (0x3F)
#define ACMP0_HIGH_TO_LOW_OFFSET     (0x00)

#define ACMP0_CONFIG_1               (0x4C)

#define I2C_GPIO_ADDR                (0x7C)
#define WATER_METER_READER_ADDR      ((0x02 << 3) | 1)

#define SLOW_CNT_RESET_MASK          (0x01)
#define USAGE_CNT_RESET_MASK         (0x02)
#define OFFSET_CAL_MASK              (0x04)
#define GAIN_CAL_MASK                (0x08)
#define DC_OFFSET_PROGRAM_MASK       (0x10)
#define GAIN_CAL_PROGRAM_MASK        (0x20)
#define DISABLE_PWR_DUTY_CYCLE_MASK  (0x40)
#define RESET_PIPE_MASK              (0x80)

#define SLOW_CNT_RESET_BIT_OFFSET    (0)
#define USAGE_CNT_RESET_BIT_OFFSET   (1)
#define OFFSET_CAL_BIT_OFFSET        (2)
#define GAIN_CAL_BIT_OFFSET          (3)
#define DC_OFFSET_CAL_BIT_PROGRAM    (4)
#define GAIN_CAL_BIT_PROGRAM         (5)
#define DISABLE_PWR_DUTY_CYCLE_BIT   (6)
#define RESET_PIPE_BIT               (7)

#define SPLD_CLK_FREQ                (32768)
#define OSC0_CLK_FREQ                (2048)

#define PULSE_PIN                    (GPIO_NUM_8)
#define LEAK_PIN                     (GPIO_NUM_18)
#define LED_DATA_PIN                 (GPIO_NUM_2)
#define LED_PWR_PIN                  (GPIO_NUM_1)
#define USER_INPUT_PIN               (GPIO_NUM_9)

#define APP_CONFIG_VERSION           (1)
#define APP_CONFIG_VALID_FLAG        (0xA5A5)

#define ESP_INTR_FLAG_DEFAULT        (0)

#define DEBOUNCE_TIME_MS             (300)

// Time to wait before going to sleep after boot. Wifi connection and all other
// processing must happen within this time.
#define SLEEP_TIMER_SEC              (60)

#define VBAT_DIV_ADC                 (ADC1_CHANNEL_5)
#define MAG_OUTPUT_ADC               (ADC1_CHANNEL_6)

#define CALIBRATION_VOLTAGE_TARGET   (1650)
#define CALIBRATION_TOLERANCE_MV     (2000)
#define CALIBRATION_STEP_DIVISOR     (124/2)

#define CALIBRATION_MAX_REHOSTAT_GAIN 1023
#define CALIBRATION_MAX_REHOSTAT_OFFSET 1023

// Calibration settings to capture min/max values of a 50 - 300 Hz sine wave

// Sampling interval (ms): 5 ms (200 samples/sec) provides sufficient frequency for peak detection 
// within each cycle of up to 300 Hz without excessive overhead.
#define CALIBRATION_SAMPLE_INTERVAL_MS 5
// Sampling duration (ms): 500 ms captures multiple cycles of both 50 Hz and 300 Hz, ensuring 
// accurate min/max values across the frequency range.
#define CALIBRATION_SAMPLING_DURATION_MS 500
// Total number of samples taken during the calibration period.
#define CALIBRATION_NUM_SAMPLES (CALIBRATION_SAMPLING_DURATION_MS / CALIBRATION_SAMPLE_INTERVAL_MS)

#define CALIBRATION_HYSTERESIS_MIN   (100)

#define CALIBRATION_OFFSET           (52)

#if CONFIG_IDF_TARGET_ESP32
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

#define FRAM_APP_CONFIG_ADDR         (0x03)

#define FRAM_MB85RC64TA_I2C_ADDR     (0x50)
#define FRAM_DEVICE_ID_SIZE          (3)
#define FRAM_RESERVED_SLAVE_ID       (0x7C)
#define FRAM_SLEEP_CMD               (0x86)

#define FRAM_FUJITSU_MANUF_ID        (0x0A)
#define FRAM_FUJITSU_DENSITY_CODE    (0x03)
#define FRAM_MB85RC64TA_PROD_ID      (0x358)

#define FRAM_MEMORY_SIZE             (8192)

typedef enum {
    OSC0_DIVIDER_1 = 0,
    OSC0_DIVIDER_2 = 1,
    OSC0_DIVIDER_4 = 2,
    OSC0_DIVIDER_3 = 3,
    OSC0_DIVIDER_8 = 4,
    OSC0_DIVIDER_12 = 5,
    OSC0_DIVIDER_24 = 6,
    OSC0_DIVIDER_64 = 7
} osc0_divider_t;

typedef enum {
    PIPE_DIVIDER_2 = 0,
    PIPE_DIVIDER_4 = 1,
    PIPE_DIVIDER_6 = 2,
    PIPE_DIVIDER_8 = 3,
    PIPE_DIVIDER_10 = 4,
    PIPE_DIVIDER_12 = 5,
    PIPE_DIVIDER_14 = 6,
    PIPE_DIVIDER_16 = 7,
    PIPE_DIVIDER_18 = 8,
    PIPE_DIVIDER_20 = 9,
    PIPE_DIVIDER_22 = 10,
    PIPE_DIVIDER_24 = 11,
    PIPE_DIVIDER_26 = 12,
    PIPE_DIVIDER_28 = 13,
    PIPE_DIVIDER_30 = 14,
    PIPE_DIVIDER_32 = 15,
} pipe_delay_divider_t;

const LEDColor COLOR_OFF = {0, 0, 0};
const LEDColor COLOR_YELLOW = {255, 255, 0};
const LEDColor COLOR_RED = {255, 0, 0};
const LEDColor COLOR_GREEN = {0, 255, 0};
const LEDColor COLOR_BLUE = {0, 0, 255};
const LEDColor COLOR_ORANGE = {255, 165, 0};
const LEDColor COLOR_PURPLE = {128, 0, 128};
const LEDColor COLOR_CYAN = {0, 255, 255};
const LEDColor COLOR_MAGENTA = {255, 0, 255};
const LEDColor COLOR_LIME = {0, 255, 128};
const LEDColor COLOR_PINK = {255, 105, 180};
const LEDColor COLOR_LIGHT_BLUE = {173, 216, 230};
const LEDColor COLOR_WHITE = {255, 255, 255};
const LEDColor COLOR_LIGHT_GREEN = {0, 128, 0};
const LEDColor COLOR_TURQUOISE = {64, 224, 208};

// Function to get the numerical value of the OSC0 divider
static int get_osc0_divider_value(osc0_divider_t value) {
    switch (value) {
      case OSC0_DIVIDER_1:
          return 1;
      case OSC0_DIVIDER_2:
          return 2;
      case OSC0_DIVIDER_3:
          return 3;
      case OSC0_DIVIDER_4:
          return 4;
      case OSC0_DIVIDER_8:
          return 8;
      case OSC0_DIVIDER_12:
          return 12;
      case OSC0_DIVIDER_24:
          return 24;
      case OSC0_DIVIDER_64:
          return 64;
      default:
          return -1;  // Error case
    }
}

// Declare the custom event base
ESP_EVENT_DEFINE_BASE(WATER_METER_EVENTS);

// Function to get the numerical value of the PIPE delay divider
static int get_pipe_delay_divider_value(pipe_delay_divider_t value) {
    switch (value) {
      case PIPE_DIVIDER_2:
          return 2;
      case PIPE_DIVIDER_4:
          return 4;
      case PIPE_DIVIDER_6:
          return 6;
      case PIPE_DIVIDER_8:
          return 8;
      case PIPE_DIVIDER_10:
          return 10;
      case PIPE_DIVIDER_12:
          return 12;
      case PIPE_DIVIDER_14:
          return 14;
      case PIPE_DIVIDER_16:
          return 16;
      case PIPE_DIVIDER_18:
          return 18;
      case PIPE_DIVIDER_20:
          return 20;
      case PIPE_DIVIDER_22:
          return 22;
      case PIPE_DIVIDER_24:
          return 24;
      case PIPE_DIVIDER_26:
          return 26;
      case PIPE_DIVIDER_28:
          return 28;
      case PIPE_DIVIDER_30:
          return 30;
      case PIPE_DIVIDER_32:
          return 32;
      default:
          return -1;  // Error case
    }
}

RTC_DATA_ATTR uint64_t rtc_usage_counter = 0;

YDripComponent::YDripComponent() : PollingComponent(1000) {

}
void IRAM_ATTR YDripComponent::pulse_isr_handler(void* arg) {
    if (!arg) {
        return;
    }

    YDripComponent* instance = static_cast<YDripComponent*>(arg);
    esp_err_t err = esp_event_isr_post_to(instance->water_meter_event_loop, WATER_METER_EVENTS, WATER_METER_PULSE_EVENT, NULL, 0, NULL);
}

void IRAM_ATTR YDripComponent::leak_isr_handler(void* arg) {
    if (!arg) {
        return;
    }

    YDripComponent* instance = static_cast<YDripComponent*>(arg);
    esp_err_t err = esp_event_isr_post_to(instance->water_meter_event_loop, WATER_METER_EVENTS, WATER_METER_LEAK_EVENT, NULL, 0, NULL);
}

void IRAM_ATTR YDripComponent::input_isr_handler(void* arg) {
    if (!arg) {
        return;
    }

    static volatile uint64_t last_press_time = 0;
    YDripComponent* instance = static_cast<YDripComponent*>(arg);

    // Get the current time in microseconds
    uint64_t current_time = esp_timer_get_time();  // Microseconds

    // Check if the debounce time has passed since the last press
    if (current_time - last_press_time > DEBOUNCE_TIME_MS * 1000) {
        last_press_time = current_time;
        esp_err_t err = esp_event_isr_post_to(instance->water_meter_event_loop, WATER_METER_EVENTS, WATER_METER_USER_INPUT, NULL, 0, NULL);
    }
}

esp_err_t YDripComponent::adc_init() {
    esp_err_t ret;

    // Check if the Two Point or Vref calibration values are burned into eFuse
    ret = esp_adc_cal_check_efuse(ADC_CALI_SCHEME);
    if (ret == ESP_OK) {
        // Characterize ADC1
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

        // Configure the ADC width
        ret = adc1_config_width((adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure ADC width, error: %d", ret);
            return ret;
        }

        ret = adc1_config_channel_atten(VBAT_DIV_ADC, ADC_ATTEN_DB_12);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure VBAT_DIV_ADC attenuation, error: %d", ret);
            return ret;
        }

        ret = adc1_config_channel_atten(MAG_OUTPUT_ADC, ADC_ATTEN_DB_12);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure MAG_OUTPUT_ADC attenuation, error: %d", ret);
            return ret;
        }

    } else {
        ESP_LOGE(TAG, "Failed to check eFuse for ADC calibration values. Default calibration values will be used. error: %d", ret);
    }

    return ret;
}

void YDripComponent::test_set_get_comparator_threshold(int high_mv, int low_mv) {
    esp_err_t err;

    // Set the comparator thresholds
    err = wm_set_comparator_threshold(high_mv, low_mv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set thresholds. Error: %d", err);
        return;
    }

    // Get the comparator thresholds
    int read_high_mv = 0;
    int read_low_mv = 0;

    err = wm_get_comparator_threshold(&read_high_mv, &read_low_mv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get thresholds. Error: %d", err);
        return;
    }

    // Print and compare the set and returned values
    ESP_LOGI(TAG, "Set High: %d mV, Got High: %d mV", high_mv, read_high_mv);
    ESP_LOGI(TAG, "Set Low: %d mV, Got Low: %d mV", low_mv, read_low_mv);

    if (high_mv != read_high_mv || low_mv != read_low_mv) {
        ESP_LOGE(TAG, "Test failed: Set high_mv = %d mV, Got high_mv = %d mV", high_mv, read_high_mv);
        ESP_LOGE(TAG, "Test failed: Set low_mv = %d mV, Got low_mv = %d mV", low_mv, read_low_mv);
    } else {
        ESP_LOGI(TAG, "Test passed for high_mv: %d mV and low_mv: %d mV", high_mv, low_mv);
    }
}

void YDripComponent::set_default_sensor_values() {
    uint16_t valid_pulse_alert_count = wm_get_valid_pulse_alert_count(usage_alert_count_);
    uint16_t current_pulse_alert_count;
    wm_get_pulse_alert_count(&current_pulse_alert_count);
    // Check and set pulse alert count if different
    if (valid_pulse_alert_count != current_pulse_alert_count) {
        ESP_LOGI(TAG, "Setting pulse alert count to %d", usage_alert_count_);
        wm_set_pulse_alert_count(usage_alert_count_);
    } else {
        ESP_LOGI(TAG, "No need to set pulse alert count, values are the same.");
    }

    uint16_t current_leak_alert_count;
    wm_get_leak_alert_count(&current_leak_alert_count);
    // Check and set leak alert count if different
    if (leak_alert_count_ != current_leak_alert_count) {
        ESP_LOGI(TAG, "Setting leak alert count to %d", leak_alert_count_);
        wm_set_leak_alert_count(leak_alert_count_);
    } else {
        ESP_LOGI(TAG, "No need to set leak alert count, values are the same.");
    }

    // Set low frequency leak threshold if different
    double current_freq_thresh;
    wm_get_low_freq_thresh(&current_freq_thresh);
    if (low_freq_leak_thresh_ != current_freq_thresh) {
        ESP_LOGI(TAG, "Setting low frequency threshold to %f", low_freq_leak_thresh_);
        wm_set_low_freq_thresh(low_freq_leak_thresh_);
    } else {
        ESP_LOGI(TAG, "No need to set low frequency threshold, values are the same.");
    }

    // Set slow clock frequency if different
    slow_clock_frequency_t current_slow_clock_freq;
    wm_get_slow_clock(&current_slow_clock_freq);
    if (current_slow_clock_freq != SLOW_CLOCK_32Hz) {
        ESP_LOGI(TAG, "Setting slow clock frequency to %d", SLOW_CLOCK_32Hz);
        wm_set_slow_clock(SLOW_CLOCK_32Hz);
    } else {
        ESP_LOGI(TAG, "No need to set slow clock frequency, values are the same.");
    }
}

esp_err_t YDripComponent::load_calibration_data() {
    // Retrieve current values before writing to avoid unnecessary writes
    uint16_t current_rheostat0_value, current_rheostat1_value;
    uint16_t current_pulse_alert_limit, current_leak_alert_limit;
    double current_low_freq_thresh;
    slow_clock_frequency_t current_slow_clock_freq;
    uint8_t current_low_to_high_thresh, current_high_to_low_thresh;
     
    // Read current values for comparison
    wm_read_reg16(RHEOSTAT0_SET_VALUE_ADDR, &current_rheostat0_value);
    wm_read_reg16(RHEOSTAT1_SET_VALUE_ADDR, &current_rheostat1_value);
    wm_get_pulse_alert_count(&current_pulse_alert_limit);
    wm_get_leak_alert_count(&current_leak_alert_limit);
    wm_get_low_freq_thresh(&current_low_freq_thresh);
    wm_get_slow_clock(&current_slow_clock_freq);
    wm_get_comparator_threshold_reg(&current_low_to_high_thresh, &current_high_to_low_thresh);

    // Only write values if they are different from current values
    bool write_error = false;

    if (current_rheostat0_value != app_settings.rheostat_value_non_inv_reg) {
        if (wm_write_reg16(RHEOSTAT0_SET_VALUE_ADDR, app_settings.rheostat_value_non_inv_reg) != ESP_OK) {
            write_error = true;
        }
    }

    if (current_rheostat1_value != app_settings.rheostat_value_inv_reg) {
        if (wm_write_reg16(RHEOSTAT1_SET_VALUE_ADDR, app_settings.rheostat_value_inv_reg) != ESP_OK) {
            write_error = true;
        }
    }

    if (current_pulse_alert_limit != app_settings.pulse_alert_limit) {
        if (wm_set_pulse_alert_count(app_settings.pulse_alert_limit) != ESP_OK) {
            write_error = true;
        }
    }

    if (current_leak_alert_limit != app_settings.leak_alert_limit) {
        if (wm_set_leak_alert_count(app_settings.leak_alert_limit) != ESP_OK) {
            write_error = true;
        }
    }

    if (current_low_freq_thresh != app_settings.low_freq_detect_thresh) {
        if (wm_set_low_freq_thresh(app_settings.low_freq_detect_thresh) != ESP_OK) {
            write_error = true;
        }
    }

    if (current_slow_clock_freq != static_cast<slow_clock_frequency_t>(app_settings.slow_clock_freq)) {
        if (wm_set_slow_clock(static_cast<slow_clock_frequency_t>(app_settings.slow_clock_freq)) != ESP_OK) {
            write_error = true;
        }
    }

    if (current_low_to_high_thresh != app_settings.low_to_high_thresh ||
        current_high_to_low_thresh != app_settings.high_to_low_thresh) {
        if (wm_set_comparator_threshold_reg(app_settings.low_to_high_thresh, app_settings.high_to_low_thresh)) {
            write_error = true;
        }
    }

    // Check if any write operation failed
    if (write_error) {
        return ESP_FAIL;
    } else {
        return ESP_OK;
    }
}
void YDripComponent::setup() {
    ESP_LOGI(TAG, "Setting up YDrip...");
    ESP_LOGI(TAG, "Version %s", SW_VERSION);

    event_queue = xQueueCreate(16, sizeof(water_meter_event_t));
    if (event_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return;
    }

    fram_init();

    current_state = STATE_UNCALIBRATED;
    battery_percentage = 50;

    wakeup_reason = esp_sleep_get_wakeup_cause();
    bool woke_from_sleep = false;
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1 || wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        woke_from_sleep = true;
    }
    ESP_LOGI(TAG, "Wakeup reason %x", wakeup_reason);

    
    status_led.init(LED_DATA_PIN);
    gpio_set_direction(LED_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PWR_PIN, 0); // Power on LED
    cal_led_color = COLOR_GREEN;

    if (!this->usage_sensor_) {
        ESP_LOGE(TAG, "No usage sensor");
        return;
    }
    if (!this->leak_sensor_) {
        ESP_LOGE(TAG, "No leak sensor");
        return;
    }
    if (!this->battery_sensor_) {
        ESP_LOGE(TAG, "No battery sensor");
        return;
    }

    // HACK: Disable power saving for HW v0.4 due to hardware issues
    wm_set_power_duty_cycle(false);
    // disaable ACMP sampling mode since it doesn't work without duty cycling
    uint8_t acmp0_config_value;
    read_bytes_(ACMP0_CONFIG_1, &acmp0_config_value, sizeof(acmp0_config_value));
    if (acmp0_config_value & 0x4) {
        ESP_LOGI(TAG, "Disabling ACMP0 sampling mode");
        update_register(ACMP0_CONFIG_1, 0x4, 0x0);
    }
    read_bytes_(ACMP0_CONFIG_1, &acmp0_config_value, sizeof(acmp0_config_value));
    ESP_LOGD(TAG, "ACMP0 register %x", acmp0_config_value);
    
    memset(&app_settings, 0, sizeof(app_settings));

    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        // User input wake up
        current_state = STATE_UNCALIBRATED;
        invalidate_fram_app_config();
        app_settings.is_calibrated = false;
        ESP_LOGW(TAG, "Device is not calibrated");
    } else {
        {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask & (1ULL << PULSE_PIN)) {
                ESP_LOGI(TAG, "Wakeup caused by PULSE_PIN");
                // Reset small leak detection after water is turned on
                wm_reset_leak_counter();
            } else if (wakeup_pin_mask & (1ULL << LEAK_PIN)) {
                ESP_LOGI(TAG, "Wakeup caused by LEAK_PIN");
            }

            if (load_app_config_from_fram() == ESP_OK &&
                app_settings.valid_flag == APP_CONFIG_VALID_FLAG &&
                app_settings.is_calibrated) {

                if (load_calibration_data()) {
                    ESP_LOGE(TAG, "Failed to apply calibration settings.");
                    app_settings.is_calibrated = false;
                } else {
                    ESP_LOGI(TAG, "Successfully loaded calibration data");
                    current_state = STATE_CALIBRATED;
                }
            } else {
                ESP_LOGW(TAG, "Device is not calibrated %d", app_settings.is_calibrated);
                app_settings.is_calibrated = false;
            }
        }

        if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT1) {
            ESP_LOGI(TAG, "None pulse wakeup event. Apply default values");
            set_default_sensor_values();
            wm_update_app_config_from_device();
            save_app_config_to_fram();
        }
    }

    // Configure GPIO interrupt for pulse input
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_set_direction(PULSE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LEAK_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(USER_INPUT_PIN, GPIO_MODE_INPUT);
    gpio_config_t io_conf = { };
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << PULSE_PIN) | (1ULL << LEAK_PIN) | (1ULL << USER_INPUT_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    // used for debug without sleep
    gpio_isr_handler_add(PULSE_PIN, YDripComponent::pulse_isr_handler, this);
    gpio_isr_handler_add(LEAK_PIN, YDripComponent::leak_isr_handler, this);
    gpio_isr_handler_add(USER_INPUT_PIN, YDripComponent::input_isr_handler, this);

    /* Set up sleep */
    sleep_scheduled = false;
    uint64_t wakeup_pin_mask = (1ULL << PULSE_PIN) | (1ULL << LEAK_PIN);
    //uint64_t wakeup_pin_mask = (1ULL << LEAK_PIN);
    esp_sleep_enable_ext1_wakeup(wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_sleep_enable_ext0_wakeup(USER_INPUT_PIN, 0);

    // Create a dedicated event loop for the water meter
    esp_event_loop_args_t loop_args = {
        .queue_size = 5,
        .task_name = "water_meter_event_task",
        .task_priority = 10,
        .task_stack_size = 4096,
        .task_core_id = tskNO_AFFINITY
    };

    // Create a custom event loop
    esp_err_t err = esp_event_loop_create(&loop_args, &water_meter_event_loop);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(err));
        return;
    }

    // Register the event handler for water meter events on the custom loop
    err = esp_event_handler_instance_register_with(water_meter_event_loop, WATER_METER_EVENTS, ESP_EVENT_ANY_ID, &YDripComponent::water_meter_event_handler, this, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event handler: %s", esp_err_to_name(err));
        return;
    }

    err = adc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup ADC: %s", esp_err_to_name(err));
        return;
    }

    send_event(WATER_METER_BOOT);
}

void YDripComponent::api_callback(float state) {
    ESP_LOGD(TAG, "Water usage callback from Home Assistant: %.2f", state);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        uint64_t wakeup_pin_status = esp_sleep_get_ext1_wakeup_status();

        if (wakeup_pin_status & (1ULL << PULSE_PIN)) {
            ESP_LOGD(TAG, "Pulse pin wakeup source. Entering sleep.");
            sleep_scheduled = true;
        }

        if (wakeup_pin_status & (1ULL << LEAK_PIN)) {
            ESP_LOGD(TAG, "Leak pin wakeup source. Entering sleep.");
            sleep_scheduled = true;
        }
    }
}

void YDripComponent::update() {

}

unsigned long last_time_ = 0;
void YDripComponent::loop() {
    process_events();

    if (api_is_connected()) {
        on_api_connected();
    }

    update_led();

    static int sleep_delay = 4;
    unsigned long current_time = millis();
    if (current_time - last_time_ >= 1000) {
        last_time_ = current_time;

        if (sleep_delay <= 0) {
            esp_deep_sleep_start();
        }
        if (sleep_scheduled) {
            ESP_LOGD(TAG, "Sleeping in %d", sleep_delay);
            sleep_delay--;
        }

        // Read the raw ADC voltage (in millivolts) and convert to volts
        float adc_voltage = read_adc_battery_voltage() / 1000.0;
        adc_voltage *= 2.0;

        // Apply the linear correction (slope and offset)
        float battery_voltage = adc_voltage;
        //ESP_LOGI(TAG, "adc_voltage %f, battery_voltage %f", adc_voltage, battery_voltage);

        battery_percentage = (battery_voltage - 3.4) / (4.2 - 3.4) * 100;
        if (battery_percentage < 0) battery_percentage = 0;
        if (battery_percentage > 100) battery_percentage = 100;
    }
}

void YDripComponent::update_led() {
    switch (current_state) {
        case STATE_UNCALIBRATED:
            status_led.set_color(COLOR_YELLOW);
            break;

        case STATE_CALIBRATED:
            status_led.set_color(cal_led_color);
            break;

        case STATE_CALIBRATING:
            status_led.breathe(COLOR_BLUE);
            break;

        case STATE_FAILED_CALIBRATION:
            status_led.breathe(COLOR_RED);
            break;

        case STATE_LEAK_DETECTED:
            status_led.set_color(COLOR_RED);
            break;

        default:
            status_led.set_color(COLOR_OFF);
            break;
    }
}


void YDripComponent::on_api_connected() {
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        static bool once = true;
        if (once) {
            once = false;

            api::global_api_server->get_home_assistant_state("sensor.ydrip_water_usage", { }, [this](std::string state) {
                    if (!state.empty()) {
                        ESP_LOGD(TAG, "Home Assistant state: %s", state.c_str());
                        this->api_callback(0);
                    }
            });
            publish_sensor_state();
        }
    }

    static bool once2 = true;
    if (once2) {
        once2 = false;
        publish_battery_state();
        publish_debug_state();
    }
}

void YDripComponent::sleep_timer_callback(void* arg) {
    if (!arg) {
        return;
    }

    YDripComponent* instance = static_cast<YDripComponent*>(arg);

    esp_deep_sleep_start();
}

void YDripComponent::start_sleep_timer(int seconds) {
    esp_timer_create_args_t timer_args = {
        .callback = &this->sleep_timer_callback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "one_shot_sleep_timer",
        .skip_unhandled_events = false
    };

    esp_timer_handle_t timer_handle;
    esp_err_t err = esp_timer_create(&timer_args, &timer_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        return;
    }

    // Start the timer for 60 seconds (60000 milliseconds)
    err = esp_timer_start_once(timer_handle, seconds * 1000 * 1000);  // Argument is in microseconds
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Sleeping in %d seconds", seconds);
}

void YDripComponent::handle_event(water_meter_event_t event) {
    switch (current_state) {
        case STATE_UNCALIBRATED:
            handle_uncalibrated_state(event);
            break;
        case STATE_CALIBRATED:
            handle_calibrated_state(event);
            break;
        case STATE_CALIBRATING:
            handle_calibrating_state(event);
            break;
        case STATE_FAILED_CALIBRATION:
            // Same as uncalibrated state
            handle_uncalibrated_state(event);
            break;
        case STATE_LEAK_DETECTED:
            break;
    }
}

void YDripComponent::send_event(water_meter_event_t event) {
    if (event_queue != nullptr) {
        // Send event to the queue with a timeout of 0 ticks
        if (xQueueSend(event_queue, &event, 0) != pdPASS) {
            ESP_LOGW(TAG, "Failed to enqueue event: %d", event);
        }
    }
}

void YDripComponent::process_events() {
    water_meter_event_t event;

    if (xQueueReceive(event_queue, &event, 0) == pdPASS) {
        switch (current_state) {
            case STATE_UNCALIBRATED:
                handle_uncalibrated_state(event);
                break;
            case STATE_CALIBRATED:
                handle_calibrated_state(event);
                break;
            case STATE_CALIBRATING:
                handle_calibrating_state(event);
                break;
            case STATE_FAILED_CALIBRATION:
                handle_uncalibrated_state(event);
                break;
            case STATE_LEAK_DETECTED:
                break;
        }
    }
}

void YDripComponent::handle_calibrated_state(water_meter_event_t event) {
    esp_err_t err = ESP_FAIL;
    static int pulse_count = 0;

    switch (event) {
        case WATER_METER_PULSE_EVENT:
            pulse_count++;
            ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
            if (cal_led_color.r == COLOR_GREEN.r &&
                cal_led_color.g == COLOR_GREEN.g &&
                cal_led_color.b == COLOR_GREEN.b) {
                cal_led_color = COLOR_LIGHT_GREEN;
            } else {
                cal_led_color = COLOR_GREEN;
            }

            break;
        case WATER_METER_USER_INPUT:
            pulse_count = 0;
            handle_uncalibrated_state(WATER_METER_USER_INPUT);
            break;
        case WATER_METER_LEAK_EVENT:
            ESP_LOGI(TAG, "Leak event");
            break;
        case WATER_METER_BOOT:
            if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
                // Give SLEEP_TIMER_SEC to report data to server 
                start_sleep_timer(SLEEP_TIMER_SEC);
            } else {
                // Wakeup was casued for reason other than usage/leak event
                // so go back to sleep
                start_sleep_timer(5);
            }

            break;
        default:
            ESP_LOGE(TAG, "No action for event %d in calibrated state.", event);
            break;
    }
}

void YDripComponent::handle_calibrating_state(water_meter_event_t event) {
    esp_err_t err = ESP_FAIL;

    switch (event) {
      case WATER_METER_CAL_STEP:
          if (this->calibration.direction == 1) {
              // Positive calibration direction
              if (this->calibration.low_non_inv > this->calibration.high_non_inv &&
                  this->calibration.low_inv > this->calibration.high_inv) {
                  ESP_LOGI(TAG, "Switching to negative direction calibration pass.");
                  // Reset search bounds for the next direction
                  this->calibration.low_non_inv = 0;
                  this->calibration.high_non_inv = CALIBRATION_MAX_REHOSTAT_OFFSET;
                  this->calibration.low_inv = 0;
                  this->calibration.high_inv = CALIBRATION_MAX_REHOSTAT_GAIN;
                  this->calibration.direction = -1;
                  calibration_init();
              }
              calibration_step_work(this->calibration.direction);
          } else {
              // Negative calibration direction
              if (this->calibration.low_non_inv > this->calibration.high_non_inv &&
                  this->calibration.low_inv > this->calibration.high_inv) {
                  ESP_LOGI(TAG, "Calibration complete.");
                  this->calibration.active = false;
                  this->cancel_interval("calibration_interval");
                  send_event(WATER_METER_CAL_DONE);
              } else {
                  this->calibration.direction = -1;
                  calibration_step_work(this->calibration.direction);
              }
          }
          break;

      case WATER_METER_CAL_DONE:
          //wm_set_power_duty_cycle(true);
          if (calibration_done() == ESP_OK) {
              ESP_LOGI(TAG, "Calibration completed successfully.");
              current_state = STATE_CALIBRATED;
              sleep_scheduled = true;
          } else {
              current_state = STATE_FAILED_CALIBRATION;
          }
          break;

      default:
          ESP_LOGE(TAG, "No action for event %d in uncalibrated state.", event);
          break;
    }
}

void YDripComponent::handle_uncalibrated_state(water_meter_event_t event) {
    esp_err_t err = ESP_FAIL;

    switch (event) {
        case WATER_METER_USER_INPUT:
            ESP_LOGI(TAG, "Starting calibration.");
            current_state = STATE_CALIBRATING;
            err = wm_start_calibration(CALIBRATION_VOLTAGE_TARGET);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Calibration failed. Error: %d", err);
                current_state = STATE_FAILED_CALIBRATION;
            }

            publish_debug_state();
            break;

        default:
            ESP_LOGE(TAG, "No action for event %d in uncalibrated state.", event);
            break;
    }
}

void YDripComponent::water_meter_event_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) {
    if (!handler_arg) {
        return;
    }

    YDripComponent* instance = static_cast<YDripComponent*>(handler_arg);

    if (base == WATER_METER_EVENTS) {
        instance->send_event((water_meter_event_t)id);
    }
}

esp_err_t YDripComponent::wm_start_calibration(int target_voltage)
{
    // Initialize calibration parameters
    this->calibration = {
      .target_voltage = target_voltage,
      .best_error = INT16_MAX,
      .min_voltage = INT16_MAX,
      .max_voltage = INT16_MIN,
      .low_non_inv = 0,
      .low_inv = 0,
      .high_non_inv = CALIBRATION_MAX_REHOSTAT_OFFSET,
      .high_inv = CALIBRATION_MAX_REHOSTAT_GAIN,
      .current_sample = 0,
      .attempts = 0,
      .active = true,
      .direction = 1,
    };

    wm_set_power_duty_cycle(false);
    calibration_init();

    // Set up a repeating timer to call calibration_step
    this->set_interval("calibration_interval", CALIBRATION_SAMPLE_INTERVAL_MS, [this]() { this->calibration_step(); });

    return ESP_OK;
}

void YDripComponent::calibration_init() {
    uint16_t rheostat_value_non_inv, rheostat_value_inv;

    rheostat_value_non_inv = (this->calibration.low_non_inv + this->calibration.high_non_inv) / 2;
    rheostat_value_inv = (this->calibration.high_inv + this->calibration.low_inv) / 2;

    // Set rheostat value starting point
    wm_write_reg16(RHEOSTAT0_SET_VALUE_ADDR, rheostat_value_non_inv);
    wm_write_reg16(RHEOSTAT1_SET_VALUE_ADDR, rheostat_value_inv);
}

void YDripComponent::calibration_step() {
    if (!this->calibration.active) {
        this->cancel_interval("calibration_interval");
        //wm_set_power_duty_cycle(true);
        return;
    }

    send_event(WATER_METER_CAL_STEP);
}

void YDripComponent::calibration_step_work(int direction) {
    // Sample the ADC once per call
    int adc_raw_value = adc1_get_raw(MAG_OUTPUT_ADC);
    int voltage = esp_adc_cal_raw_to_voltage(adc_raw_value, &adc1_chars);

    // Update min and max voltages based on single sample
    this->calibration.min_voltage = std::min(this->calibration.min_voltage, voltage);
    this->calibration.max_voltage = std::max(this->calibration.max_voltage, voltage);
    this->calibration.current_sample++;

    // Only proceed with calibration logic after collecting enough samples
    if (this->calibration.current_sample < CALIBRATION_NUM_SAMPLES) {
        return;  // Continue collecting samples
    }

    // Calculate the midpoint voltage and reset sample count
    int middle_voltage = (this->calibration.min_voltage + this->calibration.max_voltage) / 2;
    int error = abs(this->calibration.target_voltage - middle_voltage);

    // Determine rheostat values based on direction
    uint16_t rheostat_value_non_inv, rheostat_value_inv;
    if (direction == 1) {  // Positive direction
        rheostat_value_non_inv = (this->calibration.low_non_inv + this->calibration.high_non_inv) / 2;
        rheostat_value_inv = (this->calibration.high_inv + this->calibration.low_inv) / 2;
    } else {  // Negative direction
        rheostat_value_non_inv = (this->calibration.high_non_inv + this->calibration.low_non_inv) / 2;
        rheostat_value_inv = (this->calibration.low_inv + this->calibration.high_inv) / 2;
    }

    ESP_LOGD(TAG, "error: %d,  non_inv: %d, inv: %d, min: %d, max: %d, ms: %lld",
         error,
         rheostat_value_non_inv,
         rheostat_value_inv,
         this->calibration.min_voltage,
         this->calibration.max_voltage,
         esp_timer_get_time() / 1000);

    // Set rheostat values
    wm_write_reg16(RHEOSTAT0_SET_VALUE_ADDR, rheostat_value_non_inv);
    wm_write_reg16(RHEOSTAT1_SET_VALUE_ADDR, rheostat_value_inv);

    // Update the best error and values if this configuration is better
    if (error < this->calibration.best_error) {
        ESP_LOGI(TAG, "New best midpoint found with rh0: %d rh1: %d, error: %d",
                 rheostat_value_non_inv, rheostat_value_inv, error);

        this->calibration.best_error = error;
        app_settings.rheostat_value_non_inv_reg = rheostat_value_non_inv;
        app_settings.rheostat_value_inv_reg = rheostat_value_inv;
        app_settings.min_voltage = this->calibration.min_voltage;
        app_settings.max_voltage = this->calibration.max_voltage;
        app_settings.target_voltage = this->calibration.target_voltage;
    }

    // Adjust binary search ranges based on the divergence
    if (middle_voltage > this->calibration.target_voltage) {
        if (direction == 1) {
            this->calibration.low_non_inv = rheostat_value_non_inv + 1;
            this->calibration.high_inv = rheostat_value_inv - 1;
        } else {
            this->calibration.high_non_inv = rheostat_value_non_inv - 1;
            this->calibration.low_inv = rheostat_value_inv + 1;
        }
    } else {
        if (direction == 1) {
            this->calibration.high_non_inv = rheostat_value_non_inv - 1;
            this->calibration.low_inv = rheostat_value_inv + 1;
        } else {
            this->calibration.low_non_inv = rheostat_value_non_inv + 1;
            this->calibration.high_inv = rheostat_value_inv - 1;
        }
    }

    this->calibration.attempts++;
    this->calibration.current_sample = 0;
    this->calibration.min_voltage = INT16_MAX;
    this->calibration.max_voltage = INT16_MIN;
}

esp_err_t YDripComponent::calibration_done() {
    // After finding the best values, set the rheostats to these optimal values
    esp_err_t ret = wm_write_reg16(RHEOSTAT0_SET_VALUE_ADDR, app_settings.rheostat_value_non_inv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set final Rheostat 0 value");
        return ret;
    }

    ret = wm_write_reg16(RHEOSTAT1_SET_VALUE_ADDR, app_settings.rheostat_value_inv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set final Rheostat 1 value");
        return ret;
    }

    // Final check for successful calibration
    if (this->calibration.best_error <= CALIBRATION_TOLERANCE_MV) {
        ESP_LOGI(TAG, "Middle of sine wave within tolerance. Best error: %d", this->calibration.best_error);
        wm_reset_usage_counter();
        wm_reset_leak_counter();
        ret = ESP_OK;
    } else {
        ESP_LOGE(TAG, "Unable to achieve target voltage midpoint within tolerance. Closest achieved error: %d", this->calibration.best_error);
        ret = ESP_FAIL;
    }


    if (ret == ESP_OK) {
        //float hysteresis_percentage = 0.3;
        //int low_threshold = app_settings.min_voltage + (app_settings.max_voltage - app_settings.min_voltage) * hysteresis_percentage;
        //int high_threshold = app_settings.max_voltage - (app_settings.max_voltage - app_settings.min_voltage) * hysteresis_percentage;

        int middle_voltage = app_settings.min_voltage + (app_settings.max_voltage - app_settings.min_voltage)/2;
        int low_threshold = middle_voltage;
        int high_threshold = middle_voltage + CALIBRATION_OFFSET*2;

        ESP_LOGI(TAG, "Setting comparator thresholds: Low = %d mV, High = %d mV", low_threshold, high_threshold);

        // Set the comparator thresholds using your existing function
        ret = wm_set_comparator_threshold(high_threshold, low_threshold);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Comparator thresholds successfully set.");
            int read_high_mv = 0;
            int read_low_mv = 0;
            wm_get_comparator_threshold(&read_high_mv, &read_low_mv);
            int hysteresis_mv = read_high_mv - read_low_mv;

            ESP_LOGI(TAG, "Actual comparator threshold values: Low = %d mV, High = %d mV", read_low_mv, read_high_mv);
            ESP_LOGI(TAG, "%d mV of hysteresis", hysteresis_mv);

            if (hysteresis_mv < CALIBRATION_HYSTERESIS_MIN) {
                ESP_LOGE(TAG, "Hysteresis (%dmV) below threshold of %dmV. Calibration failed.", hysteresis_mv, CALIBRATION_HYSTERESIS_MIN);
                app_settings.is_calibrated = false;
                ret = ESP_FAIL;
            } else {
                app_settings.is_calibrated = true;
                wm_update_app_config_from_device();
                ret = save_app_config_to_fram();
            }
        } else {
            ESP_LOGE(TAG, "Failed to set comparator thresholds.");
        }
    } else {
        ESP_LOGE(TAG, "Calibration failed. Could not reach the target voltage after %d attempts.", this->calibration.attempts);
    }

    return ret;
}

void YDripComponent::publish_sensor_state() {
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        uint64_t wakeup_pin_status = esp_sleep_get_ext1_wakeup_status();

        if ((wakeup_pin_status & (1ULL << PULSE_PIN))) {
            ESP_LOGI(TAG, "Pulse event detected");
            rtc_usage_counter += usage_alert_count_;
            //this->usage_sensor_->publish_state(rtc_usage_counter);
            this->usage_sensor_->publish_state(app_settings.pulse_alert_limit);
        }

        if ((wakeup_pin_status & (1ULL << LEAK_PIN))) {
            ESP_LOGI(TAG, "Leak event detected");
            this->leak_sensor_->publish_state(true);
        }
    }
}

void YDripComponent::publish_battery_state() {
    if (battery_sensor_ == nullptr) {
        return;
    }

    battery_sensor_->publish_state(battery_percentage);
    ESP_LOGI(TAG, "Publish battery percentage %f", battery_percentage);
}

void YDripComponent::publish_debug_state() {
    char debug_info[512];
    snprintf(debug_info, sizeof(debug_info),
             "AppCfg v%d\n Calib: %d\n PulseAlert: %u\n LeakAlert: %u\n ClockFreq: %uHz\n "
             "LFThresh: %.6fHz\n MinV: %umV\n MaxV: %umV\n TargetV: %umV\n "
             "ACMP HiTh: %d (%dmV)\n ACMP LoTh: %d (%dmV)\n "
             "RheoNonInvReg: %u\n RheoInvReg: %u\n UseCnt1Reg: %u\n UseCnt2Reg: %u\n "
             "LeakCntReg: %u\n LFDetReg: %u",
             app_settings.version, app_settings.is_calibrated,
             app_settings.pulse_alert_limit, app_settings.leak_alert_limit,
             app_settings.slow_clock_freq, app_settings.low_freq_detect_thresh,
             app_settings.min_voltage, app_settings.max_voltage, app_settings.target_voltage,
             app_settings.low_to_high_thresh, wm_comparator_threshold_reg_to_mv(app_settings.low_to_high_thresh),
             app_settings.high_to_low_thresh, wm_comparator_threshold_reg_to_mv(app_settings.high_to_low_thresh),
             app_settings.rheostat_value_non_inv_reg, app_settings.rheostat_value_inv_reg,
             app_settings.usage_count_1_reg, app_settings.usage_count_2_reg,
             app_settings.leak_alert_count_reg, app_settings.low_freq_detect_reg
    );

    ESP_LOGI(TAG, "%s", debug_info);

    // Create a UDP socket
    int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return;
    }

    // Configure the target address
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(this->debug_ip_.c_str());
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(this->debug_port_);

    // Send the debug info over UDP
    int err = sendto(sock, debug_info, strlen(debug_info), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "Message sent successfully to %s:%d", this->debug_ip_.c_str(), this->debug_port_);
    }

    close(sock);
}

uint32_t YDripComponent::read_adc_battery_voltage() {
    uint32_t adc_sum = 0;
    const int num_samples = 10;

    // Read the ADC value 10 times and accumulate the result
    for (int i = 0; i < num_samples; i++) {
        adc_sum += adc1_get_raw(VBAT_DIV_ADC);
    }

    // Calculate the average ADC value
    uint32_t adc_avg = adc_sum / num_samples;

    // Convert the averaged ADC reading to voltage
    return esp_adc_cal_raw_to_voltage(adc_avg, &adc1_chars);
}


void YDripComponent::dump_config() {
    ESP_LOGI(TAG, "YDrip:");
    LOG_I2C_DEVICE(this);
    if (this->is_failed()) {
        ESP_LOGE(TAG, "Communication with YDrip failed!");
        return;
    }

    wm_update_app_config_from_device();
    ESP_LOGI(TAG, "App Config:");

    ESP_LOGI(TAG, "  App config version: %d", app_settings.version);

    ESP_LOGI(TAG, "  Is calibrated: %d", app_settings.is_calibrated);

    // Print alert limits
    ESP_LOGI(TAG, "  Pulse Alert Limit: %u", app_settings.pulse_alert_limit);
    ESP_LOGI(TAG, "  Leak Alert Limit: %u", app_settings.leak_alert_limit);

    // Print slow clock frequency and low frequency detection threshold
    ESP_LOGI(TAG, "  Slow Clock Frequency: %u Hz", app_settings.slow_clock_freq);
    ESP_LOGI(TAG, "  Low Frequency Detection Threshold: %.6f Hz", app_settings.low_freq_detect_thresh);

    // Print calibration info
    ESP_LOGI(TAG, "  Min Voltage: %u mV", app_settings.min_voltage);
    ESP_LOGI(TAG, "  Max Voltage: %u mV", app_settings.max_voltage);
    ESP_LOGI(TAG, "  Target Voltage: %u mV", app_settings.target_voltage);
    int vdda = 3300;
    ESP_LOGI(TAG, "  ACMP High Threshold: %d (%dmV)",
             app_settings.low_to_high_thresh,
             wm_comparator_threshold_reg_to_mv(app_settings.low_to_high_thresh));
    ESP_LOGI(TAG, "  ACMP Low Threshold: %d (%dmV)",
             app_settings.high_to_low_thresh,
             wm_comparator_threshold_reg_to_mv(app_settings.high_to_low_thresh));

    // Print register values
    ESP_LOGI(TAG, "  Rheostat Value Non-Inverted Register: %u", app_settings.rheostat_value_non_inv_reg);
    ESP_LOGI(TAG, "  Rheostat Value Inverted Register: %u", app_settings.rheostat_value_inv_reg);
    ESP_LOGI(TAG, "  Usage Count 1 Register: %u", app_settings.usage_count_1_reg);
    ESP_LOGI(TAG, "  Usage Count 2 Register: %u", app_settings.usage_count_2_reg);
    ESP_LOGI(TAG, "  Leak Alert Count Register: %u", app_settings.leak_alert_count_reg);
    ESP_LOGI(TAG, "  Low Frequency Detection Register: %u", app_settings.low_freq_detect_reg);

}

void YDripComponent::wm_update_app_config_from_device(void) {
    uint16_t rheostat_value_non_inv_reg = 0;
    uint16_t rheostat_value_inv_reg = 0;

    // Read raw register settings
    wm_read_reg16(RHEOSTAT0_SET_VALUE_ADDR, &rheostat_value_non_inv_reg);
    wm_read_reg16(RHEOSTAT1_SET_VALUE_ADDR, &rheostat_value_inv_reg);
    app_settings.rheostat_value_non_inv_reg = rheostat_value_non_inv_reg;
    app_settings.rheostat_value_inv_reg = rheostat_value_inv_reg;

    read_bytes_(CNT2_DATA_LIMIT_ADDR, &app_settings.usage_count_1_reg, sizeof(app_settings.usage_count_1_reg));
    read_bytes_(CNT4_DATA_LIMIT_ADDR, &app_settings.usage_count_2_reg, sizeof(app_settings.usage_count_2_reg));
    read_bytes_(CNT6_DATA_LIMIT_ADDR, &app_settings.leak_alert_count_reg, sizeof(app_settings.leak_alert_count_reg));
    read_bytes_(CNT3_DATA_LIMIT_ADDR, &app_settings.low_freq_detect_reg, sizeof(app_settings.low_freq_detect_reg));

    // Use temporary variables for alignment-sensitive operations
    uint16_t temp_pulse_alert_limit;
    uint16_t temp_leak_alert_limit;
    double temp_low_freq_detect_thresh;

    // Read processed values derived from register settings
    wm_get_pulse_alert_count(&temp_pulse_alert_limit);
    wm_get_leak_alert_count(&temp_leak_alert_limit);

    slow_clock_frequency_t slow_clock_freq;
    wm_get_slow_clock(&slow_clock_freq);

    wm_get_low_freq_thresh(&temp_low_freq_detect_thresh);

    // Assign the results back to the packed structure
    app_settings.pulse_alert_limit = temp_pulse_alert_limit;
    app_settings.leak_alert_limit = temp_leak_alert_limit;
    app_settings.slow_clock_freq = (uint16_t)slow_clock_freq;
    app_settings.low_freq_detect_thresh = temp_low_freq_detect_thresh;

    // Read ACMP threshold values
    wm_get_comparator_threshold_reg(&app_settings.low_to_high_thresh, &app_settings.high_to_low_thresh);

}

float YDripComponent::get_setup_priority() const { return setup_priority::DATA; }

void YDripComponent::set_usage_alert_count(uint16_t count) {
    usage_alert_count_ = count;
}

void YDripComponent::set_leak_alert_count(uint16_t count) {

    leak_alert_count_ = count;
}

void YDripComponent::set_low_freq_leak_thresh(double threshold) {
    low_freq_leak_thresh_ = threshold;
}

void YDripComponent::set_debug_ip(const std::string &debug_ip) {
  this->debug_ip_ = debug_ip;
}

void YDripComponent::set_debug_port(const uint16_t &debug_port) {
  this->debug_port_ = debug_port;
}

esp_err_t YDripComponent::wm_set_slow_clock(slow_clock_frequency_t desired_freq) {
    esp_err_t err = ESP_FAIL;

    osc0_divider_t OSC0_DIVIDER;
    pipe_delay_divider_t PIPE_DIVIDER;

    switch (desired_freq) {
      case SLOW_CLOCK_1024Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_1;
          PIPE_DIVIDER = PIPE_DIVIDER_2;
          break;
      case SLOW_CLOCK_512Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_2;
          PIPE_DIVIDER = PIPE_DIVIDER_2;
          break;
      case SLOW_CLOCK_256Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_4;
          PIPE_DIVIDER = PIPE_DIVIDER_2;
          break;
      case SLOW_CLOCK_128Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_8;
          PIPE_DIVIDER = PIPE_DIVIDER_2;
          break;
      case SLOW_CLOCK_64Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_8;
          PIPE_DIVIDER = PIPE_DIVIDER_4;
          break;
      case SLOW_CLOCK_32Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_8;
          PIPE_DIVIDER = PIPE_DIVIDER_8;
          break;
      case SLOW_CLOCK_16Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_64;
          PIPE_DIVIDER = PIPE_DIVIDER_2;
          break;
      case SLOW_CLOCK_8Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_64;
          PIPE_DIVIDER = PIPE_DIVIDER_4;
          break;
      case SLOW_CLOCK_4Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_64;
          PIPE_DIVIDER = PIPE_DIVIDER_8;
          break;
      case SLOW_CLOCK_2Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_64;
          PIPE_DIVIDER = PIPE_DIVIDER_16;
          break;
      case SLOW_CLOCK_1Hz:
          OSC0_DIVIDER = OSC0_DIVIDER_64;
          PIPE_DIVIDER = PIPE_DIVIDER_32;
          break;

      default:
          return ESP_ERR_INVALID_ARG;
    }

    // Set the first divider
    uint8_t value = OSC0_DIVIDER << OSC0_OUT1_CLOCK_OFFSET;
    err = update_register(OSC0_OUT1_CLOCK_ADDR, OSC0_OUT1_CLOCK_MASK, value);
    if (err != ESP_OK) return err;


    // Set the second divider
    // Put pipe delay in reset state, otherwise it's possible the flip flops can
    // get in an inconsistent state while updating
    err |= update_register(I2C_GPIO_ADDR, RESET_PIPE_MASK, 1 << RESET_PIPE_BIT);
    err |= update_register(I2C_GPIO_ADDR, RESET_PIPE_MASK, 0 << RESET_PIPE_BIT);

    /* From datasheet:
         A clock divider can be made by setting OUT1 to inverted, wiring OUT1
         to IN, and choosing OUT1 PD num to be half of your desired divider,
         ie. if you want a /4, choose OUT1 PD num to be 2.
    */
    value = PIPE_DIVIDER << PIPE_DELAY_CNT_OFFSET;
    err |= update_register(PIPE_DELAY_CNT_ADDR, PIPE_DELAY_CNT_MASK, value);

    // Disable pipe delay reset
    err |= update_register(I2C_GPIO_ADDR, RESET_PIPE_MASK, 1 << RESET_PIPE_BIT);

    return err;
}

esp_err_t YDripComponent::wm_get_slow_clock(slow_clock_frequency_t* slow_clock_freq) {
    esp_err_t err = ESP_FAIL;
    uint8_t osc0_reg = 0;
    uint8_t pipe_reg = 0;
    int osc0_div = -1;
    int pipe_div = -1;

    if (!slow_clock_freq) {
        return ESP_FAIL;
    }

    // Read first divider
    err = read_bytes_(OSC0_OUT1_CLOCK_ADDR, &osc0_reg, sizeof(osc0_reg));
    if (err != ESP_OK) {
        return err;
    }
    osc0_reg = (osc0_reg & OSC0_OUT1_CLOCK_MASK) >> OSC0_OUT1_CLOCK_OFFSET;
    osc0_div = get_osc0_divider_value((osc0_divider_t)osc0_reg);

    // Read second divider
    err = read_bytes_(PIPE_DELAY_CNT_ADDR, &pipe_reg, sizeof(pipe_reg));
    if (err != ESP_OK) {
        return err;
    }
    pipe_reg = (pipe_reg & PIPE_DELAY_CNT_MASK) >> PIPE_DELAY_CNT_OFFSET;
    pipe_div = get_pipe_delay_divider_value((pipe_delay_divider_t)pipe_reg);

    if (osc0_div == -1 || pipe_div == -1 || osc0_div == 0 || pipe_div == 0) {
        ESP_LOGI(TAG, "invalid div: %d %d", osc0_div, pipe_div);
        return ESP_FAIL;
    }

    *slow_clock_freq = static_cast<slow_clock_frequency_t>(OSC0_CLK_FREQ / osc0_div / pipe_div);
    return ESP_OK;
}

esp_err_t YDripComponent::wm_get_pulse_alert_count(uint16_t* value) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cnt_2_value = 0;
    uint8_t cnt_4_value = 0;

    // Read the values from the corresponding addresses
    esp_err_t err = read_bytes_(CNT2_DATA_LIMIT_ADDR, &cnt_2_value, sizeof(cnt_2_value));
    if (err != ESP_OK) {
        return err;
    }

    err = read_bytes_(CNT4_DATA_LIMIT_ADDR, &cnt_4_value, sizeof(cnt_4_value));
    if (err != ESP_OK) {
        return err;
    }

    // Calculate the pulse alert count
    *value = (uint16_t)((cnt_2_value + 1) * (cnt_4_value + 1));

    return ESP_OK;
}

uint16_t YDripComponent::wm_get_valid_pulse_alert_count(uint16_t count) {
    uint8_t cnt_2_value = (count / 2) - 1;
    if (cnt_2_value == 0) {
        cnt_2_value = 1;
    }

    uint8_t cnt_4_value = (count / (cnt_2_value + 1)) - 1;
    if (cnt_4_value == 0) {
        cnt_4_value = 1;
    }
    return (cnt_2_value + 1) * (cnt_4_value + 1);
}

esp_err_t YDripComponent::wm_set_pulse_alert_count(uint16_t count) {
    uint8_t cnt_2_value = (count / 2) - 1;
    if (cnt_2_value == 0) {
        cnt_2_value = 1;
    }

    uint8_t cnt_4_value = (count / (cnt_2_value + 1)) - 1;
    if (cnt_4_value == 0) {
        cnt_4_value = 1;
    }
    uint16_t wakeup_count = (cnt_2_value + 1) * (cnt_4_value + 1);

    esp_err_t err = write_bytes_(CNT2_DATA_LIMIT_ADDR, &cnt_2_value, sizeof(cnt_2_value));
    err |= write_bytes_(CNT4_DATA_LIMIT_ADDR, &cnt_4_value, sizeof(cnt_4_value));
    ESP_LOGI(TAG, "Set wake up count: %d", wakeup_count);

    return err;
}

esp_err_t YDripComponent::wm_get_leak_alert_count(uint16_t* value) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cnt_6_value = 0;

    // Read the value from the corresponding address
    esp_err_t err = read_bytes_(CNT6_DATA_LIMIT_ADDR, &cnt_6_value, sizeof(cnt_6_value));
    if (err != ESP_OK) {
        return err;
    }

    // Calculate the leak alert count
    *value = (uint16_t)(cnt_6_value + 1);

    return ESP_OK;
}

esp_err_t YDripComponent::wm_set_leak_alert_count(uint16_t count) {
    uint8_t cnt_6_value = count - 1;
    if (cnt_6_value == 0) {
        cnt_6_value = 1;
    }

    uint16_t leak_alert_count = (cnt_6_value + 1);

    esp_err_t err = write_bytes_(CNT6_DATA_LIMIT_ADDR, &cnt_6_value, sizeof(cnt_6_value));
    ESP_LOGI(TAG, "Set leak alert count: %d", leak_alert_count);

    return err;
}

esp_err_t YDripComponent::wm_get_low_freq_thresh(double* freq_thresh) {
    if (freq_thresh == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    slow_clock_frequency_t slow_clock_freq;
    uint8_t val;

    // Check if wm_get_slow_clock is successful
    ESP_RETURN_ON_ERROR(wm_get_slow_clock(&slow_clock_freq), TAG, "Failed to get slow clock divider");

    // Read the value from the corresponding address
    esp_err_t err = read_bytes_(CNT3_DATA_LIMIT_ADDR, &val, sizeof(val));
    if (err != ESP_OK) {
        return err;
    }

    // Calculate the frequency threshold based on the counter value
    *freq_thresh = slow_clock_freq / (val + 2.0f);

    return ESP_OK;
}

esp_err_t YDripComponent::wm_set_low_freq_thresh(double freq_thresh) {
    slow_clock_frequency_t slow_clock_freq;

    if (freq_thresh <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if wm_get_slow_clock is successful
    ESP_RETURN_ON_ERROR(wm_get_slow_clock(&slow_clock_freq), TAG, "Failed to get slow clock divider");

    // The minimum frequency threshold corresponds to the maximum possible counter value.
    // For an 8-bit counter, the maximum value is 255.
    // According to the delay time formula: Delay time = (Counter data + 2) / slow_clock_freq
    // Thus, the minimum frequency threshold is calculated as:
    // min_freq_thresh = slow_clock_freq / (255 + 2) = slow_clock_freq / 257.0f
    double min_freq_thresh = (double)slow_clock_freq / 257.0f;

    // The maximum frequency threshold corresponds to the minimum possible counter value.
    // The minimum counter value is 1.
    // According to the delay time formula: Delay time = (Counter data + 2) / slow_clock_freq
    // Thus, the maximum frequency threshold is calculated as:
    // max_freq_thresh = slow_clock_freq / (1 + 2) = slow_clock_freq / 3.0f
    double max_freq_thresh = (double)slow_clock_freq / 3.0f;

    // Clamp the input frequency threshold to the valid range
    if (freq_thresh < min_freq_thresh) {
        freq_thresh = min_freq_thresh;
    } else if (freq_thresh > max_freq_thresh) {
        freq_thresh = max_freq_thresh;
    }

    // Calculate the counter data value 'val' for the given frequency threshold
    // According to the delay time formula:
    // freq_thresh = slow_clock_freq / (Counter data + 2)
    // Rearranging to solve for Counter data:
    // Counter data = (slow_clock_freq / freq_thresh) - 2
    uint8_t val = (uint8_t)((slow_clock_freq / freq_thresh) - 2);

    // Check if write_bytes_ is successful
    ESP_RETURN_ON_ERROR(
        write_bytes_(CNT3_DATA_LIMIT_ADDR, &val, sizeof(val)),
        TAG,
        "Failed to write CNT3_DATA_LIMIT_ADDR"
        );

    // Calculate the actual frequency threshold that was set based on the counter value
    // This is to verify the operation:
    // calculated_freq_thresh = slow_clock_freq / (val + 2)
    double calculated_freq_thresh = slow_clock_freq / (val + 2.0f);
    ESP_LOGI(TAG, "set_low_freq_thresh: %f Hz (val: %d)", calculated_freq_thresh, val);

    return ESP_OK;
}

esp_err_t YDripComponent::wm_set_power_duty_cycle(bool enable) {
    esp_err_t err = ESP_FAIL;

    if (enable) {
        err |= update_register(I2C_GPIO_ADDR, DISABLE_PWR_DUTY_CYCLE_MASK, 0 << DISABLE_PWR_DUTY_CYCLE_BIT);
    } else {
        err |= update_register(I2C_GPIO_ADDR, DISABLE_PWR_DUTY_CYCLE_MASK, 1 << DISABLE_PWR_DUTY_CYCLE_BIT);
    }

    return err;
}

esp_err_t YDripComponent::wm_reset_usage_counter() {
    esp_err_t err = ESP_FAIL;

    err |= update_register(I2C_GPIO_ADDR, USAGE_CNT_RESET_MASK, 0 << USAGE_CNT_RESET_BIT_OFFSET);
    err |= update_register(I2C_GPIO_ADDR, USAGE_CNT_RESET_MASK, 1 << USAGE_CNT_RESET_BIT_OFFSET);
    err |= update_register(I2C_GPIO_ADDR, USAGE_CNT_RESET_MASK, 0 << USAGE_CNT_RESET_BIT_OFFSET);

    return err;
}

esp_err_t YDripComponent::wm_reset_leak_counter() {
    esp_err_t err = ESP_FAIL;

    err |= update_register(I2C_GPIO_ADDR, SLOW_CNT_RESET_MASK, 0 << SLOW_CNT_RESET_BIT_OFFSET);
    err |= update_register(I2C_GPIO_ADDR, SLOW_CNT_RESET_MASK, 1 << SLOW_CNT_RESET_BIT_OFFSET);
    err |= update_register(I2C_GPIO_ADDR, SLOW_CNT_RESET_MASK, 0 << SLOW_CNT_RESET_BIT_OFFSET);

    return err;
}

esp_err_t YDripComponent::wm_set_comparator_threshold(int high_mv, int low_mv) {
    esp_err_t err = ESP_FAIL;

    // ACMP threshold mv = VDDA * ([high/low]_thres / 64), VDDA is now 3300 mV
    int vdda = 3300;

    // Convert millivolts to 6-bit threshold values (scale and round)
    uint8_t low_thresh = static_cast<uint8_t>(round((low_mv * 64.0) / vdda));
    uint8_t high_thresh = static_cast<uint8_t>(round((high_mv * 64.0) / vdda));

    // Subtract by 1 to match hardware representation
    if (low_thresh > 0) {
        low_thresh -= 1;
    }
    if (high_thresh > 0) {
        high_thresh -= 1;
    }

    err = wm_set_comparator_threshold_reg(high_thresh, low_thresh);

    return err;
}

esp_err_t YDripComponent::wm_set_comparator_threshold_reg(uint8_t high_thresh, uint8_t low_thresh) {
    esp_err_t err = ESP_FAIL;

    // Write the thresholds to the registers
    err = update_register(
        ACMP0_LOW_TO_HIGH_ADDR,
        ACMP0_LOW_TO_HIGH_MASK,
        high_thresh << ACMP0_LOW_TO_HIGH_OFFSET);
    if (err != ESP_OK) {
        return err;
    }

    err = update_register(
        ACMP0_HIGH_TO_LOW_ADDR,
        ACMP0_HIGH_TO_LOW_MASK,
        low_thresh << ACMP0_HIGH_TO_LOW_OFFSET);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

uint16_t YDripComponent::wm_comparator_threshold_reg_to_mv(uint8_t thresh) {
    // Register is zero indexed
    thresh += 1;
    
    uint16_t vdda = 3300;
    return static_cast<int>(round((thresh * vdda) / 64.0));
}

esp_err_t YDripComponent::wm_get_comparator_threshold_reg(uint8_t* high_thresh, uint8_t* low_thresh) {
    esp_err_t err = ESP_FAIL;

    if (!high_thresh || !low_thresh) {
        return err;
    }
    // Read the raw values from the respective registers
    err = wm_read_reg(ACMP0_LOW_TO_HIGH_ADDR, high_thresh, 1);
    if (err != ESP_OK) {
        return err;
    }

    err = wm_read_reg(ACMP0_HIGH_TO_LOW_ADDR, low_thresh, 1);
    if (err != ESP_OK) {
        return err;
    }

    *high_thresh = (*high_thresh & ACMP0_LOW_TO_HIGH_MASK) >> ACMP0_LOW_TO_HIGH_OFFSET;
    *low_thresh = (*low_thresh & ACMP0_HIGH_TO_LOW_MASK) >> ACMP0_HIGH_TO_LOW_OFFSET;

    return ESP_OK;
}

esp_err_t YDripComponent::wm_get_comparator_threshold(int* high_mv, int* low_mv) {
    esp_err_t err = ESP_FAIL;

    if (!high_mv || !low_mv) {
        return err;
    }

    uint8_t low_thres = 0, high_thres = 0;
    err = wm_get_comparator_threshold_reg(&high_thres, &low_thres);
    if (err != ESP_OK) {
        return err;
    }

    *low_mv = wm_comparator_threshold_reg_to_mv(low_thres);
    *high_mv = wm_comparator_threshold_reg_to_mv(high_thres);

    return ESP_OK;
}

esp_err_t YDripComponent::wm_write_reg16(uint8_t reg, uint16_t value) {
    uint8_t data[2];

    // Split the 16-bit value into MSB and LSB for big-endian format
    data[1] = (uint8_t)(value >> 8);  // MSB: 0x03
    data[0] = (uint8_t)value;         // LSB: 0xFF

    // Write the two bytes to consecutive registers starting from 'reg'
    esp_err_t ret = write_bytes_(reg, data, sizeof(data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write 16-bit value to register 0x%02X", reg);
        return ret;
    }

    //ESP_LOGD(TAG, "Wrote 16-bit value 0x%04X to register 0x%02X", value, reg);

    return ESP_OK;
}

esp_err_t YDripComponent::wm_write_reg(uint8_t reg, uint8_t* data, size_t size) {
    return write_bytes_(reg, data, size);
}

esp_err_t YDripComponent::wm_read_reg16(uint8_t reg, uint16_t* value) {
    uint8_t data[2] = { 0, 0 };

    if (!value) {
        ESP_LOGE(TAG, "Invalid argument: value is null");
        return ESP_ERR_INVALID_ARG;
    }

    // Read two consecutive bytes starting from the register address
    esp_err_t ret = read_bytes_(reg, data, sizeof(data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register");
        return ret;
    }

    // Combine the two bytes into a 16-bit value (Big-Endian to Host Endian)
    *value = (data[1] << 8) | data[0];

    return ret;
}

esp_err_t YDripComponent::wm_read_reg(uint8_t reg, uint8_t* data, size_t size) {
    return read_bytes_(reg, data, size);
}

esp_err_t YDripComponent::update_register(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current_value;
    esp_err_t err = read_bytes_(reg, &current_value, sizeof(current_value));
    if (err != ESP_OK) {
        return err;
    }

    // Clear the bits in the mask and set the new value
    uint8_t new_value = (current_value & ~mask) | (value & mask);
    return write_bytes_(reg, &new_value, sizeof(new_value));
}

esp_err_t YDripComponent::write_bytes_(uint8_t reg, const uint8_t* data, size_t size) {
    this->set_i2c_address(WATER_METER_READER_ADDR);

    // I2C write: first argument is the register, followed by data and size
    if (this->write_bytes(reg, data, static_cast<uint8_t>(size))) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to write to register 0x%02X", reg);
        return ESP_FAIL;
    }
}

esp_err_t YDripComponent::read_bytes_(uint8_t reg, uint8_t* data, size_t size) {
    this->set_i2c_address(WATER_METER_READER_ADDR);

    // I2C read: first argument is the register, followed by data and size
    if (this->read_bytes(reg, data, static_cast<uint8_t>(size))) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to read from register 0x%02X", reg);
        return ESP_FAIL;
    }
}

esp_err_t YDripComponent::invalidate_fram_app_config() {
    esp_err_t err;

    // Set the valid_flag in app_settings to 0 (invalidating the config)
    app_settings.valid_flag = 0;

    // Write only the valid_flag back to FRAM (if you want to just update that field)
    err = fram_page_write(FRAM_APP_CONFIG_ADDR, (const uint8_t*)&app_settings.valid_flag, sizeof(app_settings.valid_flag));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to invalidate app config in FRAM: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "App config invalidated in FRAM.");
    }

    return err;
}


esp_err_t YDripComponent::save_app_config_to_fram() {
    esp_err_t err;

    // Set up some fields in the app_settings struct
    app_settings.valid_flag = APP_CONFIG_VALID_FLAG;
    app_settings.version = APP_CONFIG_VERSION;

    // Write the entire app_settings struct to FRAM starting at FRAM_BASE_ADDRESS
    err = fram_page_write(FRAM_APP_CONFIG_ADDR, (const uint8_t*)&app_settings, sizeof(app_settings));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save app config to FRAM: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "App config successfully saved to FRAM.");
    }

    return err;
}

esp_err_t YDripComponent::load_app_config_from_fram() {
    esp_err_t err;

    // Read the entire app_settings struct from FRAM starting at FRAM_BASE_ADDRESS
    err = fram_sequential_read(FRAM_APP_CONFIG_ADDR, (uint8_t*)&app_settings, sizeof(app_settings));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load app config from FRAM: %s", esp_err_to_name(err));
        return err;
    }

    // Validate the read data by checking the valid_flag
    if (app_settings.valid_flag != APP_CONFIG_VALID_FLAG) {
        ESP_LOGE(TAG, "No valid data in app config: 0x%X", app_settings.valid_flag);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "App config successfully loaded from FRAM.");
    return ESP_OK;
}

esp_err_t YDripComponent::fram_page_write(uint16_t address, const uint8_t* data, size_t size) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    this->set_i2c_address(FRAM_MB85RC64TA_I2C_ADDR);

    esp_err_t err;
    size_t buffer_size = 2 + size;
    uint8_t* buffer = (uint8_t*)malloc(buffer_size);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Set the address bytes in the buffer
    buffer[0] = (address >> 8) & 0xFF;
    buffer[1] = address & 0xFF;

    // Copy user data
    memcpy(&buffer[2], data, size);

    // Write the buffer to the FRAM (address + data)
    esphome::i2c::ErrorCode i2c_err = this->write(buffer, buffer_size, true);
    free(buffer);

    if (i2c_err != esphome::i2c::ErrorCode::ERROR_OK) {
        ESP_LOGE(TAG, "I2C write failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t YDripComponent::fram_sequential_read(uint16_t address, uint8_t* data, size_t size) {
    this->set_i2c_address(FRAM_MB85RC64TA_I2C_ADDR);

    // Prepare the address buffer
    uint8_t addressBuffer[2];
    addressBuffer[0] = (address >> 8) & 0xFF;  // High byte of address
    addressBuffer[1] = address & 0xFF;         // Low byte of address

    // Write the address to the FRAM (without stop condition, since we need to read after)
    esphome::i2c::ErrorCode err = this->write(addressBuffer, sizeof(addressBuffer), false);
    if (err != esphome::i2c::ErrorCode::ERROR_OK) {
        ESP_LOGE(TAG, "Failed to send address to FRAM");
        return ESP_FAIL;
    }

    // Read the data from the FRAM
    err = this->read(data, size);
    if (err != esphome::i2c::ErrorCode::ERROR_OK) {
        ESP_LOGE(TAG, "Failed to read data from FRAM");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t YDripComponent::fram_init() {
    uint8_t dev_id[3] = {0};
    esp_err_t err;

    // Step 1: Send the Reserved Slave ID 0xF8 to initiate the device ID read process
    this->set_i2c_address(FRAM_RESERVED_SLAVE_ID);

    // Step 2: Send the device address (A2, A1, A0 bits, R/W is don't care)
    uint8_t device_address = FRAM_MB85RC64TA_I2C_ADDR << 1;
    err = this->write(&device_address, 1, false);
    if (err != esphome::i2c::ErrorCode::ERROR_OK) {
        ESP_LOGE(TAG, "Failed to send device address: %d", err);
        return ESP_FAIL;
    }

    // Step 3: Re-send START condition followed by Reserved Slave ID 0xF9 for reading
    this->set_i2c_address(FRAM_RESERVED_SLAVE_ID);

    // Step 4: Read 3 bytes (Device ID) from the FRAM
    err = this->read(dev_id, 3);
    if (err != esphome::i2c::ErrorCode::ERROR_OK) {
        ESP_LOGE(TAG, "Failed to read Device ID: %d", err);
        return ESP_FAIL;
    }

    // Step 5: Process the read Device ID
    uint16_t manufacture_id = (dev_id[0] << 4) | (dev_id[1] >> 4);
    uint8_t density_code = dev_id[1] & 0x0F;
    uint16_t product_id = (density_code << 8) | dev_id[2];

    // Step 6: Compare with defined values and log mismatches
    if (manufacture_id != FRAM_FUJITSU_MANUF_ID) {
        ESP_LOGE(TAG, "Manufacturer ID mismatch: expected 0x%X, got 0x%X", FRAM_FUJITSU_MANUF_ID, manufacture_id);
        return ESP_FAIL;
    }
    if (density_code != FRAM_FUJITSU_DENSITY_CODE) {
        ESP_LOGE(TAG, "Density Code mismatch: expected 0x%X, got 0x%X", FRAM_FUJITSU_DENSITY_CODE, density_code);
        return ESP_FAIL;
    }
    if (product_id != FRAM_MB85RC64TA_PROD_ID) {
        ESP_LOGE(TAG, "Product ID mismatch: expected 0x%X, got 0x%X", FRAM_MB85RC64TA_PROD_ID, product_id);
        return ESP_FAIL;
    }

    return ESP_OK;
}

}  // namespace ydrip
}  // namespace esphome




