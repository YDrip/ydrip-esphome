import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor, i2c
from esphome.const import (
    CONF_ID,
    UNIT_CUBIC_METER,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_WATER,
    STATE_CLASS_TOTAL_INCREASING,
    DEVICE_CLASS_MOISTURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    CONF_FORCE_UPDATE,
    DEVICE_CLASS_BATTERY,
    UNIT_PERCENT
)

DEPENDENCIES = ["i2c", "binary_sensor"]

AUTO_LOAD = [ "binary_sensor" ]

# Define the namespace for YDrip component
ydrip_ns = cg.esphome_ns.namespace("ydrip")
YDripComponent = ydrip_ns.class_("YDripComponent", cg.PollingComponent, i2c.I2CDevice)

# Define configuration keys
CONF_USAGE = "usage"
CONF_LEAK = "leak"
CONF_USAGE_ALERT_COUNT = "usage_alert_count"
CONF_LEAK_ALERT_COUNT = "leak_alert_count"
CONF_LOW_FREQ_LEAK_THRESH = "low_freq_leak_thresh"
CONF_BATTERY_LEVEL = "battery_level"

# Define schema for configuration
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(YDripComponent),
        cv.Optional(CONF_USAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CUBIC_METER,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_WATER,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_LEAK): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOISTURE,
            icon="mdi:water-alert",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_FORCE_UPDATE, default=True): cv.boolean,
        cv.Optional(CONF_USAGE_ALERT_COUNT, default=64): cv.int_range(min=0, max=65535),
        cv.Optional(CONF_LEAK_ALERT_COUNT, default=4): cv.int_range(min=0, max=65535),
        cv.Optional(CONF_LOW_FREQ_LEAK_THRESH, default=2): cv.float_range(min=0.0, max=100.0),
        cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),

    }
).extend(cv.polling_component_schema("60s")).extend(i2c.i2c_device_schema(0x10))


# Code generation
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Handle usage sensor
    if CONF_USAGE in config:
        usage_sensor = await sensor.new_sensor(config[CONF_USAGE])
        cg.add(var.set_usage_sensor(usage_sensor))

    # Handle leak sensor
    if CONF_LEAK in config:
        leak_sensor = await binary_sensor.new_binary_sensor(config[CONF_LEAK])
        cg.add(var.set_leak_sensor(leak_sensor))

    # Configure alert counts and threshold
    if CONF_USAGE_ALERT_COUNT in config:
        cg.add(var.set_usage_alert_count(config[CONF_USAGE_ALERT_COUNT]))

    if CONF_LEAK_ALERT_COUNT in config:
        cg.add(var.set_leak_alert_count(config[CONF_LEAK_ALERT_COUNT]))

    if CONF_LOW_FREQ_LEAK_THRESH in config:
        cg.add(var.set_low_freq_leak_thresh(config[CONF_LOW_FREQ_LEAK_THRESH]))

    # Handle battery level sensor
    if CONF_BATTERY_LEVEL in config:
        battery_sensor = await sensor.new_sensor(config[CONF_BATTERY_LEVEL])
        cg.add(var.set_battery_sensor(battery_sensor))
