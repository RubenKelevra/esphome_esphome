import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_ID,
    DEVICE_CLASS_ILLUMINANCE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_LIGHTBULB,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_EMPTY,
    UNIT_KELVIN,
    UNIT_LUX,
    UNIT_PERCENT,
)

from . import AS7261Component, CONF_AS7261_ID

DEPENDENCIES = ["as7261"]

CONF_CALCULATED_DUV = "calculated_duv"
CONF_CCT = "cct"
CONF_COMPLETED_MEASUREMENT_COUNT = "completed_measurement_count"
CONF_DEVICE_TEMPERATURE = "device_temperature"
CONF_DUV_CIE1976 = "duv_cie1976"
CONF_LUX = "lux"
CONF_NEAR_IR_PERCENT = "near_ir_percent"
CONF_OKLAB_L = "oklab_l"
CONF_OKLAB_A = "oklab_a"
CONF_OKLAB_B = "oklab_b"
CONF_OKLCH_L = "oklch_l"
CONF_OKLCH_C = "oklch_c"
CONF_OKLCH_H = "oklch_h"
CONF_RAW_CLEAR = "raw_clear"
CONF_RAW_DARK = "raw_dark"
CONF_RAW_NEAR_IR = "raw_near_ir"
CONF_X = "x"
CONF_Y = "y"
CONF_Z = "z"

cct_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_KELVIN,
    icon=ICON_THERMOMETER,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

lux_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_LUX,
    device_class=DEVICE_CLASS_ILLUMINANCE,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)

unitless_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_EMPTY,
    accuracy_decimals=6,
    state_class=STATE_CLASS_MEASUREMENT,
)

def diagnostic_schema(schema):
    return schema.extend(
        {cv.Optional(CONF_DISABLED_BY_DEFAULT, default=True): cv.boolean}
    )


diagnostic_unitless_schema = diagnostic_schema(
    sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        accuracy_decimals=6,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        state_class=STATE_CLASS_MEASUREMENT,
    )
)

diagnostic_counts_schema = diagnostic_schema(
    sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        icon=ICON_LIGHTBULB,
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        state_class=STATE_CLASS_MEASUREMENT,
    )
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_AS7261_ID): cv.use_id(AS7261Component),
        cv.Optional(CONF_CCT): cct_schema,
        cv.Optional(CONF_CALCULATED_DUV): unitless_schema,
        cv.Optional(CONF_LUX): lux_schema,
        cv.Optional(CONF_OKLAB_L): unitless_schema,
        cv.Optional(CONF_OKLAB_A): unitless_schema,
        cv.Optional(CONF_OKLAB_B): unitless_schema,
        cv.Optional(CONF_OKLCH_L): unitless_schema,
        cv.Optional(CONF_OKLCH_C): unitless_schema,
        cv.Optional(CONF_OKLCH_H): unitless_schema,
        cv.Optional(CONF_COMPLETED_MEASUREMENT_COUNT): diagnostic_counts_schema,
        cv.Optional(CONF_DEVICE_TEMPERATURE): diagnostic_schema(
            sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        cv.Optional(CONF_DUV_CIE1976): diagnostic_unitless_schema,
        cv.Optional(CONF_NEAR_IR_PERCENT): diagnostic_schema(
            sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        cv.Optional(CONF_RAW_CLEAR): diagnostic_counts_schema,
        cv.Optional(CONF_RAW_DARK): diagnostic_counts_schema,
        cv.Optional(CONF_RAW_NEAR_IR): diagnostic_counts_schema,
        cv.Optional(CONF_X): diagnostic_unitless_schema,
        cv.Optional(CONF_Y): diagnostic_unitless_schema,
        cv.Optional(CONF_Z): diagnostic_unitless_schema,
    }
)

SENSORS = {
    CONF_CCT: "set_cct_sensor",
    CONF_CALCULATED_DUV: "set_calculated_duv_sensor",
    CONF_LUX: "set_lux_sensor",
    CONF_OKLAB_L: "set_oklab_l_sensor",
    CONF_OKLAB_A: "set_oklab_a_sensor",
    CONF_OKLAB_B: "set_oklab_b_sensor",
    CONF_OKLCH_L: "set_oklch_l_sensor",
    CONF_OKLCH_C: "set_oklch_c_sensor",
    CONF_OKLCH_H: "set_oklch_h_sensor",
    CONF_COMPLETED_MEASUREMENT_COUNT: "set_completed_measurement_count_sensor",
    CONF_DEVICE_TEMPERATURE: "set_device_temperature_sensor",
    CONF_DUV_CIE1976: "set_duv_cie1976_sensor",
    CONF_NEAR_IR_PERCENT: "set_near_ir_percent_sensor",
    CONF_RAW_CLEAR: "set_raw_clear_sensor",
    CONF_RAW_DARK: "set_raw_dark_sensor",
    CONF_RAW_NEAR_IR: "set_raw_near_ir_sensor",
    CONF_X: "set_x_sensor",
    CONF_Y: "set_y_sensor",
    CONF_Z: "set_z_sensor",
}


async def to_code(config):
    component = await cg.get_variable(config[CONF_AS7261_ID])
    for conf_id, setter in SENSORS.items():
        if sensor_config := config.get(conf_id):
            sens = await sensor.new_sensor(sensor_config)
            cg.add(getattr(component, setter)(sens))
