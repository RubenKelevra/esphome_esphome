import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_CHIP,
)

from . import AS7261Component, CONF_AS7261_ID

DEPENDENCIES = ["as7261"]

CONF_FIRMWARE_VERSION = "firmware_version"


def diagnostic_schema(schema):
    return schema.extend(
        {cv.Optional(CONF_DISABLED_BY_DEFAULT, default=True): cv.boolean}
    )


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_AS7261_ID): cv.use_id(AS7261Component),
        cv.Optional(CONF_FIRMWARE_VERSION): diagnostic_schema(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_CHIP,
            )
        ),
    }
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_AS7261_ID])
    if firmware_version_config := config.get(CONF_FIRMWARE_VERSION):
        sens = await text_sensor.new_text_sensor(firmware_version_config)
        cg.add(component.set_firmware_version_text_sensor(sens))
