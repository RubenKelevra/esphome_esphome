import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG

from .. import AS7261Component, as7261_ns

DEPENDENCIES = ["as7261"]

CONF_MEASURE = "measure"

AS7261MeasureButton = as7261_ns.class_("AS7261MeasureButton", button.Button)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.use_id(AS7261Component),
        cv.Required(CONF_MEASURE): button.button_schema(
            AS7261MeasureButton,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:play",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    measure_button = await button.new_button(config[CONF_MEASURE])
    await cg.register_parented(measure_button, parent)
