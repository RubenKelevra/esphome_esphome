import math

from esphome import pins
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_GAIN,
    CONF_ID,
    CONF_INTEGRATION_TIME,
    CONF_INVERTED,
    CONF_RESET_PIN,
)

CODEOWNERS = ["@cyrond"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

as7261_ns = cg.esphome_ns.namespace("as7261")
AS7261Component = as7261_ns.class_(
    "AS7261Component", cg.PollingComponent, uart.UARTDevice
)

CONF_AS7261_ID = "as7261_id"
CONF_INT_PIN = "int_pin"
CONF_PRECISION_MODE = "precision_mode"
CONF_OKLAB_REFERENCE_ILLUMINANCE = "oklab_reference_illuminance"

DEFAULT_OKLAB_REFERENCE_ILLUMINANCE = 1000.0
OKLAB_REFERENCE_ILLUMINANCE_PRESETS = {
    "asr_a3_4_color_inspection": DEFAULT_OKLAB_REFERENCE_ILLUMINANCE,
}

AS7261Gain = as7261_ns.enum("AS7261Gain")
AS7261_GAINS = {
    "1X": AS7261Gain.AS7261_GAIN_1X,
    "3.7X": AS7261Gain.AS7261_GAIN_3_7X,
    "16X": AS7261Gain.AS7261_GAIN_16X,
    "64X": AS7261Gain.AS7261_GAIN_64X,
}


def validate_exposure_config(config):
    has_gain = CONF_GAIN in config
    has_integration_time = CONF_INTEGRATION_TIME in config
    if has_gain != has_integration_time:
        raise cv.Invalid(
            f"{CONF_GAIN} and {CONF_INTEGRATION_TIME} must be configured together; "
            "omit both to use auto exposure"
        )
    return config


def validate_oklab_reference_illuminance(value):
    if isinstance(value, str):
        normalized_value = cv.string(value).strip().lower().replace("-", "_")
        if normalized_value in OKLAB_REFERENCE_ILLUMINANCE_PRESETS:
            return OKLAB_REFERENCE_ILLUMINANCE_PRESETS[normalized_value]

    illuminance = cv.float_with_unit(
        "illuminance", "(lx|lux)?", optional_unit=True
    )(value)
    if not math.isfinite(illuminance) or illuminance <= 0.0:
        raise cv.Invalid(
            f"{CONF_OKLAB_REFERENCE_ILLUMINANCE} must be a finite positive illuminance"
        )
    return illuminance


def validate_active_low_pin(config):
    if not config.get(CONF_INVERTED):
        raise cv.Invalid("AS7261 pins are active-low; set inverted: true")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS7261Component),
            cv.Required(CONF_INT_PIN): cv.All(
                pins.gpio_input_pin_schema, validate_active_low_pin
            ),
            cv.Required(CONF_RESET_PIN): cv.All(
                pins.gpio_output_pin_schema, validate_active_low_pin
            ),
            cv.Optional(CONF_PRECISION_MODE, default=False): cv.boolean,
            cv.Optional(CONF_GAIN): cv.enum(AS7261_GAINS, upper=True),
            cv.Optional(CONF_INTEGRATION_TIME): cv.int_range(min=1, max=255),
            cv.Optional(
                CONF_OKLAB_REFERENCE_ILLUMINANCE,
                default=DEFAULT_OKLAB_REFERENCE_ILLUMINANCE,
            ): validate_oklab_reference_illuminance,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA),
    validate_exposure_config,
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "as7261",
    baud_rate=115200,
    require_rx=True,
    require_tx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    int_pin = await cg.gpio_pin_expression(config[CONF_INT_PIN])
    cg.add(var.set_int_pin(int_pin))

    reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(reset_pin))

    cg.add(var.set_precision_mode(config[CONF_PRECISION_MODE]))
    cg.add(
        var.set_oklab_reference_illuminance(
            config[CONF_OKLAB_REFERENCE_ILLUMINANCE]
        )
    )

    if CONF_GAIN in config:
        cg.add(var.set_manual_exposure(config[CONF_GAIN], config[CONF_INTEGRATION_TIME]))
