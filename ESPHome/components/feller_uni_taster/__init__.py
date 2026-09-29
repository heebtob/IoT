import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_BAUD_RATE, CONF_ID

CODEOWNERS = ["@heebtob"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

CONF_BYTE_TIMEOUT = "byte_timeout"
CONF_BUTTON_INDICATION = "button_indication"
CONF_SYSTEM_STATE_INDICATION = "system_state_indication"
CONF_RESET_ON_BOOT = "reset_on_boot"
CONF_POLL_INTERVAL = "poll_interval"
CONF_FELLER_UNI_TASTER_ID = "feller_uni_taster_id"
CONF_INDEX = "index"
CONF_RESTORE_VALUE = "restore_value"

feller_ns = cg.esphome_ns.namespace("feller_uni_taster")
FellerUniTaster = feller_ns.class_("FellerUniTaster", cg.Component, uart.UARTDevice)

BAUD_CODES = {
    1200: 0x01,
    2400: 0x02,
    4800: 0x04,
    9600: 0x08,
    19200: 0x10,
    38400: 0x20,
    57600: 0x30,
    115200: 0x60,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FellerUniTaster),
            cv.Optional(CONF_BAUD_RATE, default=9600): cv.one_of(
                *BAUD_CODES, int=True
            ),
            cv.Optional(CONF_BYTE_TIMEOUT, default=0): cv.int_range(min=0, max=255),
            cv.Optional(CONF_BUTTON_INDICATION, default=True): cv.boolean,
            cv.Optional(CONF_SYSTEM_STATE_INDICATION, default=True): cv.boolean,
            cv.Optional(CONF_RESET_ON_BOOT, default=True): cv.boolean,
            cv.Optional(CONF_POLL_INTERVAL, default="0s"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


def FINAL_VALIDATE_SCHEMA(config):
    return uart.final_validate_device_schema(
        "feller_uni_taster",
        baud_rate=config[CONF_BAUD_RATE],
        require_tx=True,
        require_rx=True,
        data_bits=8,
        parity="NONE",
        stop_bits=1,
    )(config)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))
    cg.add(var.set_baud_code(BAUD_CODES[config[CONF_BAUD_RATE]]))
    cg.add(var.set_byte_timeout(config[CONF_BYTE_TIMEOUT]))
    cg.add(var.set_button_indication(config[CONF_BUTTON_INDICATION]))
    cg.add(var.set_system_state_indication(config[CONF_SYSTEM_STATE_INDICATION]))
    cg.add(var.set_reset_on_boot(config[CONF_RESET_ON_BOOT]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL].total_milliseconds))
