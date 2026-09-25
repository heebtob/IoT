import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_PORT,
    CONF_TX_PIN,
    PLATFORM_ESP32,
)
from esphome.core import CORE

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["network", "socket"]

CONF_UART_NUM = "uart_num"
CONF_DE_PIN = "de_pin"
CONF_NET = "net"
CONF_SUBNET = "subnet"
CONF_UNIVERSE = "universe"
CONF_CHANNELS = "channels"
CONF_START_CHANNEL = "start_channel"
CONF_REFRESH_RATE = "refresh_rate"
CONF_TIMEOUT = "timeout"
CONF_TIMEOUT_ACTION = "timeout_action"
CONF_SHORT_NAME = "short_name"
CONF_LONG_NAME = "long_name"

artnet_dmx_ns = cg.esphome_ns.namespace("artnet_dmx")
ArtNetDMX = artnet_dmx_ns.class_("ArtNetDMX", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ArtNetDMX),
            cv.Optional(CONF_UART_NUM, default=1): cv.int_range(min=0, max=2),
            cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_DE_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_NET, default=0): cv.int_range(min=0, max=127),
            cv.Optional(CONF_SUBNET, default=0): cv.int_range(min=0, max=15),
            cv.Optional(CONF_UNIVERSE, default=0): cv.int_range(min=0, max=15),
            cv.Optional(CONF_PORT, default=6454): cv.port,
            cv.Optional(CONF_CHANNELS, default=512): cv.int_range(min=1, max=512),
            cv.Optional(CONF_START_CHANNEL, default=1): cv.int_range(min=1, max=512),
            cv.Optional(CONF_REFRESH_RATE, default=40.0): cv.float_range(min=1, max=44),
            cv.Optional(CONF_TIMEOUT, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_TIMEOUT_ACTION, default="hold"): cv.one_of(
                "hold", "blackout", lower=True
            ),
            cv.Optional(CONF_SHORT_NAME, default=CORE.name[:17]): cv.All(
                cv.string, cv.Length(max=17)
            ),
            cv.Optional(CONF_LONG_NAME, default=CORE.name[:63]): cv.All(
                cv.string, cv.Length(max=63)
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on(PLATFORM_ESP32),
    cv.only_with_framework("esp-idf"),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))
    if CONF_DE_PIN in config:
        de_pin = await cg.gpio_pin_expression(config[CONF_DE_PIN])
        cg.add(var.set_de_pin(de_pin))

    cg.add(var.set_uart_num(config[CONF_UART_NUM]))
    cg.add(var.set_net(config[CONF_NET]))
    cg.add(var.set_subnet(config[CONF_SUBNET]))
    cg.add(var.set_universe(config[CONF_UNIVERSE]))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_channels(config[CONF_CHANNELS]))
    cg.add(var.set_start_channel(config[CONF_START_CHANNEL]))
    cg.add(var.set_refresh_rate(config[CONF_REFRESH_RATE]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT].total_milliseconds))
    cg.add(var.set_timeout_blackout(config[CONF_TIMEOUT_ACTION] == "blackout"))
    cg.add(var.set_short_name(config[CONF_SHORT_NAME]))
    cg.add(var.set_long_name(config[CONF_LONG_NAME]))
