import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light
from esphome.components.light.types import LightEffect
from esphome.const import CONF_DEFAULT_TRANSITION_LENGTH, CONF_EFFECTS, CONF_ID

from .. import CONF_FELLER_UNI_TASTER_ID, CONF_INDEX, FellerUniTaster, feller_ns

FellerLed = feller_ns.class_("FellerLed", light.LightOutput)
FellerBlinkEffect = feller_ns.class_("FellerBlinkEffect", LightEffect)
CONF_OUTPUT_ID = "output_id"
CONF_BLINK_EFFECT_ID = "blink_effect_id"

CONFIG_SCHEMA = light.light_schema(FellerLed, light.LightType.RGB).extend(
    {
        cv.Optional(CONF_DEFAULT_TRANSITION_LENGTH, default="0s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_EFFECTS): cv.invalid("Only the built-in hardware Blink effect is supported"),
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(FellerLed),
        cv.GenerateID(CONF_BLINK_EFFECT_ID): cv.declare_id(FellerBlinkEffect),
        cv.GenerateID(CONF_FELLER_UNI_TASTER_ID): cv.use_id(FellerUniTaster),
        cv.Required(CONF_INDEX): cv.int_range(min=1, max=8),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_FELLER_UNI_TASTER_ID])
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID], parent, config[CONF_INDEX])
    await light.register_light(var, config)
    state = await cg.get_variable(config[CONF_ID])
    effect = cg.new_Pvariable(config[CONF_BLINK_EFFECT_ID], "Blink")
    cg.add(state.add_effects([effect]))
    cg.add(parent.register_led(var, config[CONF_INDEX]))
