import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID, CONF_RESTORE_VALUE, ENTITY_CATEGORY_CONFIG

from .. import CONF_FELLER_UNI_TASTER_ID, FellerUniTaster, feller_ns

FellerBrightness = feller_ns.class_("FellerBrightness", number.Number, cg.Component)

CONFIG_SCHEMA = number.number_schema(
    FellerBrightness, entity_category=ENTITY_CATEGORY_CONFIG
).extend(
    {
        cv.GenerateID(CONF_FELLER_UNI_TASTER_ID): cv.use_id(FellerUniTaster),
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await number.register_number(var, config, min_value=0, max_value=255, step=1)
    parent = await cg.get_variable(config[CONF_FELLER_UNI_TASTER_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_restore_value(config[CONF_RESTORE_VALUE]))
    cg.add(parent.register_brightness(var))
