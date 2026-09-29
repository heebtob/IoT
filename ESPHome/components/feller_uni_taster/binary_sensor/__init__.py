import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

from .. import CONF_FELLER_UNI_TASTER_ID, CONF_INDEX, FellerUniTaster, feller_ns

FellerButton = feller_ns.class_("FellerButton", binary_sensor.BinarySensor)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(FellerButton).extend(
    {
        cv.GenerateID(CONF_FELLER_UNI_TASTER_ID): cv.use_id(FellerUniTaster),
        cv.Required(CONF_INDEX): cv.int_range(min=1, max=8),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await binary_sensor.register_binary_sensor(var, config)
    parent = await cg.get_variable(config[CONF_FELLER_UNI_TASTER_ID])
    cg.add(parent.register_button(var, config[CONF_INDEX]))
