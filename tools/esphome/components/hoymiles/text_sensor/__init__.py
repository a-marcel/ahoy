import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    text_sensor,
    hoymiles
)

from esphome.const import (
    CONF_ID,
    CONF_TYPE,
)

from .. import hoymiles_ns

AUTO_LOAD=[
    "text_sensor"
]

HoymilesTextSensor = hoymiles_ns.class_('HoymilesTextSensor', text_sensor.TextSensor, cg.PollingComponent)

HoymilesTextSesnorTypes = hoymiles_ns.enum("HoymilesTextSesnorTypes")

CONF_INVERTER_ID = "inverter_id"


CONF_SUPPORTED_TYPES = {
    "LAST_UPDATED": HoymilesTextSesnorTypes.LAST_UPDATED,
}

CONFIG_SCHEMA = text_sensor.TEXT_SENSOR_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(HoymilesTextSensor),
    cv.Required(CONF_INVERTER_ID): cv.validate_id_name,
    cv.Required(CONF_TYPE): cv.enum(CONF_SUPPORTED_TYPES, upper=True)
}).extend(cv.polling_component_schema('never')).extend(hoymiles.HOYMILES_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await text_sensor.register_text_sensor(var, config)
    await cg.register_component(var, config)
    await hoymiles.register_hoymiles_device(var, config)
    
    cg.add(var.set_inverter_id(config[CONF_INVERTER_ID]))
    cg.add(var.set_type(config[CONF_TYPE]))