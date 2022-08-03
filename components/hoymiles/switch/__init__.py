import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import hoymiles, switch
from esphome.const import CONF_ID

from .. import hoymiles_ns

HoymilesSwitch = hoymiles_ns.class_('HoymilesSwitch', switch.Switch, cg.Component)

CONF_INVERTER_ID = "inverter_id"

CONFIG_SCHEMA = switch.SWITCH_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(HoymilesSwitch),
    cv.Required(CONF_INVERTER_ID): cv.validate_id_name,
}).extend(cv.COMPONENT_SCHEMA).extend(hoymiles.HOYMILES_DEVICE_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield switch.register_switch(var, config)
    yield hoymiles.register_hoymiles_device(var, config)

    cg.add(var.set_inverter_id(config[CONF_INVERTER_ID]))

