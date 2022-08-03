import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import (
    number,
    hoymiles
)

from esphome.const import (
    CONF_MAX_POWER
)

from .. import hoymiles_ns

AUTO_LOAD=[
    "number"
]

HoymilesLimit = hoymiles_ns.class_("HoymilesLimit", number.Number, cg.Component)

CONF_INVERTER_ID = "inverter_id"

CONFIG_SCHEMA = number.NUMBER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(HoymilesLimit),
        cv.Required(CONF_INVERTER_ID): cv.validate_id_name,
        cv.Required(CONF_MAX_POWER): cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA).extend(hoymiles.HOYMILES_DEVICE_SCHEMA)


async def to_code(config):

    max_value = config[CONF_MAX_POWER]
    
    var = await number.new_number(config, min_value=0, max_value=max_value, step=1)
    await cg.register_component(var, config)

    cg.add(var.set_max_power(config[CONF_MAX_POWER]))

    await hoymiles.register_hoymiles_device(var, config)

    cg.add(var.set_inverter_id(config[CONF_INVERTER_ID]))