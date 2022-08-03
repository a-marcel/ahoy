import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import (
    button,
    hoymiles
)

from esphome.const import (
    CONF_ID,
    CONF_TYPE,
    DEVICE_CLASS_RESTART,
    ENTITY_CATEGORY_CONFIG,    
)

from .. import hoymiles_ns

AUTO_LOAD=[
    "button"
]

HoymilesButton = hoymiles_ns.class_("HoymilesButton", button.Button, cg.Component)

HoymilesButtonTypes = hoymiles_ns.enum("HoymilesButtonTypes")


CONF_INVERTER_ID = "inverter_id"

CONF_SUPPORTED_TYPES = {
    "RESTART": HoymilesButtonTypes.RESTART,
    "CLEAN_STATE": HoymilesButtonTypes.CLEAN_STATE

}

CONFIG_SCHEMA = (
    button.button_schema()
    .extend(
        {
            cv.GenerateID(): cv.declare_id(HoymilesButton),
            cv.Required(CONF_INVERTER_ID): cv.validate_id_name,
           cv.Required(CONF_TYPE): cv.enum(CONF_SUPPORTED_TYPES, upper=True)
        }
    )
    .extend(cv.COMPONENT_SCHEMA).extend(hoymiles.HOYMILES_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await button.register_button(var, config)
    await cg.register_component(var, config)
    await hoymiles.register_hoymiles_device(var, config)

    cg.add(var.set_inverter_id(config[CONF_INVERTER_ID]))
    cg.add(var.set_type(config[CONF_TYPE]))