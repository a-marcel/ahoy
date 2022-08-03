import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ICON_CURRENT_AC,

    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_POWER,
    CONF_FREQUENCY,
    CONF_TEMPERATURE,

    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_HERTZ,
    UNIT_WATT_HOURS,
    UNIT_KILOWATT_HOURS,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_TEMPERATURE,

    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
)

from esphome.components import hoymiles, sensor

DEPENDENCIES = ['hoymiles']

AUTO_LOAD = ['sensor']

hoymiles_sensor_ns = cg.esphome_ns.namespace('hoymiles')

HoymilesSensor = hoymiles_sensor_ns.class_('HoymilesSensor', cg.Component)


# CONF_CHANNEL_1 = "channel_1"
CONF_CHANNEL_1 = "channel_1"
CONF_CHANNEL_2 = "channel_2"
CONF_CHANNEL_3 = "channel_3"
CONF_CHANNEL_4 = "channel_4"
CONF_CHANNEL_5 = "channel_5"
CONF_CHANNEL_6 = "channel_6"

CONF_VALUE_YIELD_DAY = "yield_day"
CONF_VALUE_YIELD_TOTAL = "yield_total"
CONF_IRRADIATION="irradiation"

HOYMILES_DC_CHANNEL_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VALUE_YIELD_DAY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VALUE_YIELD_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_IRRADIATION): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),        
    }
)

CONF_GRID = "grid"

HOYMILES_GRID_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
            unit_of_measurement=UNIT_HERTZ,
            icon=ICON_CURRENT_AC,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),            
    }
)

CONF_VALUE_YIELD_DAY = "yield_day"
CONF_VALUE_YIELD_TOTAL = "yield_total"
CONF_GENERAL = "general"

CONF_EFFICIENCY = "efficiency"

HOYMILES_GENERAL_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_VALUE_YIELD_DAY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VALUE_YIELD_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),

        cv.Optional(CONF_EFFICIENCY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


CONF_INVERTER_ID = "inverter_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HoymilesSensor),
    cv.Required(CONF_INVERTER_ID): cv.validate_id_name,

    cv.Optional(CONF_CHANNEL_1): HOYMILES_DC_CHANNEL_SCHEMA,
    cv.Optional(CONF_CHANNEL_2): HOYMILES_DC_CHANNEL_SCHEMA,
    cv.Optional(CONF_CHANNEL_3): HOYMILES_DC_CHANNEL_SCHEMA,    
    cv.Optional(CONF_CHANNEL_4): HOYMILES_DC_CHANNEL_SCHEMA,    
    cv.Optional(CONF_CHANNEL_5): HOYMILES_DC_CHANNEL_SCHEMA,    
    cv.Optional(CONF_CHANNEL_6): HOYMILES_DC_CHANNEL_SCHEMA,    

    cv.Optional(CONF_GRID): HOYMILES_GRID_SCHEMA,    
    cv.Optional(CONF_GENERAL): HOYMILES_GENERAL_SCHEMA,    

}).extend(cv.polling_component_schema('never')).extend(hoymiles.HOYMILES_DEVICE_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    # yield sensor.register_sensor(var, config)
    yield hoymiles.register_hoymiles_device(var, config)

    cg.add(var.set_inverter_id(config[CONF_INVERTER_ID]))

    for i, channel in enumerate([CONF_CHANNEL_1, CONF_CHANNEL_2, CONF_CHANNEL_3, CONF_CHANNEL_4, CONF_CHANNEL_5, CONF_CHANNEL_6]):
        if channel not in config:
            continue
        conf = config[channel]

        if CONF_VOLTAGE in conf:
            sens = yield sensor.new_sensor(conf[CONF_VOLTAGE])
            cg.add(var.set_dc_voltage_sensor(i, sens))
        if CONF_CURRENT in conf:
            sens = yield sensor.new_sensor(conf[CONF_CURRENT])
            cg.add(var.set_dc_current_sensor(i, sens))
        if CONF_POWER in conf:
            sens = yield sensor.new_sensor(conf[CONF_POWER])
            cg.add(var.set_dc_power_sensor(i, sens))
        if CONF_IRRADIATION in conf:
            sens = yield sensor.new_sensor(conf[CONF_IRRADIATION])
            cg.add(var.set_dc_irradiation_sensor(i, sens))
        if CONF_VALUE_YIELD_DAY in conf:
            sens = yield sensor.new_sensor(conf[CONF_VALUE_YIELD_DAY])
            cg.add(var.set_dc_yield_day_sensor(i, sens))
        if CONF_VALUE_YIELD_TOTAL in conf:
            sens = yield sensor.new_sensor(conf[CONF_VALUE_YIELD_TOTAL])
            cg.add(var.set_dc_yield_total_sensor(i, sens))

    if (CONF_GRID in config):

        grid_conf = config[CONF_GRID]

        if CONF_VOLTAGE in grid_conf:
            sens = yield sensor.new_sensor(grid_conf[CONF_VOLTAGE])
            cg.add(var.set_grid_voltage_sensor(sens))
        if CONF_CURRENT in grid_conf:
            sens = yield sensor.new_sensor(grid_conf[CONF_CURRENT])
            cg.add(var.set_grid_current_sensor(sens))
        if CONF_POWER in grid_conf:
            sens = yield sensor.new_sensor(grid_conf[CONF_POWER])
            cg.add(var.set_grid_power_sensor(sens))
        if CONF_FREQUENCY in grid_conf:
            sens = yield sensor.new_sensor(grid_conf[CONF_FREQUENCY])
            cg.add(var.set_grid_frequency_sensor(sens))

    if (CONF_GENERAL in config):

        general_conf = config[CONF_GENERAL]
        if CONF_VALUE_YIELD_DAY in general_conf:
            sens = yield sensor.new_sensor(general_conf[CONF_VALUE_YIELD_DAY])
            cg.add(var.set_general_yield_day_sensor(sens))
        if CONF_VALUE_YIELD_TOTAL in general_conf:
            sens = yield sensor.new_sensor(general_conf[CONF_VALUE_YIELD_TOTAL])
            cg.add(var.set_general_yield_total_sensor(sens))
        if CONF_TEMPERATURE in general_conf:
            sens = yield sensor.new_sensor(general_conf[CONF_TEMPERATURE])
            cg.add(var.set_general_temperatur_sensor(sens))
        if CONF_POWER in general_conf:
            sens = yield sensor.new_sensor(general_conf[CONF_POWER])
            cg.add(var.set_general_power_sensor(sens))
        
        if CONF_EFFICIENCY in general_conf:
            sens = yield sensor.new_sensor(general_conf[CONF_EFFICIENCY])
            cg.add(var.set_general_efficiency_sensor(sens))
