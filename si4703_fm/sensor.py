import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, 
    CONF_ACCURACY_DECIMALS, 
    ICON_SIGNAL,
    #CONF_UPDATE_INTERVAL
)
# Import z __init__.py
from . import si4703_fm_ns, Si4703FM, CONF_SI4703_FM_ID

# DEPENDENCIES = ['si4703_fm'] # Odstránené - spôsobuje circular dependency

# Definícia schémy pre senzor RSSI
CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement='dBµV', 
    icon=ICON_SIGNAL,
    accuracy_decimals=0,
).extend({
    cv.GenerateID(): cv.declare_id(sensor.Sensor),

    cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
    
    # Senzor bude používať update_interval z hlavného hubu
    # cv.Optional(CONF_UPDATE_INTERVAL, default='5s'): cv.positive_time_period_milliseconds,
}).extend(cv.COMPONENT_SCHEMA) # PollingComponent tu nie je potrebný, Hub robí polling

# Funkcia pre registráciu senzora
async def to_code(config):
    # 1. Získame hub
    hub_var = await cg.get_variable(config[CONF_SI4703_FM_ID])
    
    # 2. Vytvoríme štandardný senzor (toto nahradí new_Pvariable aj register_sensor)
    var = await sensor.new_sensor(config)
    
    # 3. Pripojíme senzor k hubu
    cg.add(hub_var.set_rssi_sensor(var))
