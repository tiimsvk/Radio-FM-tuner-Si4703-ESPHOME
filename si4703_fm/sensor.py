# custom_components/si4703_fm/sensor.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
# Pouzivame iba zakladne konstanty. Jednotku ('dBµV') nastavujeme priamo ako retazec.
from esphome.const import CONF_ID, CONF_NAME, CONF_ACCURACY_DECIMALS, ICON_SIGNAL

# Import z __init__.py
from . import si4703_fm_ns, Si4703FM, CONF_SI4703_FM_ID

DEPENDENCIES = ['si4703_fm']

# Definícia schémy pre senzor RSSI
CONFIG_SCHEMA = sensor.sensor_schema(
    # Kritická oprava: Použitie reťazca pre jednotku obchádza chýbajúci import.
    unit_of_measurement='dBµV', 
    icon=ICON_SIGNAL,
    accuracy_decimals=0,
).extend({
    cv.GenerateID(): cv.declare_id(sensor.Sensor),
    # Odkaz na hlavný hub (Si4703FM)
    cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM), 
})

# Funkcia pre registráciu senzora
def to_code(config):
    # Nájdeme parent hub (Si4703FM)
    hub = yield cg.get_variable(config[CONF_SI4703_FM_ID])
    
    # Vytvoríme senzor
    sens = cg.new_Pvariable(config[CONF_ID])
    yield sensor.register_sensor(sens, config)
    
    # Priradíme senzor k hubu pomocou setter metódy
    cg.add(hub.set_rssi_sensor(sens))