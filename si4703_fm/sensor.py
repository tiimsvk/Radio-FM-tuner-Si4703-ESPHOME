import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, 
    CONF_ACCURACY_DECIMALS, 
    ICON_SIGNAL,
    ICON_GAUGE,
    CONF_UNIT_OF_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
# Import z __init__.py
from . import si4703_fm_ns, Si4703FM, CONF_SI4703_FM_ID

# Konštanty pre senzory
CONF_RSSI = "rssi" # OPRAVENÉ: Použijeme "rssi" kľúč
CONF_SNR = "snr"
#CONF_BLER_A = "bler_a"
#CONF_BLER_D = "bler_d"
CONF_PTY = "pty"

# Hlavná schéma pre celý modul
CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
    
    # --- RSSI Senzor ---
    cv.Optional(CONF_RSSI): sensor.sensor_schema( # OPRAVENÉ: Použitie CONF_RSSI
        unit_of_measurement='dBµV', 
        icon=ICON_SIGNAL,
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ).extend({
        cv.GenerateID(): cv.declare_id(sensor.Sensor),
    }),
    
    # --- NOVÉ: SNR Senzor ---
    cv.Optional(CONF_SNR): sensor.sensor_schema(
        unit_of_measurement='dB', 
        icon=ICON_GAUGE,
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ).extend({
        cv.GenerateID(): cv.declare_id(sensor.Sensor),
    }),

    ## --- NOVÉ: BLER A Senzor ---
    #cv.Optional(CONF_BLER_A): sensor.sensor_schema(
    #    unit_of_measurement='BLER', # Block Error Rate (rozsah 0-3)
    #    icon=ICON_GAUGE,
    #    accuracy_decimals=0,
    #).extend({
    #    cv.GenerateID(): cv.declare_id(sensor.Sensor),
    #}),
    #
    ## --- NOVÉ: BLER D Senzor ---
    #cv.Optional(CONF_BLER_D): sensor.sensor_schema(
    #    unit_of_measurement='BLER', # Block Error Rate (rozsah 0-3)
    #    icon=ICON_GAUGE,
    #    accuracy_decimals=0,
    #).extend({
    #    cv.GenerateID(): cv.declare_id(sensor.Sensor),
    #}),
    
    # --- PTY Sensor
    cv.Optional(CONF_PTY): sensor.sensor_schema(
        unit_of_measurement='PTY Code', # Programme Type Code (rozsah 0-31)
        icon=ICON_GAUGE,
        accuracy_decimals=0,
    ).extend({
        cv.GenerateID(): cv.declare_id(sensor.Sensor),
    }),
    
}).extend(cv.COMPONENT_SCHEMA)

# Funkcia pre registráciu senzorov
async def to_code(config):
    hub_var = await cg.get_variable(config[CONF_SI4703_FM_ID])
    
    # 1. OPRAVENÉ: RSSI
    if CONF_RSSI in config:
        conf = config[CONF_RSSI]
        var = await sensor.new_sensor(conf)
        cg.add(hub_var.set_rssi_sensor(var)) # set_rssi_sensor je v C++
        
    # 2. NOVÉ: SNR
    if CONF_SNR in config:
        conf = config[CONF_SNR]
        var = await sensor.new_sensor(conf)
        cg.add(hub_var.set_snr_sensor(var))

    ## 3. NOVÉ: BLER A
    #if CONF_BLER_A in config:
    #    conf = config[CONF_BLER_A]
    #    var = await sensor.new_sensor(conf)
    #    cg.add(hub_var.set_bler_a_sensor(var))
    #
    ## 4. NOVÉ: BLER D
    #if CONF_BLER_D in config:
    #    conf = config[CONF_BLER_D]
    #    var = await sensor.new_sensor(conf)
    #    cg.add(hub_var.set_bler_d_sensor(var))
    
    # 5. PTY
    if CONF_PTY in config:
        conf = config[CONF_PTY]
        var = await sensor.new_sensor(conf)
        cg.add(hub_var.set_pty_sensor(var)) # set_pty_sensor je v C++
