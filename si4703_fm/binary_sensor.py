# SÚBOR: si4703_fm/binary_sensor.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_ID, 
    CONF_NAME, 
    CONF_ICON, # ikona pre ucho / skenovanie
    ENTITY_CATEGORY_DIAGNOSTIC
)

# Importujeme triedy z __init__.py
from . import Si4703FM, Si4703StereoIndicator, CONF_SI4703_FM_ID
from . import Si4703TPIndicator, Si4703TAIndicator

CONF_STEREO_INDICATOR = "stereo_indicator"
CONF_TP_INDICATOR = "tp_indicator"
CONF_TA_INDICATOR = "ta_indicator"

# --- Schéma pre Stereo Indikátor ---
STEREO_INDICATOR_SCHEMA = binary_sensor.binary_sensor_schema(
    Si4703StereoIndicator,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    icon=CONF_ICON,
).extend(
    {
        cv.Optional(CONF_NAME, default="Rádio Stereo Indikátor"): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)

# --- Schéma pre TP Indicator (Traffic Programme) ---
TP_INDICATOR_SCHEMA = binary_sensor.binary_sensor_schema(
    Si4703TPIndicator,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    icon="mdi:road-variant", # Ikona pre dopravný program
).extend(
    {
        cv.Optional(CONF_NAME, default="RDS Dopravný Program (TP)"): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)

# --- Schéma pre TA Indicator (Traffic Announcement) ---
TA_INDICATOR_SCHEMA = binary_sensor.binary_sensor_schema(
    Si4703TAIndicator,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    icon="mdi:car-emergency", # Ikona pre aktívne hlásenie
).extend(
    {
        cv.Optional(CONF_NAME, default="RDS Dopravné Hlásenie (TA)"): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)

# --- Hlavná konfiguračná schéma ---
CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
        cv.Optional(CONF_STEREO_INDICATOR): STEREO_INDICATOR_SCHEMA,
        cv.Optional(CONF_TP_INDICATOR): TP_INDICATOR_SCHEMA,
        cv.Optional(CONF_TA_INDICATOR): TA_INDICATOR_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)

# --- Kód pre generovanie ---
async def to_code(config):
    hub_var = await cg.get_variable(config[CONF_SI4703_FM_ID])

    if CONF_STEREO_INDICATOR in config:
        conf = config[CONF_STEREO_INDICATOR]
        # Vytvorenie inštancie C++ triedy Stereo Indicator
        var = cg.new_Pvariable(conf[CONF_ID], hub_var)
        await binary_sensor.register_binary_sensor(var, conf)
        # Priradenie inštancie k hlavnému hubu Si4703FM
        cg.add(hub_var.set_stereo_indicator_sensor(var))
        
    if CONF_TP_INDICATOR in config:
        conf = config[CONF_TP_INDICATOR]
        var = cg.new_Pvariable(conf[CONF_ID], hub_var)
        await binary_sensor.register_binary_sensor(var, conf)
        cg.add(hub_var.set_tp_indicator_sensor(var))

    if CONF_TA_INDICATOR in config:
        conf = config[CONF_TA_INDICATOR]
        var = cg.new_Pvariable(conf[CONF_ID], hub_var)
        await binary_sensor.register_binary_sensor(var, conf)
        cg.add(hub_var.set_ta_indicator_sensor(var))