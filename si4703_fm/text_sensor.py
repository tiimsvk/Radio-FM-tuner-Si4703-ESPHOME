import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, CONF_NAME, CONF_ICON

# Import z __init__.py
from . import si4703_fm_ns, Si4703FM, CONF_SI4703_FM_ID

# ODSTRÁNENÁ RIADKA: Si4703RdsTextSensor = si4703_fm_ns.class_("Si4703RdsTextSensor", cg.Component, text_sensor.TextSensor)
# Teraz priamo používame text_sensor.TextSensor ako typ!

# Definujeme kľúč
CONF_RDS_TEXT = "rds_text"
CONF_RDS_PS = "rds_ps"

# --- Schéma pre RDS PS Sensor ---
RDS_PS_SCHEMA = text_sensor.TEXT_SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(text_sensor.TextSensor), 
        cv.Optional(CONF_NAME, default="FM Názov Stanice"): cv.string, # Zmena default názvu
        cv.Optional(text_sensor.CONF_ICON, default="mdi:radio"): cv.icon, # Iná ikona
    }
).extend(cv.COMPONENT_SCHEMA)

# --- Schéma pre RDS Text Sensor ---
RDS_TEXT_SCHEMA = text_sensor.TEXT_SENSOR_SCHEMA.extend(
    {
        # Používame štandardné ID pre text_sensor.TextSensor
        cv.GenerateID(): cv.declare_id(text_sensor.TextSensor), 
        cv.Optional(CONF_NAME, default="FM Rádio Text"): cv.string,
        cv.Optional(text_sensor.CONF_ICON, default=CONF_ICON): cv.icon,
    }
).extend(cv.COMPONENT_SCHEMA)

# --- Hlavná schéma pre platformu 'text_sensor.si4703_fm' ---
CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
        cv.Optional(CONF_RDS_TEXT): RDS_TEXT_SCHEMA,
        cv.Optional(CONF_RDS_PS): RDS_PS_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # KĽÚČOVÁ OPRAVA: Premenná hub_var musí byť definovaná prvá a UNIVERZÁLNE!
    hub_var = await cg.get_variable(config[CONF_SI4703_FM_ID])

    # Generovanie RT senzora (pôvodný kód)
    if CONF_RDS_TEXT in config:
        text_sensor_var = await text_sensor.new_text_sensor(config[CONF_RDS_TEXT])
        # Riadok 50: Teraz je hub_var garantovane definovaný.
        cg.add(hub_var.set_rds_text_sensor(text_sensor_var))

    # Generovanie PS senzora (NOVÝ kód)
    if CONF_RDS_PS in config:
        ps_sensor_var = await text_sensor.new_text_sensor(config[CONF_RDS_PS])
        # Použitie už definovanej hub_var
        cg.add(hub_var.set_rds_ps_sensor(ps_sensor_var))