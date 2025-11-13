# SÚBOR: si4703_fm/switch.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID, CONF_NAME, CONF_ICON, ENTITY_CATEGORY_CONFIG 

# Importujeme hlavné triedy z __init__.py
from . import Si4703FM, Si4703PowerSwitch, Si4703MuteSwitch, CONF_SI4703_FM_ID

# Kľúče pre YAML
CONF_POWER_SWITCH = "power_switch"
CONF_MUTE_SWITCH = "mute_switch" # <--- NOVÉ

# --- Schéma pre Power Switch (LEN pre entitu, bez ID hubu) ---
POWER_SCHEMA = switch.switch_schema(
    Si4703PowerSwitch,
    entity_category=ENTITY_CATEGORY_CONFIG, # Nastaví kategóriu na Konfiguračnú
).extend(
    {
        # Len základné vlastnosti entít
        cv.Optional(CONF_NAME, default="Rádio Power"): cv.string,
        cv.Optional(CONF_ICON, default="mdi:power-settings"): cv.icon,
    }
).extend(cv.COMPONENT_SCHEMA)

# --- NOVÉ: Schéma pre Mute Switch ---
MUTE_SCHEMA = switch.switch_schema(
    Si4703MuteSwitch,
    entity_category=ENTITY_CATEGORY_CONFIG, # Nastaví kategóriu na Konfiguračnú
).extend(
    {
        cv.Optional(CONF_NAME, default="Rádio Mute"): cv.string,
        cv.Optional(CONF_ICON, default="mdi:volume-off"): cv.icon,
    }
).extend(cv.COMPONENT_SCHEMA)

# --- Hlavná schéma pre platformu 'switch.si4703_fm' (S ID hubu na hornej úrovni) ---
CONFIG_SCHEMA = cv.Schema(
    {
        # NOVÉ: ID hubu je teraz povinné na hornej úrovni
        cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
        
        # Power switch je teraz definovaný ako list alebo len jedna entita
        cv.Required(CONF_POWER_SWITCH): POWER_SCHEMA,
        cv.Optional(CONF_MUTE_SWITCH): MUTE_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA) # Predĺžime o základnú komponentnú schému


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SI4703_FM_ID])

    # Kód pre Power Switch
    conf = config[CONF_POWER_SWITCH]
    var = cg.new_Pvariable(conf[CONF_ID], hub)
    await switch.register_switch(var, conf) 
    cg.add(hub.set_power_switch(var))

    # Kód pre Mute Switch
    if CONF_MUTE_SWITCH in config:
        conf = config[CONF_MUTE_SWITCH]
        var = cg.new_Pvariable(conf[CONF_ID], hub)
        await switch.register_switch(var, conf)
        cg.add(hub.set_mute_switch(var))
