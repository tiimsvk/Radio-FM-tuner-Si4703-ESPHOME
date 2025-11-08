# SÚBOR: si4703_fm/switch.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID, CONF_NAME, CONF_ICON, ENTITY_CATEGORY_CONFIG 

# Importujeme hlavné triedy z __init__.py
from . import Si4703FM, Si4703PowerSwitch, CONF_SI4703_FM_ID

# Kľúč pre YAML
CONF_POWER_SWITCH = "power_switch"


# --- Schéma pre Power Switch (LEN pre entitu, bez ID hubu) ---
# Používame switch.switch_schema, ktorá sa stará o register komponentu a entitu
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


# --- Hlavná schéma pre platformu 'switch.si4703_fm' (S ID hubu na hornej úrovni) ---
CONFIG_SCHEMA = cv.Schema(
    {
        # NOVÉ: ID hubu je teraz povinné na hornej úrovni
        cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
        
        # Power switch je teraz definovaný ako list alebo len jedna entita
        cv.Required(CONF_POWER_SWITCH): POWER_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA) # Predĺžime o základnú komponentnú schému


async def to_code(config):
    # 1. Získame odkaz na hlavný hub (ID hubu je na hornej úrovni 'config')
    hub = await cg.get_variable(config[CONF_SI4703_FM_ID])

    # 2. Vytvoríme switch
    conf = config[CONF_POWER_SWITCH]
    
    # Vytvorenie inštancie triedy Si4703PowerSwitch (s hubom ako parent)
    var = cg.new_Pvariable(conf[CONF_ID], hub) 
    
    # 3. Zaregistrujeme ho
    await cg.register_component(var, conf)
    await switch.register_switch(var, conf) # Správna registrácia switch entít
    
    # 4. Pripojíme pointer na switch k hlavnému hubu (cez setter)
    cg.add(hub.set_power_switch(var))