import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID, CONF_MODE, CONF_NAME, CONF_ICON, CONF_UNIT_OF_MEASUREMENT,
    CONF_STEP, CONF_MIN_VALUE, CONF_MAX_VALUE, CONF_INITIAL_VALUE,
    CONF_OPTIMISTIC,
)
# Importujeme hlavnú triedu z __init__.py
from . import Si4703FM, si4703_fm_ns, CONF_SI4703_FM_ID

# Definujeme jednotky a ikony
UNIT_MEGHERTZ = "MHz" 
ICON_RADIATOR_HEATER = "mdi:radio-tower"

# Definujeme triedu pre túto platformu
Si4703Frequency = si4703_fm_ns.class_("Si4703Frequency", number.Number, cg.Component)

# Toto je schéma pre platformu number:
CONFIG_SCHEMA = number.NUMBER_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(Si4703Frequency),
    cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM), # Prepojenie na hlavný hub
    
    # Špecifické nastavenia pre frekvenciu
    cv.Optional(CONF_MIN_VALUE, default=87.5): cv.float_,
    cv.Optional(CONF_MAX_VALUE, default=108.0): cv.float_,
    cv.Optional(CONF_STEP, default=0.1): cv.float_,
    cv.Optional(CONF_INITIAL_VALUE, default=98.0): cv.float_,
    cv.Optional(CONF_OPTIMISTIC, default=True): cv.boolean,
    cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_MEGHERTZ): cv.string,
    cv.Optional(CONF_ICON, default=ICON_RADIATOR_HEATER): cv.icon,
    cv.Optional(CONF_MODE, default="BOX"): cv.enum(
      number.NUMBER_MODES, 
      upper=True
    ),
}).extend(cv.COMPONENT_SCHEMA)

# Nová (správna) verzia
async def to_code(config):
    # 1. Vytvorenie Number entity
    var = cg.new_Pvariable(config[CONF_ID], await cg.get_variable(config[CONF_SI4703_FM_ID]))
    
    # 2. Registrácia Number entity (používame fix z predchádzajúceho kroku)
    await cg.register_component(var, config) 
    await number.register_number(
        var, 
        config, 
        min_value=config[CONF_MIN_VALUE], 
        max_value=config[CONF_MAX_VALUE], 
        step=config[CONF_STEP]
    ) 
    
    # 3. Získanie odkazu na hlavný komponent (hub) a prepojenie
    # ZMENA: POÈKAJ NA VÝSLEDOK cg.get_variable()
    hub_var = await cg.get_variable(config[CONF_SI4703_FM_ID])
    
    # Teraz hub_var už nie je coroutine, ale C++ Pvariable
    cg.add(hub_var.set_frequency_number(var))