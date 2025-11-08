import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID, CONF_MODE, CONF_NAME, CONF_ICON, CONF_UNIT_OF_MEASUREMENT,
    CONF_STEP, CONF_MIN_VALUE, CONF_MAX_VALUE, CONF_INITIAL_VALUE,
    CONF_OPTIMISTIC,
)
# Importujeme hlavné triedy z __init__.py
from . import Si4703FM, Si4703Frequency, Si4703Volume, si4703_fm_ns, CONF_SI4703_FM_ID

# Definujeme kľúče, ktoré hľadáme v YAML
CONF_NUMBER_FREQUENCY = "number_frequency"
CONF_NUMBER_VOLUME = "number_volume"

# --- Schéma pre Frekvenciu ---
FREQUENCY_SCHEMA = number.NUMBER_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(Si4703Frequency),
    cv.Optional(CONF_MIN_VALUE, default=87.5): cv.float_,
    cv.Optional(CONF_MAX_VALUE, default=108.0): cv.float_,
    cv.Optional(CONF_STEP, default=0.1): cv.float_,
    cv.Optional(CONF_INITIAL_VALUE, default=104.8): cv.float_,
    cv.Optional(CONF_OPTIMISTIC, default=True): cv.boolean,
    cv.Optional(CONF_UNIT_OF_MEASUREMENT, default="MHz"): cv.string,
    cv.Optional(CONF_ICON, default="mdi:radio-tower"): cv.icon,
    cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
}).extend(cv.COMPONENT_SCHEMA)

# --- Schéma pre Hlasitosť ---
VOLUME_SCHEMA = number.NUMBER_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(Si4703Volume),
    cv.Optional(CONF_MIN_VALUE, default=0.0): cv.float_,
    cv.Optional(CONF_MAX_VALUE, default=15.0): cv.float_, # Max Si4703 je 15
    cv.Optional(CONF_STEP, default=1.0): cv.float_,
    cv.Optional(CONF_INITIAL_VALUE, default=8.0): cv.float_,
    cv.Optional(CONF_OPTIMISTIC, default=True): cv.boolean,
    cv.Optional(CONF_ICON, default="mdi:volume-medium"): cv.icon,
    cv.Optional(CONF_MODE, default="SLIDER"): cv.enum(number.NUMBER_MODES, upper=True),
}).extend(cv.COMPONENT_SCHEMA)

# --- Hlavná schéma pre platformu 'number.si4703_fm' ---
CONFIG_SCHEMA = cv.Schema({
    # Povinný odkaz na hlavný hub
    cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
    
    # Voliteľné: Definícia frekvencie
    cv.Optional(CONF_NUMBER_FREQUENCY): FREQUENCY_SCHEMA,
    
    # Voliteľné: Definícia hlasitosti
    cv.Optional(CONF_NUMBER_VOLUME): VOLUME_SCHEMA,
})

async def to_code(config):
    # Získame odkaz na hlavný hub
    hub = await cg.get_variable(config[CONF_SI4703_FM_ID])

    # Skontrolujeme, či je definovaná frekvencia
    if CONF_NUMBER_FREQUENCY in config:
        conf = config[CONF_NUMBER_FREQUENCY]
        var = cg.new_Pvariable(conf[CONF_ID], hub)
        await cg.register_component(var, conf)
        await number.register_number(
            var, conf, 
            min_value=conf[CONF_MIN_VALUE], 
            max_value=conf[CONF_MAX_VALUE], 
            step=conf[CONF_STEP]
        )
        cg.add(hub.set_frequency_number(var))

    # Skontrolujeme, či je definovaná hlasitosť
    if CONF_NUMBER_VOLUME in config:
        conf = config[CONF_NUMBER_VOLUME]
        var = cg.new_Pvariable(conf[CONF_ID], hub)
        await cg.register_component(var, conf)
        await number.register_number(
            var, conf, 
            min_value=conf[CONF_MIN_VALUE], 
            max_value=conf[CONF_MAX_VALUE], 
            step=conf[CONF_STEP]
        )
        cg.add(hub.set_volume_number(var))
