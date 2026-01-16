# SÚBOR: si4703_fm/switch.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID, CONF_NAME, CONF_ICON, ENTITY_CATEGORY_CONFIG 

from . import (
    Si4703FM,
    Si4703PowerSwitch,
    Si4703MuteSwitch,
    Si4703AmpSwitch,
    Si4703Gpio2Switch,
    CONF_SI4703_FM_ID,
)

CONF_POWER_SWITCH = "power_switch"
CONF_MUTE_SWITCH = "mute_switch"
CONF_AMP_SWITCH = "amp_switch"
CONF_GPIO2_SWITCH = "gpio2_switch"

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

AMP_SCHEMA = switch.switch_schema(
    Si4703AmpSwitch,
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend(
    {
        cv.Optional(CONF_NAME, default="Zosilňovač (GPIO1)"): cv.string,
        cv.Optional(CONF_ICON, default="mdi:amplifier"): cv.icon,
    }
).extend(cv.COMPONENT_SCHEMA)

GPIO2_SCHEMA = switch.switch_schema(
    Si4703Gpio2Switch,
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend(
    {
        cv.Optional(CONF_NAME, default="GPIO2 Prepínač"): cv.string,
        cv.Optional(CONF_ICON, default="mdi:toggle-switch"): cv.icon,
    }
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
        cv.Required(CONF_POWER_SWITCH): POWER_SCHEMA,
        cv.Optional(CONF_MUTE_SWITCH): MUTE_SCHEMA,
        cv.Optional(CONF_AMP_SWITCH): AMP_SCHEMA,
        cv.Optional(CONF_GPIO2_SWITCH): GPIO2_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA) # Predĺžime o základnú komponentnú schému


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SI4703_FM_ID])

    # Power Switch
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

    if CONF_AMP_SWITCH in config:
        conf = config[CONF_AMP_SWITCH]
        var = cg.new_Pvariable(conf[CONF_ID], hub)
        await switch.register_switch(var, conf)
        cg.add(hub.set_amp_switch(var))

    if CONF_GPIO2_SWITCH in config:
        conf = config[CONF_GPIO2_SWITCH]
        var = cg.new_Pvariable(conf[CONF_ID], hub)
        await switch.register_switch(var, conf)
        cg.add(hub.set_gpio2_switch(var))
