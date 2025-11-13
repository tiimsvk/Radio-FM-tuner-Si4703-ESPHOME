# SÚBOR: si4703_fm/button.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID, CONF_ICON, ENTITY_CATEGORY_CONFIG

from . import (
    Si4703FM,
    CONF_SI4703_FM_ID,
    si4703_fm_ns,
    Si4703SeekUpButton,
    Si4703SeekDownButton
)

CONF_SEEK_UP = "seek_up"
CONF_SEEK_DOWN = "seek_down"

# --- Schéma pre Seek Up Tlaèidlo ---
SEEK_UP_SCHEMA = button.button_schema(
    Si4703SeekUpButton,
    icon="mdi:arrow-up-bold",
    entity_category=ENTITY_CATEGORY_CONFIG,
)

# --- Schéma pre Seek Down Tlaèidlo ---
SEEK_DOWN_SCHEMA = button.button_schema(
    Si4703SeekDownButton,
    icon="mdi:arrow-down-bold",
    entity_category=ENTITY_CATEGORY_CONFIG,
)

# --- Hlavná konfiguraèná schéma ---
CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
        cv.Optional(CONF_SEEK_UP): SEEK_UP_SCHEMA,
        cv.Optional(CONF_SEEK_DOWN): SEEK_DOWN_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SI4703_FM_ID])

    if CONF_SEEK_UP in config:
        conf = config[CONF_SEEK_UP]
        var = cg.new_Pvariable(conf[CONF_ID], hub)
        await button.register_button(var, conf)
        cg.add(hub.set_seek_up_button(var))

    if CONF_SEEK_DOWN in config:
        conf = config[CONF_SEEK_DOWN]
        var = cg.new_Pvariable(conf[CONF_ID], hub)
        await button.register_button(var, conf)
        cg.add(hub.set_seek_down_button(var))