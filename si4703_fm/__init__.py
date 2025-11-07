import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_ADDRESS

DEPENDENCIES = ["i2c"]
# Povieme ESPHome, že tento komponent poskytuje aj platformu 'number'
AUTO_LOAD = ["number", "sensor"]

si4703_fm_ns = cg.esphome_ns.namespace("si4703_fm")

# --- Definícia tried ---
# Definujeme len hlavnú triedu
Si4703FM = si4703_fm_ns.class_("Si4703FM", cg.Component, i2c.I2CDevice)

# ID pre prepojenie
CONF_SI4703_FM_ID = "si4703_fm_id"
CONF_RESET_PIN = "reset_pin"
CONF_STC_INT_PIN = "stc_int_pin"

# Toto je schéma pre hlavný blok si4703_fm:
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Si4703FM),
        cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_STC_INT_PIN): pins.gpio_input_pin_schema,
    }
).extend(i2c.i2c_device_schema(0x10)).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # Tento kód inicializuje hlavný komponent Si4703FM
    parent = await cg.get_variable(config[i2c.CONF_I2C_ID])
    
    reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    stc_pin = await cg.gpio_pin_expression(config[CONF_STC_INT_PIN])

    var = cg.new_Pvariable(
        config[CONF_ID], 
        parent, 
        reset_pin,
        stc_pin
    )
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)