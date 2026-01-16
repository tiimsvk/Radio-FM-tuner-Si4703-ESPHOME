import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, number, sensor, switch, text_sensor, binary_sensor, button
from esphome.const import CONF_ID, CONF_ADDRESS, CONF_UPDATE_INTERVAL

DEPENDENCIES = ["i2c"]
# Povieme ESPHome, aby hľadalo number.py a sensor.py
AUTO_LOAD = ["number", "sensor", "switch", "text_sensor", "binary_sensor", "button"] 

si4703_fm_ns = cg.esphome_ns.namespace("si4703_fm")

# --- Definícia tried (importované ostatnými súbormi) ---
Si4703FM = si4703_fm_ns.class_("Si4703FM", cg.Component, i2c.I2CDevice)
Si4703Frequency = si4703_fm_ns.class_("Si4703Frequency", cg.Component, number.Number)
Si4703Volume = si4703_fm_ns.class_("Si4703Volume", cg.Component, number.Number)
Si4703PowerSwitch = si4703_fm_ns.class_("Si4703PowerSwitch", cg.Component, switch.Switch)
Si4703MuteSwitch = si4703_fm_ns.class_("Si4703MuteSwitch", cg.Component, switch.Switch)
Si4703AmpSwitch = si4703_fm_ns.class_("Si4703AmpSwitch", cg.Component, switch.Switch)
Si4703Gpio2Switch = si4703_fm_ns.class_("Si4703Gpio2Switch", cg.Component, switch.Switch)
Si4703StereoIndicator = si4703_fm_ns.class_("Si4703StereoIndicator", cg.Component, binary_sensor.BinarySensor)
Si4703TPIndicator = si4703_fm_ns.class_("Si4703TPIndicator", cg.Component, binary_sensor.BinarySensor)
Si4703TAIndicator = si4703_fm_ns.class_("Si4703TAIndicator", cg.Component, binary_sensor.BinarySensor)
Si4703Gpio2Sensor = si4703_fm_ns.class_("Si4703Gpio2Sensor", cg.Component, binary_sensor.BinarySensor)
Si4703SeekUpButton = si4703_fm_ns.class_("Si4703SeekUpButton", cg.Component, button.Button)
Si4703SeekDownButton = si4703_fm_ns.class_("Si4703SeekDownButton", cg.Component, button.Button)

# ID pre prepojenie
CONF_SI4703_FM_ID = "si4703_fm_id"
CONF_RESET_PIN = "reset_pin"
CONF_STC_INT_PIN = "stc_int_pin"

# Toto je schéma pre hlavný blok si4703_fm:
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Si4703FM),
        cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        # STC pin je teraz voliteľný, aby sa ušetrilo GPIO
        cv.Optional(CONF_STC_INT_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_UPDATE_INTERVAL, default="2s"): cv.positive_time_period_milliseconds,
    }
).extend(i2c.i2c_device_schema(0x10)).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # Tento kód inicializuje hlavný komponent Si4703FM
    parent = await cg.get_variable(config[i2c.CONF_I2C_ID])
    reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    
    stc_pin = None
    if CONF_STC_INT_PIN in config:
        stc_pin = await cg.gpio_pin_expression(config[CONF_STC_INT_PIN])

    var = cg.new_Pvariable(
        config[CONF_ID], 
        parent, 
        reset_pin,
        stc_pin,
    )
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # Nastavíme interval aktualizácie
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
