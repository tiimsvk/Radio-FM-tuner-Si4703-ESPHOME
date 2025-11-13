import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, CONF_NAME, CONF_ICON

from . import Si4703FM, CONF_SI4703_FM_ID

CONF_RDS_TEXT = "rds_text"
CONF_RDS_PS = "rds_ps"
CONF_RDS_CT = "rds_ct"

# --- Schémy ---
RDS_PS_SCHEMA = text_sensor.text_sensor_schema(
    icon="mdi:radio",
)

RDS_TEXT_SCHEMA = text_sensor.text_sensor_schema(
    icon="mdi:home-assistant",
)

RDS_CT_SCHEMA = text_sensor.text_sensor_schema(
    icon="mdi:clock-check-outline",
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SI4703_FM_ID): cv.use_id(Si4703FM),
        cv.Optional(CONF_RDS_TEXT): RDS_TEXT_SCHEMA,
        cv.Optional(CONF_RDS_PS): RDS_PS_SCHEMA,
        cv.Optional(CONF_RDS_CT): RDS_CT_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    hub_var = await cg.get_variable(config[CONF_SI4703_FM_ID])

    if CONF_RDS_TEXT in config:
        text_conf = config[CONF_RDS_TEXT]
        text_sensor_var = await text_sensor.new_text_sensor(text_conf)
        cg.add(hub_var.set_rds_text_sensor(text_sensor_var))

    if CONF_RDS_PS in config:
        ps_conf = config[CONF_RDS_PS]
        ps_sensor_var = await text_sensor.new_text_sensor(ps_conf)
        cg.add(hub_var.set_rds_ps_sensor(ps_sensor_var))

    if CONF_RDS_CT in config:
        ct_conf = config[CONF_RDS_CT]
        ct_sensor_var = await text_sensor.new_text_sensor(ct_conf)
        cg.add(hub_var.set_rds_ct_sensor(ct_sensor_var))
