import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, DEVICE_CLASS_CONNECTIVITY, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_SEPLOS_BMS_ID, SEPLOS_BMS_COMPONENT_SCHEMA

DEPENDENCIES = ["seplos_bms"]

CODEOWNERS = ["@syssi"]

CONF_ONLINE_STATUS = "online_status"

# Function switch configuration keys mapped to sensor indices (0-63)
CONF_FUNCTION_SWITCHES = {
    # Switch 1 - Sensing failures (indices 0-7)
    "voltage_sensing_failure": 0,
    "temperature_sensing_failure": 1,
    "current_sensing_failure": 2,
    "key_switch_failure": 3,
    "cell_voltage_diff_failure": 4,
    "charging_switch_failure": 5,
    "discharge_switch_failure": 6,
    "current_limit_switch_failure": 7,
    
    # Switch 2 - Voltage alarms and protection (indices 8-15)
    "single_high_voltage_alarm": 8,
    "single_overvoltage_protection": 9,
    "single_low_voltage_alarm": 10,
    "single_undervoltage_protection": 11,
    "total_high_voltage_alarm": 12,
    "total_overvoltage_protection": 13,
    "total_low_voltage_alarm": 14,
    "total_undervoltage_protection": 15,
    
    # Switch 3 - Temperature alarms and protection (indices 16-23)
    "charging_high_temp_alarm": 16,
    "charging_overtemp_protection": 17,
    "charging_low_temp_alarm": 18,
    "charging_undertemp_protection": 19,
    "discharge_high_temp_alarm": 20,
    "discharge_overtemp_protection": 21,
    "discharge_low_temp_alarm": 22,
    "discharge_undertemp_protection": 23,
    
    # Switch 4 - Ambient and power temperature (indices 24-31)
    "ambient_high_temp_alarm": 24,
    "ambient_overtemp_protection": 25,
    "ambient_low_temp_alarm": 26,
    "ambient_undertemp_protection": 27,
    "power_overtemp_protection": 28,
    "power_high_temp_alarm": 29,
    "battery_low_temp_heating": 30,
    "secondary_trip_protection": 31,
    
    # Switch 5 - Current protection (indices 32-39)
    "charging_overcurrent_alarm": 32,
    "charging_overcurrent_protection": 33,
    "discharge_overcurrent_alarm": 34,
    "discharge_overcurrent_protection": 35,
    "transient_overcurrent_protection": 36,
    "output_short_circuit_protection": 37,
    "transient_overcurrent_lockout": 38,
    "output_short_circuit_lockout": 39,
    
    # Switch 6 - Capacity and output protection (indices 40-47)
    "charging_high_voltage_protection": 40,
    "intermittent_power_supply": 41,
    "remaining_capacity_alarm": 42,
    "remaining_capacity_protection": 43,
    "low_voltage_charging_prohibited": 44,
    "output_reverse_polarity_protection": 45,
    "output_connection_failure": 46,
    "output_soft_start": 47,
    
    # Switch 7 - Balancing and charging (indices 48-55)
    "charge_balancing": 48,
    "static_equalization": 49,
    "timeout_prohibits_equalization": 50,
    "overtemp_prohibits_equalization": 51,
    "automatic_charging_activation": 52,
    "manual_charging_activation": 53,
    "active_current_limiting_charging": 54,
    "passive_current_limiting_charging": 55,
    
    # Switch 8 - System functions (indices 56-63)
    "switch_shutdown": 56,
    "standby_poweroff": 57,
    "history_function": 58,
    "lcd_display": 59,
    "bluetooth_communication": 60,
    "automatic_address_encoding": 61,
    "parallel_external_polling": 62,
    "reserved_switch8_bit7": 63,
}

BINARY_SENSORS = [
    CONF_ONLINE_STATUS,
]

CONFIG_SCHEMA = SEPLOS_BMS_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_ONLINE_STATUS): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        # Add optional binary sensors for all 64 function switches
        **{
            cv.Optional(key): binary_sensor.binary_sensor_schema()
            for key in CONF_FUNCTION_SWITCHES.keys()
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_BMS_ID])
    for key in BINARY_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            await binary_sensor.register_binary_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))
    
    # Register function switch binary sensors
    for key, index in CONF_FUNCTION_SWITCHES.items():
        if key in config:
            conf = config[key]
            sens = await binary_sensor.new_binary_sensor(conf)
            cg.add(hub.set_function_switch_sensor(index, sens))
