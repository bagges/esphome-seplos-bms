#include "seplos_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace seplos_bms {

static const char *const TAG = "seplos_bms";

static const uint8_t MAX_NO_RESPONSE_COUNT = 5;

// Alarm event messages (matching BLE implementation)
static const char *const ALARM_EVENT1_MESSAGES[8] = {
    "Voltage sensing failure",      // Bit 0
    "Temperature sensing failure",  // Bit 1
    "Current sensing failure",      // Bit 2
    "Key switch failure",           // Bit 3
    "Cell voltage diff failure",    // Bit 4
    "Charging switch failure",      // Bit 5
    "Discharge switch failure",     // Bit 6
    "Current limit switch failure"  // Bit 7
};

static const char *const ALARM_EVENT2_MESSAGES[8] = {
    "Single high voltage alarm",       // Bit 0
    "Single overvoltage protection",   // Bit 1
    "Single low voltage alarm",        // Bit 2
    "Single undervoltage protection",  // Bit 3
    "Total high voltage alarm",        // Bit 4
    "Total overvoltage protection",    // Bit 5
    "Total low voltage alarm",         // Bit 6
    "Total undervoltage protection"    // Bit 7
};

static const char *const ALARM_EVENT3_MESSAGES[8] = {
    "Charging high temp alarm",       // Bit 0
    "Charging overtemp protection",   // Bit 1
    "Charging low temp alarm",        // Bit 2
    "Charging undertemp protection",  // Bit 3
    "Discharge high temp alarm",      // Bit 4
    "Discharge overtemp protection",  // Bit 5
    "Discharge low temp alarm",       // Bit 6
    "Discharge undertemp protection"  // Bit 7
};

static const char *const ALARM_EVENT4_MESSAGES[8] = {
    "Ambient high temp alarm",       // Bit 0
    "Ambient overtemp protection",   // Bit 1
    "Ambient low temp alarm",        // Bit 2
    "Ambient undertemp protection",  // Bit 3
    "Power overtemp protection",     // Bit 4
    "Power high temp alarm",         // Bit 5
    "Battery low temp heating",      // Bit 6
    "Secondary trip protection"      // Bit 7
};

static const char *const ALARM_EVENT5_MESSAGES[8] = {
    "Charging overcurrent alarm",        // Bit 0
    "Charging overcurrent protection",   // Bit 1
    "Discharge overcurrent alarm",       // Bit 2
    "Discharge overcurrent protection",  // Bit 3
    "Transient overcurrent protection",  // Bit 4
    "Output short circuit protection",   // Bit 5
    "Transient overcurrent lockout",     // Bit 6
    "Output short circuit lockout"       // Bit 7
};

static const char *const ALARM_EVENT6_MESSAGES[8] = {
    "Charging high voltage protection",    // Bit 0
    "Intermittent power replenishment",    // Bit 1
    "Remaining capacity alarm",            // Bit 2
    "Remaining capacity protection",       // Bit 3
    "Low voltage charging prohibited",     // Bit 4
    "Output reverse polarity protection",  // Bit 5
    "Output connection failure",           // Bit 6
    "Internal alarm"                       // Bit 7
};

static const char *const ALARM_EVENT7_MESSAGES[8] = {
    "Internal alarm 1",            // Bit 0
    "Internal alarm 2",            // Bit 1
    "Internal alarm 3",            // Bit 2
    "Internal alarm 4",            // Bit 3
    "Automatic charging waiting",  // Bit 4
    "Manual charging waiting",     // Bit 5
    "Internal alarm 6",            // Bit 6
    "Internal alarm 7"             // Bit 7
};

static const char *const ALARM_EVENT8_MESSAGES[8] = {
    "EEP storage failure",              // Bit 0
    "RTC clock failure",                // Bit 1
    "Voltage calibration not done",     // Bit 2
    "Current calibration not done",     // Bit 3
    "Zero point calibration not done",  // Bit 4
    "Calendar not synchronized",        // Bit 5
    "Internal system error 6",          // Bit 6
    "Internal system error 7"           // Bit 7
};

// Function switch names for logging (64 total)
static const char *const FUNCTION_SWITCH_NAMES[64] = {
    // Switch 1 - Sensing failures
    "Voltage Sensing Failure",
    "Temperature Sensing Failure",
    "Current Sensing Failure",
    "Key Switch Failure",
    "Cell Voltage Diff Failure",
    "Charging Switch Failure",
    "Discharge Switch Failure",
    "Current Limit Switch Failure",
    // Switch 2 - Voltage alarms and protection
    "Single High Voltage Alarm",
    "Single Overvoltage Protection",
    "Single Low Voltage Alarm",
    "Single Undervoltage Protection",
    "Total High Voltage Alarm",
    "Total Overvoltage Protection",
    "Total Low Voltage Alarm",
    "Total Undervoltage Protection",
    // Switch 3 - Temperature alarms and protection
    "Charging High Temp Alarm",
    "Charging Overtemp Protection",
    "Charging Low Temp Alarm",
    "Charging Undertemp Protection",
    "Discharge High Temp Alarm",
    "Discharge Overtemp Protection",
    "Discharge Low Temp Alarm",
    "Discharge Undertemp Protection",
    // Switch 4 - Ambient and power temperature
    "Ambient High Temp Alarm",
    "Ambient Overtemp Protection",
    "Ambient Low Temp Alarm",
    "Ambient Undertemp Protection",
    "Power Overtemp Protection",
    "Power High Temp Alarm",
    "Battery Low Temp Heating",
    "Secondary Trip Protection",
    // Switch 5 - Current protection
    "Charging Overcurrent Alarm",
    "Charging Overcurrent Protection",
    "Discharge Overcurrent Alarm",
    "Discharge Overcurrent Protection",
    "Transient Overcurrent Protection",
    "Output Short Circuit Protection",
    "Transient Overcurrent Lockout",
    "Output Short Circuit Lockout",
    // Switch 6 - Capacity and output protection
    "Charging High Voltage Protection",
    "Intermittent Power Supply",
    "Remaining Capacity Alarm",
    "Remaining Capacity Protection",
    "Low Voltage Charging Prohibited",
    "Output Reverse Polarity Protection",
    "Output Connection Failure",
    "Output Soft Start",
    // Switch 7 - Balancing and charging
    "Charge Balancing",
    "Static Equalization",
    "Timeout Prohibits Equalization",
    "Overtemp Prohibits Equalization",
    "Automatic Charging Activation",
    "Manual Charging Activation",
    "Active Current Limiting Charging",
    "Passive Current Limiting Charging",
    // Switch 8 - System functions
    "Switch Shutdown",
    "Standby Poweroff",
    "History Function",
    "LCD Display",
    "Bluetooth Communication",
    "Automatic Address Encoding",
    "Parallel External Polling",
    "Reserved",
};

void SeplosBms::on_seplos_modbus_data(const std::vector<uint8_t> &data) {
  this->reset_online_status_tracker_();

  // num_of_cells   frame_size   data_len
  // 8              65           118 (0x76)   guessed
  // 14             77           142 (0x8E)
  // 15             79           146 (0x92)
  // 16             81           150 (0x96)
  if (data.size() >= 44 && data[8] >= 8 && data[8] <= 16) {
    this->on_telemetry_data_(data);
    return;
  }

  ESP_LOGW(TAG, "Unhandled data received (data_len: 0x%02X): %s", data[5],
           format_hex_pretty(&data.front(), data.size()).c_str());  // NOLINT
}

void SeplosBms::on_telemetry_data_(const std::vector<uint8_t> &data) {
  auto seplos_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0);
  };

  ESP_LOGI(TAG, "Telemetry frame (%d bytes) received", data.size());
  ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front(), data.size()).c_str());  // NOLINT

  // ->
  // 0x2000460010960001100CD70CE90CF40CD60CEF0CE50CE10CDC0CE90CF00CE80CEF0CEA0CDA0CDE0CD8060BA60BA00B970BA60BA50BA2FD5C14A0344E0A426803134650004603E8149F0000000000000000
  // 0x26004600307600011000000000000000000000000000000000000000000000000000000000000000000608530853085308530BAC0B9000000000002D0213880001E6B8
  //
  // *Data*
  //
  // Byte   Address Content: Description                      Decoded content               Coeff./Unit
  //   0    0x20             Protocol version      VER        2.0
  //   1    0x00             Device address        ADR
  //   2    0x46             Device type           CID1       Lithium iron phosphate battery BMS
  //   3    0x00             Function code         CID2       0x00: Normal, 0x01 VER error, 0x02 Chksum error, ...
  //   4    0x10             Data length checksum  LCHKSUM
  //   5    0x96             Data length           LENID      150 / 2 = 75
  //   6      0x00           Data flag
  //   7      0x01           Command group
  ESP_LOGV(TAG, "Command group: %d", data[7]);
  //   8      0x10           Number of cells                  16
  uint8_t cells = (this->override_cell_count_) ? this->override_cell_count_ : data[8];

  ESP_LOGV(TAG, "Number of cells: %d", cells);
  //   9      0x0C 0xD7      Cell voltage 1                   3287 * 0.001f = 3.287         V
  //   11     0x0C 0xE9      Cell voltage 2                   3305 * 0.001f = 3.305         V
  //   ...    ...            ...
  //   39     0x0C 0xD8      Cell voltage 16                                                V
  float min_cell_voltage = 100.0f;
  float max_cell_voltage = -100.0f;
  float average_cell_voltage = 0.0f;
  uint8_t min_voltage_cell = 0;
  uint8_t max_voltage_cell = 0;
  for (uint8_t i = 0; i < std::min((uint8_t) 16, cells); i++) {
    float cell_voltage = (float) seplos_get_16bit(9 + (i * 2)) * 0.001f;
    average_cell_voltage = average_cell_voltage + cell_voltage;
    if (cell_voltage < min_cell_voltage) {
      min_cell_voltage = cell_voltage;
      min_voltage_cell = i + 1;
    }
    if (cell_voltage > max_cell_voltage) {
      max_cell_voltage = cell_voltage;
      max_voltage_cell = i + 1;
    }
    this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
  }
  average_cell_voltage = average_cell_voltage / cells;

  this->publish_state_(this->min_cell_voltage_sensor_, min_cell_voltage);
  this->publish_state_(this->max_cell_voltage_sensor_, max_cell_voltage);
  this->publish_state_(this->max_voltage_cell_sensor_, (float) max_voltage_cell);
  this->publish_state_(this->min_voltage_cell_sensor_, (float) min_voltage_cell);
  this->publish_state_(this->delta_cell_voltage_sensor_, max_cell_voltage - min_cell_voltage);
  this->publish_state_(this->average_cell_voltage_sensor_, average_cell_voltage);

  uint8_t offset = 9 + (cells * 2);

  //   41     0x06           Number of temperatures           6                             V
  uint8_t temperature_sensors = data[offset];
  ESP_LOGV(TAG, "Number of temperature sensors: %d", temperature_sensors);

  //   42     0x0B 0xA6      Temperature sensor 1             (2982 - 2731) * 0.1f = 25.1          °C
  //   44     0x0B 0xA0      Temperature sensor 2             (2976 - 2731) * 0.1f = 24.5          °C
  //   46     0x0B 0x97      Temperature sensor 3             (2967 - 2731) * 0.1f = 23.6          °C
  //   48     0x0B 0xA6      Temperature sensor 4             (2982 - 2731) * 0.1f = 25.1          °C
  //   50     0x0B 0xA5      Environment temperature          (2981 - 2731) * 0.1f = 25.0          °C
  //   52     0x0B 0xA2      Mosfet temperature               (2978 - 2731) * 0.1f = 24.7          °C
  for (uint8_t i = 0; i < std::min((uint8_t) 6, temperature_sensors); i++) {
    float raw_temperature = (float) seplos_get_16bit(offset + 1 + (i * 2));
    this->publish_state_(this->temperatures_[i].temperature_sensor_, (raw_temperature - 2731.0f) * 0.1f);
  }
  offset = offset + 1 + (temperature_sensors * 2);

  //   54     0xFD 0x5C      Charge/discharge current         signed int?                   A
  float current = (float) ((int16_t) seplos_get_16bit(offset)) * 0.01f;
  this->publish_state_(this->current_sensor_, current);

  //   56     0x14 0xA0      Total battery voltage            5280 * 0.01f = 52.80          V
  float total_voltage = (float) seplos_get_16bit(offset + 2) * 0.01f;
  this->publish_state_(this->total_voltage_sensor_, total_voltage);

  float power = total_voltage * current;
  this->publish_state_(this->power_sensor_, power);
  this->publish_state_(this->charging_power_sensor_, std::max(0.0f, power));               // 500W vs 0W -> 500W
  this->publish_state_(this->discharging_power_sensor_, std::abs(std::min(0.0f, power)));  // -500W vs 0W -> 500W

  //   58     0x34 0x4E      Residual capacity                13390 * 0.01f = 133.90        Ah
  this->publish_state_(this->residual_capacity_sensor_, (float) seplos_get_16bit(offset + 4) * 0.01f);

  //   60     0x0A           Custom number                    10
  //   61     0x42 0x68      Battery capacity                 17000 * 0.01f = 170.00        Ah
  this->publish_state_(this->battery_capacity_sensor_, (float) seplos_get_16bit(offset + 7) * 0.01f);

  //   63     0x03 0x13      Stage of charge                  787 * 0.1f = 78.7             %
  this->publish_state_(this->state_of_charge_sensor_, (float) seplos_get_16bit(offset + 9) * 0.1f);

  //   65     0x46 0x50      Rated capacity                   18000 * 0.01f = 180.00        Ah
  this->publish_state_(this->rated_capacity_sensor_, (float) seplos_get_16bit(offset + 11) * 0.01f);

  if (data.size() < offset + 13 + 2) {
    return;
  }

  //   67     0x00 0x46      Number of cycles                 70
  this->publish_state_(this->charging_cycles_sensor_, (float) seplos_get_16bit(offset + 13));

  if (data.size() < offset + 15 + 2) {
    return;
  }

  //   69     0x03 0xE8      State of health                  1000 * 0.1f = 100.0           %
  this->publish_state_(this->state_of_health_sensor_, (float) seplos_get_16bit(offset + 15) * 0.1f);

  if (data.size() < offset + 17 + 2) {
    return;
  }

  //   71     0x14 0x9F      Port voltage                     5279 * 0.01f = 52.79          V
  this->publish_state_(this->port_voltage_sensor_, (float) seplos_get_16bit(offset + 17) * 0.01f);

  //   73     0x00 0x00      Reserved
  //   75     0x00 0x00      Reserved
  //   77     0x00 0x00      Reserved
  //   79     0x00 0x00      Reserved

  // Function switch data validation (offset + 27 to offset + 34 = 8 bytes)
  if (data.size() < offset + 35) {
    ESP_LOGD(TAG, "Frame too short for function switch data (size: %zu, required: %zu)", data.size(), offset + 35);
    return;
  }

  //   81     0x00           Function switch 1                Switch register 1
  //   82     0x00           Function switch 2                Switch register 2
  //   83     0x00           Function switch 3                Switch register 3
  //   84     0x00           Function switch 4                Switch register 4
  //   85     0x00           Function switch 5                Switch register 5
  //   86     0x00           Function switch 6                Switch register 6
  //   87     0x00           Function switch 7                Switch register 7
  //   88     0x00           Function switch 8                Switch register 8
  uint8_t switch1 = data[offset + 27];
  uint8_t switch2 = data[offset + 28];
  uint8_t switch3 = data[offset + 29];
  uint8_t switch4 = data[offset + 30];
  uint8_t switch5 = data[offset + 31];
  uint8_t switch6 = data[offset + 32];
  uint8_t switch7 = data[offset + 33];
  uint8_t switch8 = data[offset + 34];

  ESP_LOGD(TAG, "Function switches: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
           switch1, switch2, switch3, switch4, switch5, switch6, switch7, switch8);

  this->publish_function_switch_states_(switch1, switch2, switch3, switch4, switch5, switch6, switch7, switch8);

  // Alarm event data validation (offset + 35 to offset + 42 = 8 bytes)
  if (data.size() < offset + 43) {
    ESP_LOGD(TAG, "Frame too short for alarm event data (size: %zu, required: %zu)", data.size(), offset + 43);
    return;
  }

  //   89     0x00           Alarm event 1                    Hardware failures
  //   90     0x00           Alarm event 2                    Voltage alarms
  //   91     0x00           Alarm event 3                    Cell temperature
  //   92     0x00           Alarm event 4                    Environmental/power temperature
  //   93     0x00           Alarm event 5                    Current protection
  //   94     0x00           Alarm event 6                    Charging/output protection
  //   95     0x00           Alarm event 7                    Internal/charging waiting
  //   96     0x00           Alarm event 8                    System/calibration failures
  uint8_t alarm_event1 = data[offset + 35];
  uint8_t alarm_event2 = data[offset + 36];
  uint8_t alarm_event3 = data[offset + 37];
  uint8_t alarm_event4 = data[offset + 38];
  uint8_t alarm_event5 = data[offset + 39];
  uint8_t alarm_event6 = data[offset + 40];
  uint8_t alarm_event7 = data[offset + 41];
  uint8_t alarm_event8 = data[offset + 42];

  ESP_LOGD(TAG, "Alarm events: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
           alarm_event1, alarm_event2, alarm_event3, alarm_event4, alarm_event5, alarm_event6, alarm_event7, alarm_event8);

  // Publish alarm bitmask sensors
  this->publish_state_(this->alarm_event1_bitmask_sensor_, (float) alarm_event1);
  this->publish_state_(this->alarm_event2_bitmask_sensor_, (float) alarm_event2);
  this->publish_state_(this->alarm_event3_bitmask_sensor_, (float) alarm_event3);
  this->publish_state_(this->alarm_event4_bitmask_sensor_, (float) alarm_event4);
  this->publish_state_(this->alarm_event5_bitmask_sensor_, (float) alarm_event5);
  this->publish_state_(this->alarm_event6_bitmask_sensor_, (float) alarm_event6);
  this->publish_state_(this->alarm_event7_bitmask_sensor_, (float) alarm_event7);
  this->publish_state_(this->alarm_event8_bitmask_sensor_, (float) alarm_event8);

  // Decode and publish consolidated alarms text
  std::string consolidated_alarms = this->decode_all_alarm_events_(
      alarm_event1, alarm_event2, alarm_event3, alarm_event4, alarm_event5, alarm_event6, alarm_event7, alarm_event8);
  this->publish_state_(this->alarms_text_sensor_, consolidated_alarms);
}

void SeplosBms::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosBms:");
  LOG_SENSOR("", "Minimum Cell Voltage", this->min_cell_voltage_sensor_);
  LOG_SENSOR("", "Maximum Cell Voltage", this->max_cell_voltage_sensor_);
  LOG_SENSOR("", "Minimum Voltage Cell", this->min_voltage_cell_sensor_);
  LOG_SENSOR("", "Maximum Voltage Cell", this->max_voltage_cell_sensor_);
  LOG_SENSOR("", "Delta Cell Voltage", this->delta_cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 1", this->cells_[0].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 2", this->cells_[1].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 3", this->cells_[2].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 4", this->cells_[3].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 5", this->cells_[4].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 6", this->cells_[5].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 7", this->cells_[6].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 8", this->cells_[7].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 9", this->cells_[8].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 10", this->cells_[9].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 11", this->cells_[10].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 12", this->cells_[11].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 13", this->cells_[12].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 14", this->cells_[13].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 15", this->cells_[14].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 16", this->cells_[15].cell_voltage_sensor_);
  LOG_SENSOR("", "Temperature 1", this->temperatures_[0].temperature_sensor_);
  LOG_SENSOR("", "Temperature 2", this->temperatures_[1].temperature_sensor_);
  LOG_SENSOR("", "Temperature 3", this->temperatures_[2].temperature_sensor_);
  LOG_SENSOR("", "Temperature 4", this->temperatures_[3].temperature_sensor_);
  LOG_SENSOR("", "Temperature 5", this->temperatures_[4].temperature_sensor_);
  LOG_SENSOR("", "Temperature 6", this->temperatures_[5].temperature_sensor_);
  LOG_SENSOR("", "Total Voltage", this->total_voltage_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
  LOG_SENSOR("", "Charging Power", this->charging_power_sensor_);
  LOG_SENSOR("", "Discharging Power", this->discharging_power_sensor_);
  LOG_SENSOR("", "Charging cycles", this->charging_cycles_sensor_);
  LOG_SENSOR("", "State of charge", this->state_of_charge_sensor_);
  LOG_SENSOR("", "Residual capacity", this->residual_capacity_sensor_);
  LOG_SENSOR("", "Battery capacity", this->battery_capacity_sensor_);
  LOG_SENSOR("", "Rated capacity", this->rated_capacity_sensor_);
  LOG_SENSOR("", "Charging cycles", this->charging_cycles_sensor_);
  LOG_SENSOR("", "Average Cell Voltage", this->average_cell_voltage_sensor_);
  LOG_SENSOR("", "State of health", this->state_of_health_sensor_);
  LOG_SENSOR("", "Port Voltage", this->port_voltage_sensor_);
  
  // Alarm bitmask sensors
  LOG_SENSOR("", "Alarm Event 1 Bitmask", this->alarm_event1_bitmask_sensor_);
  LOG_SENSOR("", "Alarm Event 2 Bitmask", this->alarm_event2_bitmask_sensor_);
  LOG_SENSOR("", "Alarm Event 3 Bitmask", this->alarm_event3_bitmask_sensor_);
  LOG_SENSOR("", "Alarm Event 4 Bitmask", this->alarm_event4_bitmask_sensor_);
  LOG_SENSOR("", "Alarm Event 5 Bitmask", this->alarm_event5_bitmask_sensor_);
  LOG_SENSOR("", "Alarm Event 6 Bitmask", this->alarm_event6_bitmask_sensor_);
  LOG_SENSOR("", "Alarm Event 7 Bitmask", this->alarm_event7_bitmask_sensor_);
  LOG_SENSOR("", "Alarm Event 8 Bitmask", this->alarm_event8_bitmask_sensor_);
  
  // Function switch binary sensors
  LOG_BINARY_SENSOR("", "Voltage Sensing Failure", this->function_switches_[0].sensor_);
  LOG_BINARY_SENSOR("", "Temperature Sensing Failure", this->function_switches_[1].sensor_);
  LOG_BINARY_SENSOR("", "Current Sensing Failure", this->function_switches_[2].sensor_);
  LOG_BINARY_SENSOR("", "Key Switch Failure", this->function_switches_[3].sensor_);
  LOG_BINARY_SENSOR("", "Cell Voltage Diff Failure", this->function_switches_[4].sensor_);
  LOG_BINARY_SENSOR("", "Charging Switch Failure", this->function_switches_[5].sensor_);
  LOG_BINARY_SENSOR("", "Discharge Switch Failure", this->function_switches_[6].sensor_);
  LOG_BINARY_SENSOR("", "Current Limit Switch Failure", this->function_switches_[7].sensor_);
  LOG_BINARY_SENSOR("", "Single High Voltage Alarm", this->function_switches_[8].sensor_);
  LOG_BINARY_SENSOR("", "Single Overvoltage Protection", this->function_switches_[9].sensor_);
  LOG_BINARY_SENSOR("", "Single Low Voltage Alarm", this->function_switches_[10].sensor_);
  LOG_BINARY_SENSOR("", "Single Undervoltage Protection", this->function_switches_[11].sensor_);
  LOG_BINARY_SENSOR("", "Total High Voltage Alarm", this->function_switches_[12].sensor_);
  LOG_BINARY_SENSOR("", "Total Overvoltage Protection", this->function_switches_[13].sensor_);
  LOG_BINARY_SENSOR("", "Total Low Voltage Alarm", this->function_switches_[14].sensor_);
  LOG_BINARY_SENSOR("", "Total Undervoltage Protection", this->function_switches_[15].sensor_);
  LOG_BINARY_SENSOR("", "Charging High Temp Alarm", this->function_switches_[16].sensor_);
  LOG_BINARY_SENSOR("", "Charging Overtemp Protection", this->function_switches_[17].sensor_);
  LOG_BINARY_SENSOR("", "Charging Low Temp Alarm", this->function_switches_[18].sensor_);
  LOG_BINARY_SENSOR("", "Charging Undertemp Protection", this->function_switches_[19].sensor_);
  LOG_BINARY_SENSOR("", "Discharge High Temp Alarm", this->function_switches_[20].sensor_);
  LOG_BINARY_SENSOR("", "Discharge Overtemp Protection", this->function_switches_[21].sensor_);
  LOG_BINARY_SENSOR("", "Discharge Low Temp Alarm", this->function_switches_[22].sensor_);
  LOG_BINARY_SENSOR("", "Discharge Undertemp Protection", this->function_switches_[23].sensor_);
  LOG_BINARY_SENSOR("", "Ambient High Temp Alarm", this->function_switches_[24].sensor_);
  LOG_BINARY_SENSOR("", "Ambient Overtemp Protection", this->function_switches_[25].sensor_);
  LOG_BINARY_SENSOR("", "Ambient Low Temp Alarm", this->function_switches_[26].sensor_);
  LOG_BINARY_SENSOR("", "Ambient Undertemp Protection", this->function_switches_[27].sensor_);
  LOG_BINARY_SENSOR("", "Power Overtemp Protection", this->function_switches_[28].sensor_);
  LOG_BINARY_SENSOR("", "Power High Temp Alarm", this->function_switches_[29].sensor_);
  LOG_BINARY_SENSOR("", "Battery Low Temp Heating", this->function_switches_[30].sensor_);
  LOG_BINARY_SENSOR("", "Secondary Trip Protection", this->function_switches_[31].sensor_);
  LOG_BINARY_SENSOR("", "Charging Overcurrent Alarm", this->function_switches_[32].sensor_);
  LOG_BINARY_SENSOR("", "Charging Overcurrent Protection", this->function_switches_[33].sensor_);
  LOG_BINARY_SENSOR("", "Discharge Overcurrent Alarm", this->function_switches_[34].sensor_);
  LOG_BINARY_SENSOR("", "Discharge Overcurrent Protection", this->function_switches_[35].sensor_);
  LOG_BINARY_SENSOR("", "Transient Overcurrent Protection", this->function_switches_[36].sensor_);
  LOG_BINARY_SENSOR("", "Output Short Circuit Protection", this->function_switches_[37].sensor_);
  LOG_BINARY_SENSOR("", "Transient Overcurrent Lockout", this->function_switches_[38].sensor_);
  LOG_BINARY_SENSOR("", "Output Short Circuit Lockout", this->function_switches_[39].sensor_);
  LOG_BINARY_SENSOR("", "Charging High Voltage Protection", this->function_switches_[40].sensor_);
  LOG_BINARY_SENSOR("", "Intermittent Power Supply", this->function_switches_[41].sensor_);
  LOG_BINARY_SENSOR("", "Remaining Capacity Alarm", this->function_switches_[42].sensor_);
  LOG_BINARY_SENSOR("", "Remaining Capacity Protection", this->function_switches_[43].sensor_);
  LOG_BINARY_SENSOR("", "Low Voltage Charging Prohibited", this->function_switches_[44].sensor_);
  LOG_BINARY_SENSOR("", "Output Reverse Polarity Protection", this->function_switches_[45].sensor_);
  LOG_BINARY_SENSOR("", "Output Connection Failure", this->function_switches_[46].sensor_);
  LOG_BINARY_SENSOR("", "Output Soft Start", this->function_switches_[47].sensor_);
  LOG_BINARY_SENSOR("", "Charge Balancing", this->function_switches_[48].sensor_);
  LOG_BINARY_SENSOR("", "Static Equalization", this->function_switches_[49].sensor_);
  LOG_BINARY_SENSOR("", "Timeout Prohibits Equalization", this->function_switches_[50].sensor_);
  LOG_BINARY_SENSOR("", "Overtemp Prohibits Equalization", this->function_switches_[51].sensor_);
  LOG_BINARY_SENSOR("", "Automatic Charging Activation", this->function_switches_[52].sensor_);
  LOG_BINARY_SENSOR("", "Manual Charging Activation", this->function_switches_[53].sensor_);
  LOG_BINARY_SENSOR("", "Active Current Limiting Charging", this->function_switches_[54].sensor_);
  LOG_BINARY_SENSOR("", "Passive Current Limiting Charging", this->function_switches_[55].sensor_);
  LOG_BINARY_SENSOR("", "Switch Shutdown", this->function_switches_[56].sensor_);
  LOG_BINARY_SENSOR("", "Standby Poweroff", this->function_switches_[57].sensor_);
  LOG_BINARY_SENSOR("", "History Function", this->function_switches_[58].sensor_);
  LOG_BINARY_SENSOR("", "LCD Display", this->function_switches_[59].sensor_);
  LOG_BINARY_SENSOR("", "Bluetooth Communication", this->function_switches_[60].sensor_);
  LOG_BINARY_SENSOR("", "Automatic Address Encoding", this->function_switches_[61].sensor_);
  LOG_BINARY_SENSOR("", "Parallel External Polling", this->function_switches_[62].sensor_);
}

float SeplosBms::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void SeplosBms::update() {
  this->track_online_status_();
  this->send(0x42, this->pack_);
}

void SeplosBms::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr)
    return;

  binary_sensor->publish_state(state);
}

void SeplosBms::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;

  sensor->publish_state(value);
}

void SeplosBms::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;

  text_sensor->publish_state(state);
}

void SeplosBms::track_online_status_() {
  if (this->no_response_count_ < MAX_NO_RESPONSE_COUNT) {
    this->no_response_count_++;
  }
  if (this->no_response_count_ == MAX_NO_RESPONSE_COUNT) {
    this->publish_device_unavailable_();
    this->no_response_count_++;
  }
}

void SeplosBms::reset_online_status_tracker_() {
  this->no_response_count_ = 0;
  this->publish_state_(this->online_status_binary_sensor_, true);
}

void SeplosBms::publish_device_unavailable_() {
  this->publish_state_(this->online_status_binary_sensor_, false);
  this->publish_state_(this->errors_text_sensor_, "Offline");

  this->publish_state_(this->min_cell_voltage_sensor_, NAN);
  this->publish_state_(this->max_cell_voltage_sensor_, NAN);
  this->publish_state_(this->min_voltage_cell_sensor_, NAN);
  this->publish_state_(this->max_voltage_cell_sensor_, NAN);
  this->publish_state_(this->delta_cell_voltage_sensor_, NAN);
  this->publish_state_(this->average_cell_voltage_sensor_, NAN);
  this->publish_state_(this->total_voltage_sensor_, NAN);
  this->publish_state_(this->current_sensor_, NAN);
  this->publish_state_(this->power_sensor_, NAN);
  this->publish_state_(this->charging_power_sensor_, NAN);
  this->publish_state_(this->discharging_power_sensor_, NAN);
  this->publish_state_(this->state_of_charge_sensor_, NAN);
  this->publish_state_(this->residual_capacity_sensor_, NAN);
  this->publish_state_(this->battery_capacity_sensor_, NAN);
  this->publish_state_(this->rated_capacity_sensor_, NAN);
  this->publish_state_(this->charging_cycles_sensor_, NAN);
  this->publish_state_(this->state_of_health_sensor_, NAN);
  this->publish_state_(this->port_voltage_sensor_, NAN);

  for (auto &temperature : this->temperatures_) {
    this->publish_state_(temperature.temperature_sensor_, NAN);
  }

  for (auto &cell : this->cells_) {
    this->publish_state_(cell.cell_voltage_sensor_, NAN);
  }

  for (auto &function_switch : this->function_switches_) {
    this->publish_state_(function_switch.sensor_, false);
  }
}

void SeplosBms::publish_function_switch_states_(uint8_t sw1, uint8_t sw2, uint8_t sw3, uint8_t sw4, uint8_t sw5,
                                                uint8_t sw6, uint8_t sw7, uint8_t sw8) {
  uint8_t switches[8] = {sw1, sw2, sw3, sw4, sw5, sw6, sw7, sw8};

  for (uint8_t reg = 0; reg < 8; reg++) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      uint8_t index = reg * 8 + bit;
      bool state = (switches[reg] & (1 << bit)) != 0;
      
      // Check if state has changed and log it
      if (state != this->previous_function_switch_states_[index]) {
        ESP_LOGD(TAG, "Function switch changed: Register %d, Bit %d, %s = %s", 
                 reg + 1, bit, FUNCTION_SWITCH_NAMES[index], state ? "ON" : "OFF");
        this->previous_function_switch_states_[index] = state;
      }
      
      this->publish_state_(this->function_switches_[index].sensor_, state);
    }
  }
}

std::string SeplosBms::decode_all_alarm_events_(uint8_t alarm_event1, uint8_t alarm_event2, uint8_t alarm_event3,
                                                uint8_t alarm_event4, uint8_t alarm_event5, uint8_t alarm_event6,
                                                uint8_t alarm_event7, uint8_t alarm_event8) {
  std::string alarms_text = "";
  
  auto add_alarms = [&](const char *const messages[], uint8_t mask) {
    for (uint8_t i = 0; i < 8; i++) {
      if (mask & (1 << i)) {
        if (!alarms_text.empty()) {
          alarms_text += "; ";
        }
        alarms_text += messages[i];
      }
    }
  };

  add_alarms(ALARM_EVENT1_MESSAGES, alarm_event1);
  add_alarms(ALARM_EVENT2_MESSAGES, alarm_event2);
  add_alarms(ALARM_EVENT3_MESSAGES, alarm_event3);
  add_alarms(ALARM_EVENT4_MESSAGES, alarm_event4);
  add_alarms(ALARM_EVENT5_MESSAGES, alarm_event5);
  add_alarms(ALARM_EVENT6_MESSAGES, alarm_event6);
  add_alarms(ALARM_EVENT7_MESSAGES, alarm_event7);
  add_alarms(ALARM_EVENT8_MESSAGES, alarm_event8);

  if (alarms_text.empty()) {
    alarms_text = "No alarms";
  }

  return alarms_text;
}

}  // namespace seplos_bms
}  // namespace esphome
