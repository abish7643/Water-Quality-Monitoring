#include <Arduino.h>
#include <sensor_structs.h>

#define SERIAL_DISABLE 0 // Serial Should Be Disabled
#define SERIAL_DEBUG 1   // Serial Should Be Enabled
#define SERIAL_ESP 2     // Serial Should Be Used for ESP

#define SERIAL_MODE SERIAL_DEBUG // 0 -> DISABLE, 1 -> DEBUG, 2 -> ESP32 (Main Board)

// Serial Commands
#define SERIAL_CMD_GET_PRESSURE "PR"
#define SERIAL_CMD_GET_TURBIDITY "TR"
#define SERIAL_CMD_GET_DISTANCE "UL"
#define SERIAL_CMD_GET_TEMPERATURE "TE"
#define SERIAL_CMD_GET_TDS_1 "TD1"
#define SERIAL_CMD_GET_TDS_2 "TD2"
#define SERIAL_CMD_GET_TDS_3 "TD3"
#define SERIAL_CMD_GET_ALL "ALL"
#define SERIAL_CMD_MAX_LENGTH 6
#define SERIAL_CMD_DELIMITTER '\n'

#define SERIAL_RES_MAX_LENGTH 30
#define SERIAL_RES_START_CHAR '['
#define SERIAL_RES_END_CHAR ']'
#define SERIAL_RES_ERR_NAME "ER"

//////////////////////////////// Analog Sensors ////////////////////////////////

#define ANALOG_SENSORS                             // Enable Analog Sensors
#define ANALOG_SENSORS_MEASUREMENT_INTERVAL_MS 500 // Measurement interval in ms

#ifdef ANALOG_SENSORS
analog_sensor pressure_sensor = {SERIAL_CMD_GET_PRESSURE, A0, 0};
analog_sensor turbidity_sensor = {SERIAL_CMD_GET_TURBIDITY, A1, 0};
#endif

unsigned long prev_ms_analog_sensors_measurement = 0;

// ///////////////////////////// Ultrasonic Sensor /////////////////////////////

#define ULTRASONIC_SENSOR                              // Enable Ultrasonic Sensor
#define ULTRASONIC_SENSOR_MEASUREMENT_INTERVAL_MS 1000 // Measurement interval in ms

#ifdef ULTRASONIC_SENSOR
ultrasonic_sensor ultrasonicsensor_1 = {SERIAL_CMD_GET_DISTANCE, 11, 12, 0};
unsigned long prev_ms_ultrasonic_sensor_measurement = 0;
#endif

// //////////////////////////// Temperature Sensor /////////////////////////////

#define TEMPERATURE_SENSOR                              // Enable Temperature Sensor
#define TEMPERATURE_SENSOR_MEASUREMENT_INTERVAL_MS 1000 // Measurement interval in ms

#ifdef TEMPERATURE_SENSOR
onewire_sensor temperature_sensor = {SERIAL_CMD_GET_TEMPERATURE, 10, 0, 0};
unsigned long prev_ms_temperature_sensor_measurement = 0;
#endif

// /////////////////////////////// TDS Sensor //////////////////////////////////

// #define TDS_SENSOR                              // Enable TDS Sensor
#define TDS_SENSOR_MEASUREMENT_INTERVAL_MS 2000 // Measurement interval in ms
#define TDS_SENSOR_RES_TIMEOUT 200              // Response Timeout in ms
#define TDS_SENSOR_RES_LENGTH 11                // Response Timeout in ms
#define TDS_SENSOR_TOTAL 3                      // Total TDS Sensors
#define TDS_SENSOR_ON_TIME_MS 200               // Time Required to Turn ON After Supplying VCC

#ifdef TDS_SENSOR
uint8_t tds_measurement_status = 0;
unsigned long prev_ms_tds_measurement[TDS_SENSOR_TOTAL] = {0, 0, 0};

tds_sensor tds_sensor_1 = {SERIAL_CMD_GET_TDS_1, A5, 8, 9, 0, 0, 0, true};
tds_sensor tds_sensor_2 = {SERIAL_CMD_GET_TDS_2, A4, 4, 5, 0, 0, 0, true};
tds_sensor tds_sensor_3 = {SERIAL_CMD_GET_TDS_3, A3, 6, 7, 0, 0, 0, true};
#endif