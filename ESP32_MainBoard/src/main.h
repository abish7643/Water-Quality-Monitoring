#include <Arduino.h>
#include <sensor_structs.h>

// Modes of Main Serial
#define SERIAL_OFF 0
#define SERIAL_DEBUG 1
#define SERIAL_TDS 2

#define SERIAL_MODE SERIAL_DEBUG

// /////////////////////////////// TDS Sensor //////////////////////////////////

#define TDS_SENSOR                              // Enable TDS Sensor
#define TDS_SENSOR_MEASUREMENT_INTERVAL_MS 5000 // Measurement interval in ms
#define TDS_SENSOR_RES_TIMEOUT 200              // Response Timeout in ms
#define TDS_SENSOR_RES_LENGTH 11                // Response Timeout in ms
#define TDS_SENSOR_ON_TIME_MS 200               // Time Required to Turn ON After Supplying VCC

#if SERIAL_MODE == SERIAL_DEBUG
#define TDS_SENSOR_TOTAL 2 // Total TDS Sensors
#else
#define TDS_SENSOR_TOTAL 3 // Total TDS Sensors
#endif

#ifdef TDS_SENSOR
uint8_t tds_measurement_status = 0;

tds_sensor tds_sensor_1 = {"TD1", 27, 16, 17, 0, 0, 0, true};
tds_sensor tds_sensor_2 = {"TD2", 15, 13, 14, 0, 0, 0, true};

#define TDSSerial1 Serial1
#define TDSSerial2 Serial2

#if SERIAL_MODE == SERIAL_DEBUG
tds_sensor *TDSSensors[TDS_SENSOR_TOTAL] = {&tds_sensor_1, &tds_sensor_2};
HardwareSerial *TDSSerialPorts[TDS_SENSOR_TOTAL] = {&TDSSerial1, &TDSSerial2};

unsigned long prev_ms_tds_measurement[TDS_SENSOR_TOTAL] = {0, 0};
#else
tds_sensor tds_sensor_3 = {"TD3", 12, RX, TX, 0, 0, 0, true};
#define TDSSerial3 Serial

tds_sensor *TDSSensors[TDS_SENSOR_TOTAL] = {&tds_sensor_1, &tds_sensor_2, &tds_sensor_3};
HardwareSerial *TDSSerialPorts[TDS_SENSOR_TOTAL] = {&TDSSerial1, &TDSSerial2, &TDSSerial3};

unsigned long prev_ms_tds_measurement[TDS_SENSOR_TOTAL] = {0, 0, 0};
#endif

#endif

// /////////////////////////////// Flow Sensor /////////////////////////////////

#define FLOW_SENSOR_TOTAL 6
#define FLOW_SENSOR_MEASUREMENT_INTERVAL_MS 1000 // Measurement interval in ms

unsigned long prev_ms_flowsensor_measurement = 0;

PulseInput flowsensor_1 = {"Flow_1", 32, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_2 = {"Flow_2", 33, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_3 = {"Flow_3", 34, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_4 = {"Flow_4", 35, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_5 = {"Flow_5", 25, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_6 = {"Flow_6", 26, 0, 0, false, 0, 3.3, 1.8};

PulseInput *flow_sensors[FLOW_SENSOR_TOTAL] = {&flowsensor_1, &flowsensor_2, &flowsensor_3,
                                               &flowsensor_4, &flowsensor_5, &flowsensor_6};
