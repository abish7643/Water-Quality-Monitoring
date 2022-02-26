#include <Arduino.h>

// TDS Sensor
typedef struct tds_sensor
{
    char name[6];
    uint8_t vcc_pin;
    uint8_t pin_rx;
    uint8_t pin_tx;
    short value_1;
    short value_2;
    float temperatureC;
    bool sensor_on;
} tds_sensor;

// Flow Sensors
struct PulseInput
{
    char name[24];
    const uint8_t pin;
    float val;
    float total;
    bool pressed;
    float min;
    float max;
    float threshold;
};
