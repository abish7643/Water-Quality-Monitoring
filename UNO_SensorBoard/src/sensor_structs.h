#include <Arduino.h>

// Analog Sensors
typedef struct analog_sensor
{
    char name[6];
    uint8_t pin;
    float value;
} analog_sensor;

// Ultrasonic Sensor
typedef struct ultrasonic_sensor
{
    char name[6];
    uint8_t echoPin;
    uint8_t trigPin;
    float distance;
} ultrasonic_sensor;

// DS18B20 Temperature Sensor
typedef struct onewire_sensor
{
    char name[6];
    uint8_t pin;
    float valueC;
    float valueF;
} onewire_sensor;

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
