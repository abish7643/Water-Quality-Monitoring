#include <Arduino.h>

// Analog Sensors
typedef struct analog_sensor
{
    char name[24];
    short pin;
    float value;
    float offset;
    float min;
    float max;
    float threshold;
} analog_sensor;

// DS18B20 Temperature Sensor
typedef struct onewire_sensor
{
    char name[24];
    short pin;
    float valueC;
    float valueF;
    float min;
    float max;
    float threshold;
} onewire_sensor;

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

// Ultrasonic Sensor
typedef struct ultrasonic_sensor
{
  char name[24];
  short echoPin;
  short trigPin;
  float distance;
  long duration;
  float min;
  float max;
  float threshold;
} ultrasonic_sensor;