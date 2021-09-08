#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Serial Ports
// #define SerialMon Serial  // Serial Monitor
#define SerialTDS Serial1 // TDS Sensors
#define RXD2 13           // ESP32 RX PIN
#define TXD2 12           // ESP32 TX PIN

#define LED_BUILTIN 2

// Timing
#define MEASUREMENT_INTERVAL 1000 // 1 Second
unsigned long previousMillis = 0;

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

analog_sensor pressure_sensor = {"Pressure_1", GPIO_NUM_32, 0, 0, 0, 1320, 500};
analog_sensor turbidity_sensor = {"Turbidity_1", GPIO_NUM_33, 0, 0, 0, 1320, 500};

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

onewire_sensor temperature_sensor = {"Temperature_1", GPIO_NUM_25, 0, 0, -10, 85, 16};

OneWire oneWire(temperature_sensor.pin);
DallasTemperature TempDS18B20(&oneWire);

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

PulseInput flowsensor_1 = {"Flow_1", 17, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_2 = {"Flow_2", 18, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_3 = {"Flow_3", 19, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_4 = {"Flow_4", 20, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_5 = {"Flow_5", 21, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_6 = {"Flow_6", 22, 0, 0, false, 0, 3.3, 1.8};

PulseInput *flow_sensors[] = {&flowsensor_1, &flowsensor_2, &flowsensor_3,
                              &flowsensor_4, &flowsensor_5, &flowsensor_6};

// Functions (Interrupt Service Routine)
void IRAM_ATTR flowsensor_isr1();
void IRAM_ATTR flowsensor_isr2();
void IRAM_ATTR flowsensor_isr3();
void IRAM_ATTR flowsensor_isr4();
void IRAM_ATTR flowsensor_isr5();
void IRAM_ATTR flowsensor_isr6();

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

ultrasonic_sensor ultrasonicsensor_1 = {"Ultrasonic_1", GPIO_NUM_26, GPIO_NUM_27, 0, 0, 0, 4.5, 1.6};

void setup()
{
  // Serial Communication
  Serial.begin(115200);                            // Serial Monitor Begin
  SerialTDS.begin(115200, SERIAL_8N1, RXD2, TXD2); // Serial Begin for TDS Sensor
  delay(10);

  // Inbuilt LED
  pinMode(LED_BUILTIN, OUTPUT);   // LED BUILT_IN
  digitalWrite(LED_BUILTIN, LOW); // DEFAULT -> OFF

  // DS18B20 Temperature Sensor
  TempDS18B20.begin();

  // Analog Sensors
  pinMode(pressure_sensor.pin, INPUT);
  pinMode(turbidity_sensor.pin, INPUT);

  // Flow Sensor
  pinMode(flowsensor_1.pin, INPUT_PULLUP);
  pinMode(flowsensor_2.pin, INPUT_PULLUP);
  pinMode(flowsensor_3.pin, INPUT_PULLUP);
  pinMode(flowsensor_4.pin, INPUT_PULLUP);
  pinMode(flowsensor_5.pin, INPUT_PULLUP);
  pinMode(flowsensor_6.pin, INPUT_PULLUP);

  // Add Interrupts
  attachInterrupt(flowsensor_1.pin, flowsensor_isr1, FALLING);
  attachInterrupt(flowsensor_2.pin, flowsensor_isr2, FALLING);
  attachInterrupt(flowsensor_3.pin, flowsensor_isr3, FALLING);
  attachInterrupt(flowsensor_4.pin, flowsensor_isr4, FALLING);
  attachInterrupt(flowsensor_5.pin, flowsensor_isr5, FALLING);
  attachInterrupt(flowsensor_6.pin, flowsensor_isr6, FALLING);

  // Ultrasonic Sensor
  pinMode(ultrasonicsensor_1.trigPin, OUTPUT);       // Sets the trigPin as an Output
  pinMode(ultrasonicsensor_1.echoPin, INPUT_PULLUP); // Sets the echoPin as an Input
}

void loop()
{
  unsigned long now = millis();

  // Get Measurement at Each Interval
  if (now - previousMillis >= MEASUREMENT_INTERVAL)
  {
    digitalWrite(LED_BUILTIN, HIGH); // LED ON

    // =============== GET TDS Values =================

    Serial.println("\nTDS Sensors");
    byte message[] = {0xAA, 0x05, 0x02, 0x4F, 0x55};
    SerialTDS.write(message, sizeof(message));
    for (int i = 0; i < 11; i++)
    {
      while (!SerialTDS.available())
        ; // wait for a character

      int incomingByte = SerialTDS.read();
      //  Serial.print(incomingByte, HEX);
      //  Serial.print(' ');
      if (i == 4)
      {
        Serial.print("TDS1: ");
        Serial.print(incomingByte, DEC);
      }
      if (i == 5)
      {
        Serial.println(incomingByte, DEC);
      }
      if (i == 6)
      {
        Serial.print("TDS2: ");
        Serial.print(incomingByte, DEC);
      }
      if (i == 7)
      {
        Serial.println(incomingByte, DEC);
      }
    }
    Serial.println();

    // ================================================

    // ============== Temperature Sensor ==============

    Serial.print("\nTemperature: ");
    TempDS18B20.requestTemperatures();
    temperature_sensor.valueC = TempDS18B20.getTempCByIndex(0);
    temperature_sensor.valueF = TempDS18B20.getTempFByIndex(0);
    Serial.print(temperature_sensor.valueC);
    Serial.print("ºC, ");
    Serial.print(temperature_sensor.valueF);
    Serial.println("ºF");

    // ================================================

    // ================= Flow Sensor ==================

    Serial.println("\nFlowsensors");
    for (int i = 0; i < 6; i++)
    {
      Serial.print(flow_sensors[i]->name);
      Serial.print(": ");
      Serial.print((flow_sensors[i]->val) * 0.0496);
      Serial.print("ml, ");
      Serial.print((flow_sensors[i]->total) * 0.0496 * 60);
      Serial.println("ml/min");
      flow_sensors[i]->val = 0;
    }

    // Serial.println("----FM1-----");
    // Serial.print(flowsensor_1.total * 0.0496);
    // Serial.println("ml");
    // Serial.print(flowsensor_1.val * 0.0496 * 60);
    // Serial.println("ml/m");
    // Serial.println("------------------");
    // flowsensor_1.val = 0;
    // Serial.println("----FM2-----");
    // Serial.print(flowsensor_2.total * 0.0496);
    // Serial.println("ml");
    // Serial.print(flowsensor_2.val * 0.0496 * 60);
    // Serial.println("ml/m");
    // Serial.println("------------------");
    // flowsensor_2.val = 0;
    // Serial.println("----FM3-----");
    // Serial.print(flowsensor_3.total * 0.0496);
    // Serial.println("ml");
    // Serial.print(flowsensor_3.val * 0.0496 * 60);
    // Serial.println("ml/m");
    // Serial.println("------------------");
    // flowsensor_3.val = 0;
    // Serial.println("----FM4-----");
    // Serial.print(flowsensor_4.total * 0.0496);
    // Serial.println("ml");
    // Serial.print(flowsensor_4.val * 0.0496 * 60);
    // Serial.println("ml/m");
    // Serial.println("------------------");
    // flowsensor_4.val = 0;
    // Serial.println("----FM5-----");
    // Serial.print(flowsensor_5.total * 0.0496);
    // Serial.println("ml");
    // Serial.print(flowsensor_5.val * 0.0496 * 60);
    // Serial.println("ml/m");
    // Serial.println("------------------");
    // flowsensor_5.val = 0;
    // Serial.println("----FM6-----");
    // Serial.print(flowsensor_6.total * 0.0496);
    // Serial.println("ml");
    // Serial.print(flowsensor_6.val * 0.0496 * 60);
    // Serial.println("ml/m");
    // Serial.println("------------------");
    // flowsensor_6.val = 0;

    // ================================================

    // =============== Pressure Sensor ================

    Serial.println("\nPressure Sensor");

    // 12-Bit ADC -> 4096 => Divide by 4095 and Scaling with a Factor 3.3V
    float voltage_pressure_sensor = analogRead(pressure_sensor.pin) * 3.30 / 4095;    //Sensor output voltage
    pressure_sensor.value = (voltage_pressure_sensor - pressure_sensor.offset) * 400; //Calculate water pressure

    Serial.print("Voltage:");
    Serial.print(voltage_pressure_sensor, 3);
    Serial.print("V");

    Serial.print(", Pressure:");
    Serial.print(pressure_sensor.value, 1);
    Serial.print(" KPa");
    Serial.println();

    // ================================================

    // ============== Turbidity Sensor ================

    Serial.println("\nTurbidity Sensor");

    // 12-Bit ADC -> 4096 => Divide by 4095 and Scaling with a Factor 3.3V
    float voltage_turbidity_sensor = analogRead(turbidity_sensor.pin) * 3.30 / 4095; // Sensor output voltage

    Serial.print("Voltage: ");
    Serial.print(voltage_turbidity_sensor);
    Serial.println("V");

    // ================================================

    // ============== Ultrasonic Sensor ===============

    Serial.println("\nUltrasonic Sensor");

    digitalWrite(ultrasonicsensor_1.trigPin, LOW);  // Clears the trigPin
    delayMicroseconds(5);                           // Wait for sensor to settle
    digitalWrite(ultrasonicsensor_1.trigPin, HIGH); // Sets the trigPin on HIGH state
    delayMicroseconds(10);                          // Send 10 micro second pulse
    digitalWrite(ultrasonicsensor_1.trigPin, LOW);  // Sets the trigPin on LOW state

    // Reads the echoPin, returns the sound wave travel time in microseconds
    ultrasonicsensor_1.duration = pulseIn(ultrasonicsensor_1.echoPin, HIGH);

    // Calculating the distance in CM
    ultrasonicsensor_1.distance = ultrasonicsensor_1.duration * 0.034 / 2;

    // Prints the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(ultrasonicsensor_1.distance);
    Serial.println("cm");

    // ===============================================

    previousMillis = millis();
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void IRAM_ATTR flowsensor_isr1()
{
  flowsensor_1.total += 1;
  flowsensor_1.val += 1;
  // flowsensor_1.pressed = true;
}

void IRAM_ATTR flowsensor_isr2()
{
  flowsensor_2.total += 1;
  flowsensor_2.val += 1;
  // flowsensor_2.pressed = true;
}

void IRAM_ATTR flowsensor_isr3()
{
  flowsensor_3.total += 1;
  flowsensor_3.val += 1;
  // flowsensor_3.pressed = true;
}

void IRAM_ATTR flowsensor_isr4()
{
  flowsensor_4.total += 1;
  flowsensor_4.val += 1;
  // flowsensor_4.pressed = true;
}

void IRAM_ATTR flowsensor_isr5()
{
  flowsensor_5.total += 1;
  flowsensor_5.val += 1;
  // flowsensor_5.pressed = true;
}

void IRAM_ATTR flowsensor_isr6()
{
  flowsensor_6.total += 1;
  flowsensor_6.val += 1;
  // flowsensor_6.pressed = true;
}