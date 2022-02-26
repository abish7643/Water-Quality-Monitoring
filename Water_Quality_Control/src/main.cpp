#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <main.h>

#include <WiFi.h>
#include <HTTPClient.h>

char wifi_ssid[32] = "ELEMENTZ";
char wifi_password[32] = "4716006699";

/*
Date: Sep 8, 2021
Example: Water Quality Sensor
*/

// Serial Ports
#define SerialMon Serial // Serial Monitor

#define SerialTDS1 Serial1 // TDS Sensors 1
#define RXD1 16            // ESP32 RX PIN
#define TXD1 17            // ESP32 TX PIN

// #define SerialTDS2 Serial2 // TDS Sensors 2
#define RXD2 13 // ESP32 RX PIN
#define TXD2 14 // ESP32 TX PIN

#define LED_BUILTIN 2

// Timing
#define MEASUREMENT_INTERVAL 1000 // 1 Second
unsigned long previousMillis = 0;

analog_sensor pressure_sensor = {"Pressure_1", GPIO_NUM_36, 0, 0, 0, 1320, 500};
analog_sensor turbidity_sensor = {"Turbidity_1", GPIO_NUM_39, 0, 0, 0, 1320, 500};

onewire_sensor temperature_sensor = {"Temperature_1", GPIO_NUM_27, 0, 0, -10, 85, 16};
OneWire oneWire(temperature_sensor.pin);
DallasTemperature TempDS18B20(&oneWire);

// ultrasonic_sensor ultrasonicsensor_1 = {"Ultrasonic_1", GPIO_NUM_26, GPIO_NUM_27, 0, 0, 0, 4.5, 1.6};

#define TOTAL_FLOWSENSORS 6

PulseInput flowsensor_1 = {"Flow_1", 32, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_2 = {"Flow_2", 33, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_3 = {"Flow_3", 34, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_4 = {"Flow_4", 35, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_5 = {"Flow_5", 25, 0, 0, false, 0, 3.3, 1.8};
PulseInput flowsensor_6 = {"Flow_6", 26, 0, 0, false, 0, 3.3, 1.8};

PulseInput *flow_sensors[] = {&flowsensor_1, &flowsensor_2, &flowsensor_3,
                              &flowsensor_4, &flowsensor_5, &flowsensor_6};

// Functions (Interrupt Service Routine)
void IRAM_ATTR flowsensor_isr1();
void IRAM_ATTR flowsensor_isr2();
void IRAM_ATTR flowsensor_isr3();
void IRAM_ATTR flowsensor_isr4();
void IRAM_ATTR flowsensor_isr5();
void IRAM_ATTR flowsensor_isr6();

void setup()
{
  // Serial Communication
  SerialMon.begin(115200);                        // Serial Monitor Begin
  SerialTDS1.begin(9600, SERIAL_8N1, RXD1, TXD1); // Serial Begin for TDS Sensor
  delay(10);

  // Inbuilt LED
  pinMode(LED_BUILTIN, OUTPUT);   // LED BUILT_IN
  digitalWrite(LED_BUILTIN, LOW); // DEFAULT -> OFF

  WiFi.begin(wifi_ssid, wifi_password);
  SerialMon.println("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    SerialMon.print(".");
  }
  SerialMon.println("");
  SerialMon.print("Connected to WiFi network with IP Address: ");
  SerialMon.println(WiFi.localIP());

  // // DS18B20 Temperature Sensor
  // TempDS18B20.begin();

  // Analog Sensors
  pinMode(pressure_sensor.pin, INPUT);
  pinMode(turbidity_sensor.pin, INPUT);

  // TDS Reporting Enabled By Default
  // byte tds_reporting_en_message[] = {0xAA, 0x05, 0x01, 0x51, 0x55};
  // SerialTDS1.write(tds_reporting_en_message, sizeof(tds_reporting_en_message));

  // Flow Sensor
  SerialMon.println("Initalizing Flow Sensors: ");
  for (int i = 0; i < TOTAL_FLOWSENSORS; i++)
  {
    SerialMon.print(flow_sensors[i]->name);
    SerialMon.print(", Pin:");
    SerialMon.print(flow_sensors[i]->pin);
    SerialMon.println();

    pinMode(flow_sensors[i]->pin, INPUT_PULLUP);

    if (i == 0)
      attachInterrupt(flow_sensors[i]->pin, flowsensor_isr1, FALLING);
    else if (i == 1)
      attachInterrupt(flow_sensors[i]->pin, flowsensor_isr2, FALLING);
    else if (i == 2)
      attachInterrupt(flow_sensors[i]->pin, flowsensor_isr3, FALLING);
    else if (i == 3)
      attachInterrupt(flow_sensors[i]->pin, flowsensor_isr4, FALLING);
    else if (i == 4)
      attachInterrupt(flow_sensors[i]->pin, flowsensor_isr5, FALLING);
    else if (i == 5)
      attachInterrupt(flow_sensors[i]->pin, flowsensor_isr6, FALLING);
  }
  Serial.println();

  // Ultrasonic Sensor
  // pinMode(ultrasonicsensor_1.trigPin, OUTPUT);       // Sets the trigPin as an Output
  // pinMode(ultrasonicsensor_1.echoPin, INPUT_PULLUP); // Sets the echoPin as an Input
}

void loop()
{
  unsigned long now = millis();

  // Get Measurement at Each Interval
  if (now - previousMillis >= MEASUREMENT_INTERVAL)
  {
    digitalWrite(LED_BUILTIN, HIGH); // LED ON

    // =============== GET TDS Values =================

    SerialMon.println("\nTDS Sensors");
    byte tds_query_message[] = {0xAA, 0x05, 0x02, 0x4F, 0x55};
    // byte tds_query_message[] = {0xAA, 0x05, 0x01, 0x50, 0x55};
    // byte tds_query_message[] = {0xAA, 0x05, 0x00, 0x51, 0x55};
    SerialTDS1.write(tds_query_message, sizeof(tds_query_message));

    // unsigned long tds_poll_millis = millis();

    while (SerialTDS1.available())
    {
      int incomingByte = SerialTDS1.read();
      SerialMon.print(incomingByte, HEX);
      SerialMon.print(" ");
    }

    // for (int i = 0; i < 11; i++)
    // {
    //   bool _break_polling = false;
    //   while (!SerialTDS1.available())
    //   {
    //     if (millis() - tds_poll_millis >= 100)
    //     {
    //       SerialMon.println("TDS Poll Timeout");
    //       _break_polling = true;
    //       break;
    //     }
    //   }

    //   if (_break_polling)
    //   {
    //     tds_query_message[2] = 0x00;
    //     tds_query_message[3] = 0x51;

    //     SerialMon.print("Disabling Reporting");
    //     SerialTDS1.flush();

    //     SerialTDS1.write(tds_query_message, sizeof(tds_query_message));
    //     break;
    //   }

    //   int incomingByte = SerialTDS1.read();
    //   SerialMon.print(incomingByte, HEX);
    //   SerialMon.print(' ');
    //   // if (i == 4)
    //   // {
    //   //   SerialMon.print("TDS1: ");
    //   //   SerialMon.print(incomingByte, DEC);
    //   // }
    //   // if (i == 5)
    //   // {
    //   //   SerialMon.println(incomingByte, DEC);
    //   // }
    //   // if (i == 6)
    //   // {
    //   //   SerialMon.print("TDS2: ");
    //   //   SerialMon.print(incomingByte, DEC);
    //   // }
    //   // if (i == 7)
    //   // {
    //   //   SerialMon.print(incomingByte, DEC);
    //   // }
    // }
    SerialMon.println();

    // ================================================

    // ============== Temperature Sensor ==============

    // SerialMon.print("\nTemperature: ");
    // TempDS18B20.requestTemperatures();
    // temperature_sensor.valueC = TempDS18B20.getTempCByIndex(0);
    // temperature_sensor.valueF = TempDS18B20.getTempFByIndex(0);
    // SerialMon.print(temperature_sensor.valueC);
    // SerialMon.print("ºC, ");
    // SerialMon.print(temperature_sensor.valueF);
    // SerialMon.println("ºF");

    // ================================================

    // ================= Flow Sensor ==================

    SerialMon.println("\nFlowsensors");
    for (int i = 0; i < 6; i++)
    {
      SerialMon.print(flow_sensors[i]->name);
      SerialMon.print(": ");
      SerialMon.print((flow_sensors[i]->val) * 0.0496);
      SerialMon.print("ml, ");
      SerialMon.print((flow_sensors[i]->total) * 0.0496 * 60);
      SerialMon.println("ml/min");
      flow_sensors[i]->val = 0;
    }

    // SerialMon.println("----FM1-----");
    // SerialMon.print(flowsensor_1.total * 0.0496);
    // SerialMon.println("ml");
    // SerialMon.print(flowsensor_1.val * 0.0496 * 60);
    // SerialMon.println("ml/m");
    // SerialMon.println("------------------");
    // flowsensor_1.val = 0;

    // ================================================

    // =============== Pressure Sensor ================

    SerialMon.println("\nPressure Sensor");

    // 12-Bit ADC -> 4096 => Divide by 4095 and Scaling with a Factor 3.3V
    float voltage_pressure_sensor = analogRead(pressure_sensor.pin) * 3.30 / 4095;    // Sensor output voltage
    pressure_sensor.value = (voltage_pressure_sensor - pressure_sensor.offset) * 400; // Calculate water pressure

    SerialMon.print("Voltage:");
    SerialMon.print(voltage_pressure_sensor, 3);
    SerialMon.print("V");

    SerialMon.print(", Pressure:");
    SerialMon.print(pressure_sensor.value, 1);
    SerialMon.print(" KPa");
    SerialMon.println();

    // ================================================

    // ============== Turbidity Sensor ================

    SerialMon.println("\nTurbidity Sensor");

    // 12-Bit ADC -> 4096 => Divide by 4095 and Scaling with a Factor 3.3V
    float voltage_turbidity_sensor = analogRead(turbidity_sensor.pin) * 3.30 / 4095; // Sensor output voltage

    SerialMon.print("Voltage: ");
    SerialMon.print(voltage_turbidity_sensor);
    SerialMon.println("V");

    // ================================================

    // ============== Ultrasonic Sensor ===============

    // SerialMon.println("\nUltrasonic Sensor");

    // digitalWrite(ultrasonicsensor_1.trigPin, LOW);  // Clears the trigPin
    // delayMicroseconds(5);                           // Wait for sensor to settle
    // digitalWrite(ultrasonicsensor_1.trigPin, HIGH); // Sets the trigPin on HIGH state
    // delayMicroseconds(10);                          // Send 10 micro second pulse
    // digitalWrite(ultrasonicsensor_1.trigPin, LOW);  // Sets the trigPin on LOW state

    // // Reads the echoPin, returns the sound wave travel time in microseconds
    // ultrasonicsensor_1.duration = pulseIn(ultrasonicsensor_1.echoPin, HIGH);

    // // Calculating the distance in CM
    // ultrasonicsensor_1.distance = ultrasonicsensor_1.duration * 0.034 / 2;

    // // Prints the distance on the Serial Monitor
    // SerialMon.print("Distance: ");
    // SerialMon.print(ultrasonicsensor_1.distance);
    // SerialMon.println("cm");

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