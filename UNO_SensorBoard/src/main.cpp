#include <Arduino.h>
#include <main.h>

#if defined(TEMPERATURE_SENSOR)
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#if defined(TDS_SENSOR)
#include <SoftwareSerial.h>
#endif

#if defined(TEMPERATURE_SENSOR)
OneWire oneWire(temperature_sensor.pin);
DallasTemperature TempDS18B20(&oneWire);
#endif

#if defined(TDS_SENSOR)
SoftwareSerial TDSSerial1(tds_sensor_1.pin_rx, tds_sensor_1.pin_tx);
SoftwareSerial TDSSerial2(tds_sensor_2.pin_rx, tds_sensor_2.pin_tx);
SoftwareSerial TDSSerial3(tds_sensor_3.pin_rx, tds_sensor_3.pin_tx);

SoftwareSerial *TDSSerialPorts[TDS_SENSOR_TOTAL] = {&TDSSerial1, &TDSSerial2, &TDSSerial3};
tds_sensor *TDSSensors[TDS_SENSOR_TOTAL] = {&tds_sensor_1, &tds_sensor_2, &tds_sensor_3};
#endif

#if SERIAL_MODE == SERIAL_DEBUG || SERIAL_MODE == SERIAL_ESP
bool validate_serial_commmand = false;
char validate_serial_command_buffer[SERIAL_CMD_MAX_LENGTH];

char received_command[SERIAL_CMD_MAX_LENGTH];
unsigned int recevied_command_index = 0;
#endif

// ////////////////////////////////// Functions //////////////////////////////////
void get_distance();

void setup()
{
#if SERIAL_MODE == SERIAL_DEBUG || SERIAL_MODE == SERIAL_ESP
  Serial.begin(115200);
#endif

#ifdef ANALOG_SENSORS
  pinMode(pressure_sensor.pin, INPUT);
  pinMode(turbidity_sensor.pin, INPUT);
#endif

#ifdef ULTRASONIC_SENSOR
  pinMode(ultrasonicsensor_1.trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ultrasonicsensor_1.echoPin, INPUT);  // Sets the echoPin as an INPUT
#endif

#ifdef TEMPERATURE_SENSOR
  TempDS18B20.begin();
#endif

#ifdef TDS_SENSOR
  for (int i = 0; i < TDS_SENSOR_TOTAL; i++)
  {
    TDSSerialPorts[i]->begin(9600);
    pinMode(TDSSensors[i]->vcc_pin, OUTPUT);
    digitalWrite(TDSSensors[i]->vcc_pin, HIGH);
    TDSSensors[i]->sensor_on = true;
  }
#endif
}

void loop()
{
#if SERIAL_MODE == SERIAL_DEBUG || SERIAL_MODE == SERIAL_ESP

  while (Serial.available() > 0)
  {
    char inByte = Serial.read();

    if ((inByte != SERIAL_CMD_DELIMITTER) || (recevied_command_index > (SERIAL_CMD_MAX_LENGTH - 1)))
    {
      received_command[recevied_command_index] = inByte;
      recevied_command_index++;
    }
    else
    {

      if (inByte == SERIAL_CMD_DELIMITTER)
      {
        // Check if \r is present
        if (received_command[recevied_command_index - 1] == '\r')
        {
          received_command[recevied_command_index - 1] = '\0';
        }

        strcpy(validate_serial_command_buffer, received_command);
        validate_serial_commmand = true;
      }
      else
      {
        received_command[recevied_command_index] = '\0';
      }

      strcpy(received_command, "");
      recevied_command_index = 0;

      break;
    }
  }

  if (validate_serial_commmand)
  {
    validate_serial_commmand = false;

    // Send Back Data if Command is Valid
    // [NAME:VALUE]
    char _serial_response[SERIAL_RES_MAX_LENGTH];
    char _float_str[10];

    if (strcmp(validate_serial_command_buffer, SERIAL_CMD_GET_PRESSURE) == 0)
    {
#ifdef ANALOG_SENSORS
      dtostrf(pressure_sensor.value, 7, 3, _float_str);

      sprintf(_serial_response, "%c%s:%s%c",
              SERIAL_RES_START_CHAR, pressure_sensor.name, _float_str, SERIAL_RES_END_CHAR);
#endif
    }
    else if (strcmp(validate_serial_command_buffer, SERIAL_CMD_GET_TURBIDITY) == 0)
    {
#ifdef ANALOG_SENSORS
      dtostrf(turbidity_sensor.value, 7, 3, _float_str);

      sprintf(_serial_response, "%c%s:%s%c",
              SERIAL_RES_START_CHAR, turbidity_sensor.name, _float_str, SERIAL_RES_END_CHAR);
#endif
    }
    else if (strcmp(validate_serial_command_buffer, SERIAL_CMD_GET_DISTANCE) == 0)
    {
#ifdef ULTRASONIC_SENSOR
      dtostrf(ultrasonicsensor_1.distance, 7, 3, _float_str);

      sprintf(_serial_response, "%c%s:%s%c",
              SERIAL_RES_START_CHAR, ultrasonicsensor_1.name, _float_str, SERIAL_RES_END_CHAR);
#endif
    }
    else if (strcmp(validate_serial_command_buffer, SERIAL_CMD_GET_TEMPERATURE) == 0)
    {
#ifdef TEMPERATURE_SENSOR
      dtostrf(temperature_sensor.valueC, 7, 3, _float_str);

      sprintf(_serial_response, "%c%s:%s%c",
              SERIAL_RES_START_CHAR, temperature_sensor.name, _float_str, SERIAL_RES_END_CHAR);
#endif
    }
    else if (strcmp(validate_serial_command_buffer, SERIAL_CMD_GET_TDS_1) == 0 ||
             strcmp(validate_serial_command_buffer, SERIAL_CMD_GET_TDS_2) == 0 ||
             strcmp(validate_serial_command_buffer, SERIAL_CMD_GET_TDS_3) == 0)
    {
#ifdef TDS_SENSOR
      for (int i = 0; i < TDS_SENSOR_TOTAL; i++)
      {
        if (strcmp(validate_serial_command_buffer, TDSSensors[i]->name) == 0)
        {
          dtostrf(temperature_sensor.valueC, 6, 1, _float_str);

          sprintf(_serial_response, "%c%s:%d,%d,%s%c",
                  SERIAL_RES_START_CHAR, TDSSensors[i]->name,
                  TDSSensors[i]->value_1, TDSSensors[i]->value_2,
                  _float_str, SERIAL_RES_END_CHAR);
        }
      }
#endif
    }
    else
    {
#if SERIAL_MODE == SERIAL_DEBUG
      Serial.print("Invalid Command: ");
      Serial.println(validate_serial_command_buffer);
#endif
      sprintf(_serial_response, "%c%s:%d%c",
              SERIAL_RES_START_CHAR, SERIAL_RES_ERR_NAME, 1, SERIAL_RES_END_CHAR);
    }

    // Clear Command Buffer
    Serial.print(_serial_response);

#if SERIAL_MODE == SERIAL_DEBUG
    Serial.println();
#endif

    strcpy(validate_serial_command_buffer, "");
  }
#endif

#ifdef ANALOG_SENSORS
  if (millis() - prev_ms_analog_sensors_measurement >= ANALOG_SENSORS_MEASUREMENT_INTERVAL_MS)
  {
    pressure_sensor.value = analogRead(pressure_sensor.pin);
    turbidity_sensor.value = analogRead(turbidity_sensor.pin);

    prev_ms_analog_sensors_measurement = millis();

#if SERIAL_MODE == SERIAL_DEBUG
    Serial.print("[");
    Serial.print(pressure_sensor.name);
    Serial.print("] ");
    Serial.print(pressure_sensor.value);
    Serial.print(", [");
    Serial.print(turbidity_sensor.name);
    Serial.print("] ");
    Serial.println(turbidity_sensor.value);
#endif
  }
#endif

#ifdef ULTRASONIC_SENSOR
  if (millis() - prev_ms_ultrasonic_sensor_measurement >= ULTRASONIC_SENSOR_MEASUREMENT_INTERVAL_MS)
  {
    get_distance();

    prev_ms_ultrasonic_sensor_measurement = millis();

#if SERIAL_MODE == SERIAL_DEBUG
    Serial.print("[");
    Serial.print(ultrasonicsensor_1.name);
    Serial.print("] ");
    Serial.println(ultrasonicsensor_1.distance);
#endif
  }
#endif

#ifdef TEMPERATURE_SENSOR
  if (millis() - prev_ms_temperature_sensor_measurement >= TEMPERATURE_SENSOR_MEASUREMENT_INTERVAL_MS)
  {
    TempDS18B20.requestTemperatures();
    temperature_sensor.valueC = TempDS18B20.getTempCByIndex(0);
    temperature_sensor.valueF = TempDS18B20.getTempFByIndex(0);

    prev_ms_temperature_sensor_measurement = millis();

#if SERIAL_MODE == SERIAL_DEBUG
    Serial.print(temperature_sensor.valueC);
    Serial.print("ºC, ");
    Serial.print(temperature_sensor.valueF);
    Serial.println("ºF");
#endif
  }
#endif

#ifdef TDS_SENSOR
  for (int i = 0; i < TDS_SENSOR_TOTAL; i++)
  {
    TDSSerialPorts[i]->listen();

    // Iterate Through Each TDS Sensor with Serial Interface
    if (millis() - prev_ms_tds_measurement[i] >= TDS_SENSOR_MEASUREMENT_INTERVAL_MS)
    {
      // Turn on TDS Sensor if OFF
      if (TDSSensors[i]->sensor_on == false)
      {
        digitalWrite(TDSSensors[i]->vcc_pin, HIGH);
        TDSSensors[i]->sensor_on = true;
        delay(TDS_SENSOR_ON_TIME_MS);
      }

      byte tds_query_message[] = {0xAA, 0x05, 0x02, 0x4F, 0x55};
      TDSSerialPorts[i]->write(tds_query_message,
                               sizeof(tds_query_message) / sizeof(tds_query_message[0]));

      unsigned long _request_start_time = millis();

      int _tds_response_message[TDS_SENSOR_RES_LENGTH];
      uint8_t _index = 0;

      // Check Till Timeout is Reached
      while (millis() - _request_start_time <= TDS_SENSOR_RES_TIMEOUT)
      {
        bool _break_timeout = false;

        // Waits for Serial Port Data
        while (TDSSerialPorts[i]->available() > 0)
        {
          int _inByte = TDSSerialPorts[i]->read();
#if SERIAL_MODE == SERIAL_DEBUG
          Serial.print(_inByte, HEX);
          Serial.print(" ");
#endif

          // If Received Start Frame Byte -> Reset Index
          if (_inByte == 0xAA)
          {
            _index = 0;
          }

          _tds_response_message[_index] = _inByte;
          _index++;

          // Break if Buffer Overflow / End Frame Byte Received
          if (_index >= TDS_SENSOR_RES_LENGTH || _inByte == 0x55)
          {
            _break_timeout = true;
            break;
          }
        }

        // #if SERIAL_MODE == SERIAL_DEBUG
        //         Serial.println();
        // #endif
        if (_break_timeout)
        {
          break;
        }
      }

      // Check if Response is Valid
      // Length of Response Message is Max Length Specified
      if ((sizeof(_tds_response_message) / sizeof(_tds_response_message[0])) == TDS_SENSOR_RES_LENGTH)
      {
        // Checking For Start Frame and End Frame
        if (_tds_response_message[0] == 0xAA && _tds_response_message[10] == 0x55)
        {
          int _computed_crc = 0x00;

          // Checking for Valid CheckSum
          // CheckSum Should Be The 9th Byte of Response Message
          for (int j = 0; j < TDS_SENSOR_RES_LENGTH - 2; j++)
          {
            _computed_crc += _tds_response_message[j];
          }
          _computed_crc = (~_computed_crc) + 1;

          // Check if CRC Matches with Received CRC
          if (_computed_crc == _tds_response_message[9])
          {
#if SERIAL_MODE == SERIAL_DEBUG
            Serial.println("Valid CRC");
#endif
            // CRC Check Passed
            // Extracting TDS Value
            TDSSensors[i]->value_1 = _tds_response_message[5] + _tds_response_message[4];
            TDSSensors[i]->value_2 = _tds_response_message[7] + _tds_response_message[6];
            TDSSensors[i]->temperatureC = _tds_response_message[8];

#if SERIAL_MODE == SERIAL_DEBUG
            Serial.print(TDSSensors[i]->name);
            Serial.print(":");
            Serial.print(TDSSensors[i]->value_1);
            Serial.print(",");
            Serial.print(TDSSensors[i]->value_2);
            Serial.print(",");
            Serial.print(TDSSensors[i]->temperatureC);
            Serial.println();
#endif
          }
          else
          {
#if SERIAL_MODE == SERIAL_DEBUG
            Serial.print("Invalid CRC: ");
            Serial.print(_computed_crc);
            Serial.print("!=");
            Serial.print(_tds_response_message[9]);
            Serial.println();
#endif
          }
        }
        else
        {
#if SERIAL_MODE == SERIAL_DEBUG
          Serial.println("No Start/End Frame");
#endif
        }
      }
      else
      {
        if (sizeof(_tds_response_message) == 0)
        {
#if SERIAL_MODE == SERIAL_DEBUG
          Serial.println("No RES");
#endif
          // Turn off Sensor
          digitalWrite(TDSSensors[i]->vcc_pin, LOW);
          TDSSensors[i]->sensor_on = false;
        }
        else
        {
#if SERIAL_MODE == SERIAL_DEBUG
          Serial.println("Err RES");
#endif
        }
      }

      prev_ms_tds_measurement[i] = millis();
    }
  }
#endif
}

#ifdef ULTRASONIC_SENSOR
/*
  Compute Distance Using Ultrasonic Sensor
*/
void get_distance()
{
  // Clears the trigPin condition
  digitalWrite(ultrasonicsensor_1.trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(ultrasonicsensor_1.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicsensor_1.trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ultrasonicsensor_1.echoPin, HIGH);

  // Calculating the distance
  ultrasonicsensor_1.distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}
#endif