#include <Arduino.h>
#include <main.h>

void IRAM_ATTR flowsensor_isr1();
void IRAM_ATTR flowsensor_isr2();
void IRAM_ATTR flowsensor_isr3();
void IRAM_ATTR flowsensor_isr4();
void IRAM_ATTR flowsensor_isr5();
void IRAM_ATTR flowsensor_isr6();

void setup()
{
#if SERIAL_MODE == SERIAL_DEBUG
  Serial.begin(115200);
#endif

// Flow Sensor
#if SERIAL_MODE == SERIAL_DEBUG
  Serial.println("Initalizing Flow Sensors: ");
#endif
  for (int i = 0; i < FLOW_SENSOR_TOTAL; i++)
  {
#if SERIAL_MODE == SERIAL_DEBUG
    Serial.print(flow_sensors[i]->name);
    Serial.print(", Pin:");
    Serial.print(flow_sensors[i]->pin);
    Serial.println();
#endif

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

#if SERIAL_MODE == SERIAL_DEBUG
  Serial.println();
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
#ifdef TDS_SENSOR
  for (int i = 0; i < TDS_SENSOR_TOTAL; i++)
  {
    // Iterate Through Each TDS Sensor with Serial Interface
    if (millis() - prev_ms_tds_measurement[i] >= TDS_SENSOR_MEASUREMENT_INTERVAL_MS)
    {

#if SERIAL_MODE == SERIAL_DEBUG
      Serial.print("TDS Sensor: ");
      Serial.println(TDSSensors[i]->name);
#endif

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

#if SERIAL_MODE == SERIAL_DEBUG
      for (int i = 0; i < TDS_SENSOR_RES_LENGTH; i++)
      {
        Serial.print(_tds_response_message[i], HEX);
        Serial.print(" ");
      }

      Serial.println();
#endif

      // Checking For Start Frame and End Frame
      if (_tds_response_message[0] == 0xAA && _tds_response_message[10] == 0x55)
      {
        uint8_t _computed_crc = 0x00;

        // Checking for Valid CheckSum
        // CheckSum Should Be The 9th Byte of Response Message
        for (int j = 0; j < TDS_SENSOR_RES_LENGTH - 2; j++)
        {
          _computed_crc += _tds_response_message[j];
          _computed_crc %= 256;
        }
        _computed_crc = (~_computed_crc) + 1;

        // Check if CRC Matches with Received CRC
        if (_computed_crc == _tds_response_message[9])
        {
          // #if SERIAL_MODE == SERIAL_DEBUG
          //           Serial.println("Valid CRC");
          // #endif
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

        digitalWrite(TDSSensors[i]->vcc_pin, LOW);
        TDSSensors[i]->sensor_on = false;
      }

      Serial.println();

      prev_ms_tds_measurement[i] = millis();
    }
  }
#endif

  if (millis() - prev_ms_flowsensor_measurement >= FLOW_SENSOR_MEASUREMENT_INTERVAL_MS)
  {
    for (int i = 0; i < 6; i++)
    {
#if SERIAL_MODE == SERIAL_DEBUG
      Serial.print(flow_sensors[i]->name);
      Serial.print(": ");
      Serial.print((flow_sensors[i]->val) * 0.0496);
      Serial.print("ml, ");
      Serial.print((flow_sensors[i]->total) * 0.0496 * 60);
      Serial.println("ml/min");
#endif
      flow_sensors[i]->val = 0;
    }

#if SERIAL_MODE == SERIAL_DEBUG
    Serial.println();
#endif

    prev_ms_flowsensor_measurement = millis();
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