#include <Arduino.h>
#include <WiFi.h>        // WiFi control for ESP32
#include <ThingsBoard.h> // ThingsBoard SDK

// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

#define WIFI_AP_NAME "ELEMENTZ"                // WiFi SSID
#define WIFI_PASSWORD "4716006699"             // WiFi Password
#define TOKEN "VWJvD7Va5ALhHF"                 // Access Token
#define THINGSBOARD_SERVER "thingsboard.cloud" // ThingsBoard Server

#define SERIAL_DEBUG_BAUD 115200 // SerialBaud
#define TRANSMISSION_INTERVAL 2000

WiFiClient espClient;        // WiFi Client
ThingsBoard tb(espClient);   // ThingsBoard Client
int status = WL_IDLE_STATUS; // WiFi Status

unsigned long prev_transmission = 0; // Time passed after temperature/humidity data was sent, milliseconds.
bool subscribed = false;             // Set to true if application is subscribed for the RPC messages.

void InitWiFi();
void reconnect();

// // Processes function for RPC call "setValue"
// // RPC_Data is a JSON variant, that can be queried using operator[]
// // See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
// RPC_Response processDelayChange(const RPC_Data &data)
// {
//   Serial.println("Received the set delay RPC method");

//   // Process data

//   led_delay = data;

//   Serial.print("Set new delay: ");
//   Serial.println(led_delay);

//   return RPC_Response(NULL, led_delay);
// }

// // Processes function for RPC call "getValue"
// // RPC_Data is a JSON variant, that can be queried using operator[]
// // See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
// RPC_Response processGetDelay(const RPC_Data &data)
// {
//   Serial.println("Received the get value method");

//   return RPC_Response(NULL, led_delay);
// }

// // Processes function for RPC call "setGpioStatus"
// // RPC_Data is a JSON variant, that can be queried using operator[]
// // See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
// RPC_Response processSetGpioState(const RPC_Data &data)
// {
//   Serial.println("Received the set GPIO RPC method");

//   int pin = data["pin"];
//   bool enabled = data["enabled"];

//   if (pin < COUNT_OF(leds_control)) {
//     Serial.print("Setting LED ");
//     Serial.print(pin);
//     Serial.print(" to state ");
//     Serial.println(enabled);

//     digitalWrite(leds_control[pin], enabled);
//   }

//   return RPC_Response(data["pin"], (bool)data["enabled"]);
// }

// // RPC handlers
RPC_Callback callbacks[] = {
    // { "setValue",         processDelayChange },
    // { "getValue",         processGetDelay },
    // { "setGpioStatus",    processSetGpioState },
};

// Setup an application
void setup()
{
  // Initialize serial for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
}

// Main application loop
void loop()
{

  // Reconnect to WiFi, if needed
  if (WiFi.status() != WL_CONNECTED)
  {
    reconnect();
    return;
  }

  // Reconnect to ThingsBoard, if needed
  if (!tb.connected())
  {
    subscribed = false;

    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN))
    {
      Serial.println("Failed to connect");
      return;
    }
  }

  // Subscribe for RPC, if needed
  if (!subscribed)
  {
    Serial.println("Subscribing for RPC...");

    // Perform a subscription. All consequent data processing will happen in
    // callbacks as denoted by callbacks[] array.
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks)))
    {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;
  }

  // Check if it is a time to send DHT22 temperature and humidity
  if (millis() - prev_transmission > TRANSMISSION_INTERVAL)
  {
    // tb.sendTelemetryFloat("temperature", lastValues.temperature);
    Serial.println("Sending Sensor Data");
    tb.sendTelemetryFloat("humidity", 60);
    prev_transmission = millis();
  }

  // Process messages
  tb.loop();
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect()
{
  // Loop until we're reconnected
  status = WiFi.status();
  if (status != WL_CONNECTED)
  {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}