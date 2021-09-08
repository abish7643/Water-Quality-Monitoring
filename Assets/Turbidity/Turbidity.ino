int sensorPin = 36;    // select the input pin for the potentiometer
//int ledPin = 13;      // select the pin for the LED
//int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(sensorPin);
  Serial.print("ADC_Value: ");
  Serial.println(sensorValue);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = (sensorValue * 3.3 )/ 4095;
  // print out the value you read:
  Serial.print("Voltage: ");
  Serial.println(voltage);
  delay(1000);
}
