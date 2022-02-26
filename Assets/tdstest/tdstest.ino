#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 5); // RX, TX
bool x = 1;
int led = 13;
void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode (led, OUTPUT);
  delay(500);
}

void loop() // run over and over
{
    digitalWrite(13, 1); //LED for debug
    byte message[] = {0xAA, 0x05, 0x02, 0x4F, 0x55};
    mySerial.write(message, sizeof(message));
    for (int i = 0; i < 11; i++) { 
      while (!mySerial.available()); // wait for a character

      int incomingByte = mySerial.read();
    //  Serial.print(incomingByte, HEX);
    //  Serial.print(' ');
      if (i==4){
        Serial.print("TDS1: ");
        Serial.print(incomingByte,DEC);
      }
      if(i==5){
        Serial.println(incomingByte,DEC);
        }
       if (i==6){
        Serial.print("TDS2: ");
        Serial.print(incomingByte,DEC);
      }
      if(i==7){
        Serial.println(incomingByte,DEC);
        }
    }
    digitalWrite(13, 0);
    Serial.println();
    digitalWrite(led, LOW);
  delay(1000);
}
