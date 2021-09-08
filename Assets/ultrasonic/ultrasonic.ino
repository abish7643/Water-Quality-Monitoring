// defines pins numbers
const int trigPin = 2;
const int echoPin = 5;

// defines variables
long duration;
int distance;

void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT_PULLUP); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication
}

void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(5);

// Sets the trigPin on HIGH state for 10 micro secondsfile:///media/arun/Works/Works/Fiverr/WatermonitoringBangalore/Ultrasonictest/sketch_may26c/sketch_may26c.ino

digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);

// Calculating the distance in cm
distance= duration*0.034/2;

// Prints the distance on the Serial Monitor
Serial.print("Distance: ");
Serial.print(distance);
Serial.println("cm");
delay(250);
}
