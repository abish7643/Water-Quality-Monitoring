/************************************************************
   Water pressure sensor demo(Computer serial port)
   - Obtain the water pressure through the output voltage
     of the sensor.
**************************************************************/

/************************************************************
  Water Sensor Key Parameter
  - Parts No.:KY-3-5
  - Sensing range: 0 - 1.6 MPa
  - Input Voltage: 5VDC
  - Output Voltage: 0.5 - 4.5 VDC
    (Linearly corresponding to 0 - 1.6 MPa)
  - Accuary: 0.5% - 1% FS
**************************************************************/

/************************************************************
  Water Sensor Calibration

  The output voltage offset of the sensor is 0.5V (norminal).
  However, due to the zero-drifting of the internal circuit, the
  no-load output voltage is not exactly 0.5V. Calibration needs to
  be carried out as follow.

  Calibration: connect the 3 pin wire to the Arduio UNO (VCC, GND and Signal)
  without connecting the sensor to the water pipe and run the program
  for once. Mark down the LOWEST voltage value through the serial
  monitor and revise the "OffSet" value to complete the calibration.

  After the calibration the sensor is ready for measuring!
**************************************************************/

const float  OffSet = 0 ;

float V, P;

void setup()
{
  Serial.begin(9600);        // open serial port, set the baud rate to 9600 bps
  Serial.println("/** Water pressure sensor demo **/");
}
void loop()
{
  //Connect sensor to Analog 0
  V = analogRead(A0) * 5.00 / 1024;     //Sensor output voltage
  P = (V - OffSet) * 400;             //Calculate water pressure

  Serial.print("Voltage:");
  Serial.print(V, 3);
  Serial.println("V");

  Serial.print(" Pressure:");
  Serial.print(P, 1);
  Serial.println(" KPa");
  Serial.println();

  delay(500);
}
