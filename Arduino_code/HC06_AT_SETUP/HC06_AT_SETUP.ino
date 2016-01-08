/*
 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 11 (connect to TX of other device)
 * TX is digital pin 12 (connect to RX of other device)

 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 12); // RX, TX

void setup() 
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("This is a small terminal application to setup the HC06 BT module");
  Serial.println("Type AT, Response should be OK, that means you are connected");
  Serial.println("-->See Data sheet for more AT Commands");
  Serial.println("*********************************************************");
  Serial.println("Type AT commands:");

  mySerial.begin(57600);   // set the data rate for the SoftwareSerial port, set to the actual Baud rate of the HC06, default is 9600

}

void loop()
{ // run over and over
  if (mySerial.available())
  {
   Serial.write(mySerial.read());
  }
  if (Serial.available())
  {
    Serial.println("");
    mySerial.write(Serial.read()); 
  }

}

