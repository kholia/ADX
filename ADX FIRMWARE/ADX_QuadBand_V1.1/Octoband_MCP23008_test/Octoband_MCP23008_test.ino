
// OCTOBAND MCP23008 I2C Comm Test - WB2CBA

#include <Wire.h> // Wire.h





void setup()
{
  Serial.begin(9600);
  Wire.begin(); // wake up I2C bus
  Wire.beginTransmission(0x20);
  Wire.write(0x00); // IODIRA register
  Wire.write(0x00); // set entire PORT A as output
  Wire.endTransmission();
}
 
void loop()
{
 
//WRITING TO PORT A1
Wire.beginTransmission(0x20);
Wire.write(0x09); // address port A
Wire.write(1);  // value to send
Wire.endTransmission();
delay(1000);

//WRITING TO PORT A2
Wire.beginTransmission(0x20);
Wire.write(0x09); // address port A
Wire.write(2);  // value to send
Wire.endTransmission();
delay(1000);

//WRITING TO PORT A3
Wire.beginTransmission(0x20);
Wire.write(0x09); // address port A
Wire.write(4);  // value to send
Wire.endTransmission();
delay(1000);

//WRITING TO PORT A4
Wire.beginTransmission(0x20);
Wire.write(0x09); // address port A
Wire.write(8);  // value to send
Wire.endTransmission();
delay(1000);

//WRITING TO PORT A5
Wire.beginTransmission(0x20);
Wire.write(0x09); // address port A
Wire.write(16);  // value to send
Wire.endTransmission();
delay(1000);

//WRITING TO PORT A6
Wire.beginTransmission(0x20);
Wire.write(0x09); // address port A
Wire.write(32);  // value to send
Wire.endTransmission();
delay(1000);

//WRITING TO PORT A7
Wire.beginTransmission(0x20);
Wire.write(0x09); // address port A
Wire.write(64);  // value to send
Wire.endTransmission();
delay(1000);

//WRITING TO PORT A8
Wire.beginTransmission(0x20);
Wire.write(0x09); // address port A
Wire.write(128);  // value to send
Wire.endTransmission();
delay(1000);

}
