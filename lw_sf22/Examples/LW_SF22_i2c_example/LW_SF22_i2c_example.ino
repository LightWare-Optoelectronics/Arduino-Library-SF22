// www.Lightware.co.za          Oct 2019
// Lightware example code using the lw_sf22.h library
// This example enables the use of the I2C for communicaiton to the SF22
// and Serial Interface to the Serial Monitor on the Computer

// Refer to the Readme.txt file for the list of available function andvariables 

#include <lw_sf22.h>
#include <Wire.h>

LW_SF22 sf22(1, Serial);

void setup() {
  //this is the serial port for the terminal window
  Serial.begin(115200);

  //Switch I2C to Binary Mode
  //Have to send the three byte 0x78,0xAA,0xAA to unit at start to switch to Binary mode
  Wire.beginTransmission(sf22.I2CAddress);
  Wire.write(120);    // Set to Binary mode
  Wire.write(170);    // Set to Binary mode
  Wire.write(170);    // Set to Binary mode
  Wire.endTransmission();

  // read the Hardware model from the SF22. The 1 passed to the function
  // will enable the display of the result on the Serial Terminal.
  sf22.i2cReadHardwareModel(1);

  // Select default setting for distance output values
  sf22.i2cWriteSelectDistanceOutputValues(0); 
}

void loop() {

  // request the distance values in cm over the I2C
  // The values will be saved in the global variables
  sf22.i2cReadDistanceOutputValues(0);

  // print the distances, save in the global variables, on the Serial Monitor
  Serial.print(sf22.firstRaw_cm,DEC);
  Serial.print(" ");
  Serial.print(sf22.lastRaw_cm,DEC);
  Serial.print(" ");
  Serial.println(sf22.APDTemperature,DEC);

  delay(100);
  
  // put your main code here, to run repeatedly:
  
}
