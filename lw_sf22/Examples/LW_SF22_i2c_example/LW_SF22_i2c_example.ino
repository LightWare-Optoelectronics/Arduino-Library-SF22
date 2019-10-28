// www.Lightware.co.za          Oct 2019
// Lightware example code using the lw_sf22.h library
// This example enables the use of the I2C for communicaiton to the SF22
// and Serial Interface to the Serial Monitor on the Computer

#include <lw_sf22.h>
#include <Wire.h>

LW_SF22 sf22(1, Serial);

void setup() {
  //this is the serial port for the terminal window
  Serial.begin(115200);
/*
  // Setup the Serial port for the SF22 interface
  Serial1.begin(115200);

  // Disable any possible streaming data
  sf22.writeDataStreamType(13);
  delay(100);
  // Read the data sent from the SF22
  sf22.ProcessSerialInput(1);
  delay(100);

  
  // Request the SF22 Hardware Name
  sf22.readRequestHardwareName();
  delay(100);
  // Read the data sent from the SF22
  sf22.ProcessSerialInput(1);
  delay(100);

  // Request the SF22 Firmware Version
  sf22.readRequestFirmwareVersion();
  delay(100);
  // Read the data sent from the SF22
  sf22.ProcessSerialInput(1);
  delay(100);

  // Request the streaming output distance data selection
  sf22.readRequestDistOutConfig();
  delay(100);
  // Read the data sent from the SF22
  sf22.ProcessSerialInput(1);
  delay(100);
  
  // Set the streaming to distance in cm
  sf22.writeDataStreamType(13);
  delay(100);
  // Read the data sent from the SF22
  sf22.ProcessSerialInput(1);
  delay(100);
*/

  sf22.i2cReadHardwareModel(1);

  // Select default values
  sf22.i2cWriteSelectDistanceOutputValues(0); 
}

void loop() {

  // request the distance values in cm over the I2C
  // The values will be saved in the global variables
  sf22.i2cReadDistanceOutputValues(0);

  // print the distances on the Serial Monitor
  Serial.print(sf22.firstRaw_cm,DEC);
  Serial.print(" ");
  Serial.print(sf22.lastRaw_cm,DEC);
  Serial.print(" ");
  Serial.println(sf22.APDTemperature,DEC);

  delay(100);
  
  // put your main code here, to run repeatedly:
  
}
