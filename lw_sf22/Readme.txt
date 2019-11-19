www.lightware.co.za          Oct 2019
SF22 Serial and I2C interface library for Adruino
-------------------------------------------------


Installation
------------
Copy the whole lw_sf22 folder to your Arduino libraries 
folder (typically /Arduino/lib/targets/libraries/ )

To include this library in your sketch, go to the Sketch | Import Library menu and
select lw_sf22.  This will add a corresponding line to the top of your sketch:
#include <lw_sf22.h>

To stop using this library, delete that line from your sketch.



Available functions in this library
-----------------------------------
This library supports both the Serial port and the I2C interface.

The following functions are available for the serial port:
----------------------------------------------------------

sf22.readRequestHardwareName(void); 			// This will request the Hardware name
sf22.readRequestHardwareVersion(void); 			// This will request the product Hardware Version Number
sf22.readRequestFirmwareVersion(void); 			// This will request the product Firmware Version number
sf22.readRequestSerialNumber(void); 			// This will request the product Serial Number
sf22.readRequestToken(void); 					// This will request the next usable Token
sf22.readRequestDistOutConfig(void); 			// This will request the current Distance Output Configuration settings
sf22.readRequestDataStreamType(void); 			// This will request the current Data Stream Type setting
sf22.readRequestSerialBaudrate(void); 			// This will request the current Serial interface Baudrate
sf22.readRequestDistance(void); 				// This will request the current Distance Output as per the Distance output Configuration settings
sf22.readRequestI2CAddress(void); 				// This will request the current I2C Address
sf22.readRequestAPDTemp(void); 					// This will request the APD Temperature
sf22.readRequestSamplingRate(void); 			// This will request the Sampling Rate
sf22.writeSaveParameters(uint16_t Token); 		// This will send a save command.
												// A new token must be requested and recieved and then passed to this
												// function to send with the command
sf22.writeResetCommand(uint16_t Token); 		// This will send a Reset command.
												// A new token must be requested and recieved and then passed to this
												// function to send with the command
sf22.writeDistOutConfig(uint32_t Value); 		// This will change the Distance Output configuration
sf22.writeDataStreamType(uint32_t Type); 		// This will change the Data Stream output Type
sf22.writeSerialBaudrate(uint32_t Rate); 		// This will change the Serial Baudrate. Rate is the position in the table below
												// for example Rate = 4 is 115200 baudrate
												// serialBaudTable[9] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1440000 };
sf22.writeI2CAddress(uint32_t Address); 		// This will change the I2C interface Address
sf22.writeSamplingRate(uint16_t Rate); 			// This will change the Sampling Rate
												// Value of between 1 and 1000 samples/second	
uint8_t sf22.ProcessSerialInput(int16_t report);// Read data sent back from the SF22
												// it return 1 if new data was recieved, else 0


The following functions are available for the I2C interface:
------------------------------------------------------------

uint32_t sf22.i2cReadSelectDistanceOutputValues(int16_t report);// This will request the current Distance Output Configuration settings
sf22.i2cWriteSelectDistanceOutputValues(uint32_t Value); 		// This will change the Distance Output configuration
sf22.i2cReadDistanceOutputValues(int16_t report); 				// This function reads which distance outputs are selected and then read
																// them from the SF22.
																// They are then printed in sequence on the serial port.
sf22.i2cReadHardwareModel(int16_t report); 						// This function reads the Hardware model then save it in the global variable
sf22.i2cReadHardwareVersion(int16_t report); 					// This function reads the Hardware Version then save it in the global variable
sf22.i2cReadFirmwareVersion(int16_t report); 					// This function reads the Firmware Version then save it in the global variable
sf22.i2cReadSerialNumber(int16_t report); 						// This function reads the Serial Number then save it in the global variable
sf22.i2cReadToken(int16_t report); 										// This will request the next usable Token
sf22.i2cReadAPDTemp(int16_t report); 										// This will request the APD Temperature
sf22.i2cReadSamplingRate(int16_t report); 								// This will request the Sampling Rate
																// Value of between 1 and 1000 samples/second	
sf22.i2cWriteSamplingRate(uint16_t Rate); 						// This will change the Sampling Rate
																// Value of between 1 and 1000 samples/second	
sf22.i2cWriteSaveParameters(uint16_t Token); 					// This will send a save command.
																// A new token must be requested and recieved and then passed to this
																// function to send with the command
sf22.i2cWriteResetCommand(uint16_t Token); 						// This will send a Reset command.
																// A new token must be requested and recieved and then passed to this
																// function to send with the command
sf22.i2cWriteSerialBaudrate(uint32_t Rate); 					// This will change the Serial Baudrate. Rate is the position in the table below
																// for example Rate = 4 is 115200 baudrate
																// serialBaudTable[9] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1440000 };
sf22.i2cReadI2CAddress(int16_t report); 									// This will request the current I2C Address
sf22.i2cWriteI2CAddress(uint32_t Address); 						// This will change the I2C interface Address


The following global variables, updated by the functions above, is available:
-----------------------------------------------------------------------------

int16_t 	sf22.firstRaw_cm
int16_t 	sf22.firstFiltered_cm
int16_t 	sf22.firstStrength_cm
int16_t 	sf22.lastRaw_cm
int16_t 	sf22.lastFiltered_cm
int16_t 	sf22.lastStrength_cm
int32_t 	sf22.firstRaw_mm
int32_t 	sf22.firstFiltered_mm
int32_t 	sf22.firstStrength_mm
int32_t 	sf22.lastRaw_mm
int32_t 	sf22.lastFiltered_mm
int32_t 	sf22.lastStrength_mm
int32_t 	sf22.backgroundNoise
char 		sf22.HardwareModel[16]
uint32_t 	sf22.HardwareVersion
uint8_t 	sf22.FirmwareVersionReserved
uint8_t		sf22.FirmwareVersionMajor
uint8_t 	sf22.FirmwareVersionMinor
uint8_t 	sf22.FirmwareVersionPatch
char 		sf22.SerialNumber[16]
uint16_t 	sf22.Token
uint32_t 	sf22.DistanceOutConfig
uint32_t 	sf22.DataStreamType
uint8_t 	sf22.serialBaudrate
uint8_t 	sf22.I2CAddress
int16_t 	sf22.RawDataStream[210]
uint32_t 	sf22.StatBucketCount
uint32_t 	sf22.StatShotCount
uint16_t 	sf22.StatBucketsFront[130]
uint16_t 	sf22.StatBucketsRear[130]
int32_t 	sf22.APDTemperature
int16_t 	sf22.SamplingRate