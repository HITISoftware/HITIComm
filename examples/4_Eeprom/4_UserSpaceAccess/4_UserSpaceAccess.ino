/*
 HITIComm examples:  EEPROM / 4_UserSpaceAccess

 This sketch shows how to:
   => access the User Space from your code

 - Write (Virtual Switch)   on Digital Data 0
 - Read  (Virtual Switch)   on Digital Data 1

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

// Digital Data assignment (Buttons)
const byte DD_Write = 0; // Write value to EEPROM
const byte DD_Read  = 1; // Read value from EEPROM

void setup()
{
	// initialize library
	HC_begin();
}

void loop()
{
	// communicate with HITIPanel
	HC_communicate();

	// if the "WRITE" Button is clicked from HITIPanel
	if (HC_digitalDataRead_click(DD_Write) == HIGH)
		// Analog Data 0 value is written to Float 17 (EEPROM)
		HC_eeprom.writeFloat(17, HC_analogDataRead(0));

	// if the "READ" Button is clicked from HITIPanel
	else if (HC_digitalDataRead_click(DD_Read) == HIGH)
		// Float 17 (EEPROM) value is put in Analog Data 1
		HC_analogDataWrite(1, HC_eeprom.readFloat(17));
}
