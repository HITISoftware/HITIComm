/*
 HITIComm examples:  EEPROM / 5_DynamicIOConfig

 This sketch shows how to save and load dynamic I/O config from the EEPROM
 (dynamic I/O config = I/O config performed using the Digital I/O control panel)

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>


// run once at power on
void setup()
{
	// initialize library
	HC_begin();
	
	// add I/O Config Space
	HC_eeprom.addIOConfigSpace();

	// if an I/O config is found in the EEPROM
	if (HC_eeprom.IOConfigDetected() == HIGH)
	{
		// load this I/O config
		HC_eeprom.loadIOConfig();
	}
}


// run repeatedly after setup()
void loop()
{
	// communicate with HITIPanel
	HC_communicate();
}
