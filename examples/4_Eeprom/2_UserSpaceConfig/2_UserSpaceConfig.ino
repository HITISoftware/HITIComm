/*
 HITIComm examples:  Eeprom / 2_UserSpaceConfig

 This sketch shows how to:
   => configure the User Space

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

void setup()
{
	// initialize library
	HC_begin();

	// configure User Space (sub-spaces sizes)
	HC_eeprom.setUserSpace(
		0,     // Boolean space : default 50
		200,   // Byte space    : default 100
		0,     // Integer space : default 200
		400,   // Long space    : default 250
		0,     // Float space   : default 250
		0);    // String space  : default 4096

	// display the number of elements contained in the sub-spaces
	HC_analogDataWrite(0, HC_eeprom.getUserSpace_DataQty(HC_USERSPACE_BYTE));
	HC_analogDataWrite(1, HC_eeprom.getUserSpace_DataQty(HC_USERSPACE_LONG));
}

void loop()
{
	// communicate with HITIPanel
	HC_communicate();
}
