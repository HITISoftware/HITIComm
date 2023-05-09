/*
 HITIComm examples:  Eeprom / 3_StringSpaceConfig

 This sketch shows how to:
   => configure the String Space

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
		0,      // Boolean space : default 50
		0,      // Byte space    : default 100
		0,      // Integer space : default 200
		0,      // Long space    : default 250
		0,      // Float space   : default 250
		4096);  // String space  : default 4096

	// set max String length (does not include the terminating char '\0')
	HC_eeprom.setMaxStringLength(20); // 1 to 30, default 30
}

void loop()
{
	// communicate with HITIPanel
	HC_communicate();
}
