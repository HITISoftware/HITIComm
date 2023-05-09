/*
 HITIComm examples:  Eeprom / 1_DefaultSpaceConfig

 This sketch shows how to use HITIPanel software to:
   => view and manage EEPROM content using the EEPROM Panel

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

void setup()
{
    // initialize library
    HC_begin();
}

void loop()
{
    // handle EEPROM access. Set default User Space configuration.
    HC_communicate();
}
