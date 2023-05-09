/*
 HITIComm examples:  Basics / 2_AnalogInput

 This sketch shows how to use HITIPanel software to: 
   => read raw feedback from an analog sensor

 - Potentiometer (or any 0-5V analog sensor) on pin A0

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
	// communicate with HITIPanel
	HC_communicate();
}
