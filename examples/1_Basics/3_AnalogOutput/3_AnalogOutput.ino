/*
 HITIComm examples:  Basics / 3_AnalogOutput

 This sketch shows how to use HITIPanel software to: 
   => control a LED intensity

 - LED on pin 6 (or any pin where PWM is available)

 Copyright © 2021 Christophe LANDRET
 MIT License
 */

#include <HITIComm.h>

const int pin_LED = 6;

void setup()
{
	// initialize library
	HC_begin();
	
	// set pin 6 as a PWM Output
	pinMode(pin_LED, OUTPUT);
	HC_outputType(pin_LED, PWM);

	// turn LED on with medium intensity
	analogWrite (pin_LED, 127);
}

void loop()
{
	// communicate with HITIPanel
	HC_communicate();
}
