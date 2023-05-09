/*
 HITIComm examples:  Basics / 5_DigitalData

 This sketch shows how to use HITIPanel software to:
   => toggle position of a servo using a Virtual Switch (HITI Digital Data)

 - servo            on pin 8
 - virtual switch   on Digital Data 0

 Copyright © 2021 Christophe LANDRET
 MIT License
 */

#include <HITIComm.h>

// Pins assignment
const int pin_Servo = 8;

// Digital Data assignment
const int dd_VirtualSwitch = 0;

void setup()
{
	// initialize library
	HC_begin();

	// attach servo to the pin. Move servo to position 12.8°
	HC_attachServo(pin_Servo, 12800);
}

void loop()
{
	// communicate with HITIPanel
	HC_communicate();

	// if the Virtual Switch is activated (from Command Panel 0)
	if (HC_digitalDataRead(dd_VirtualSwitch) == HIGH)
	{
		// move Servo to position 169.3°
		HC_servoWrite(pin_Servo, 169300);
	}
	else
	{
		// move Servo to position 12.8°
		HC_servoWrite(pin_Servo, 12800);
	}
}
