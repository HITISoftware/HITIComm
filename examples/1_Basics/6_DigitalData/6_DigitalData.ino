/*
 HITIComm examples:  Basics / 6_DigitalData

 This sketch shows how to use HITIPanel software to:
   => move servo by steps of 10° using a Virtual Button (Digital Data)

 - servo            on pin 8
 - virtual button   on Digital Data 0

 Copyright © 2021 Christophe LANDRET
 MIT License
 */

#include <HITIComm.h>

// Pins assignment
const int pin_Servo = 8;

// Digital Data assignment
const int dd_VirtualButton = 0;

// Servo position
long pos = 0;

void setup()
{
	// initialize library
	HC_begin();

	// attach servo to the pin. Move servo to position 0°
	HC_attachServo(pin_Servo, pos);
}

void loop()
{
	// communicate with HITIPanel
	HC_communicate();

	// if the Virtual Button is activated (from Command Panel 0)
	if (HC_digitalDataRead(dd_VirtualButton) == HIGH)
	{
		// immediately deactivate Button
		HC_digitalDataWrite(dd_VirtualButton, LOW);

		moveServo();
	}
}

// move servo 10°. Move back to 0° if position exceeds 170°.
void moveServo()
{
	// increment position by 10°
	pos = pos + 10000;

	// reset position if it exceeds 170°
	if(pos > 170000)
		pos = 0;

	// move servo
	HC_servoWrite(pin_Servo, pos);
}
