/*
 HITIComm examples:  Basics / 10_LightDetector

 This sketch shows how to use HITIPanel software to:
   => control a servo with a hysteresis light detector

 - servo                          on pin 8
 - on-board LED                   on pin 13
 - photoresistor (light sensor)   on pin A2
 - low threshold                  on Analog Data 0
 - high threshold                 on Analog Data 1

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

// code ID
const char code_name[]    PROGMEM = "Light Detector";
const char code_version[] PROGMEM = "1.0.0";

// pins assignment
const int pin_servo = 8;
const int pin_LED = LED_BUILTIN;
const int pin_LightSensor = A2;

// Analog Data assignment:
const int ad_threshold_Low = 0;
const int ad_threshold_High = 1;

// hysteresis filter threshold values
int threshold_Low = 0;
int threshold_High = 1023;

void setup()
{
	// initialize library
	HC_begin();

	// set code ID
	HC_codeName(code_name);
	HC_codeVersion(code_version);

	// set pins mode
	pinMode(pin_LED, OUTPUT);

	// attach servo to the pin. Move servo to position 10°.
	HC_attachServo(pin_servo, 10000);

	// send initial threshold values to HITIPanel
	HC_analogDataWrite(ad_threshold_Low, threshold_Low);
	HC_analogDataWrite(ad_threshold_High, threshold_High);
}

void loop()
{
	// communicate with HITIPanel
	HC_communicate();

	// read threshold values from HITIPanel
	threshold_Low = (int)HC_analogDataRead(ad_threshold_Low);
	threshold_High = (int)HC_analogDataRead(ad_threshold_High);

	// check inputs range and values
	checkInputs();

	// if light is detected
	if (analogRead(pin_LightSensor) > threshold_High)
	{
		digitalWrite(pin_LED, HIGH);      // turn LED on
		HC_servoWrite(pin_servo, 170000); // move servo to position 170°
	}
	// if dark is detected
	else if (analogRead(pin_LightSensor) < threshold_Low)
	{
		digitalWrite(pin_LED, LOW);       // turn LED off
		HC_servoWrite(pin_servo, 10000);  // move servo back to position 10°
	}
}

void checkInputs()
{
	// check range : 0 <= Threshold <= 1023
	if (threshold_Low > 1023)
	{
		threshold_Low = 1023;

		// update value in HITIPanel
		HC_analogDataWrite(ad_threshold_Low, threshold_Low);
	}
	if (threshold_High > 1023)
	{
		threshold_High = 1023;

		// update value in HITIPanel
		HC_analogDataWrite(ad_threshold_High, threshold_High);
	}

	// check values : Low threshold <= High threshold
	if (threshold_Low > threshold_High)
	{
		threshold_Low = threshold_High;

		// update value in HITIPanel
		HC_analogDataWrite(ad_threshold_Low, threshold_Low);
	}
	if (threshold_High < threshold_Low)
	{
		threshold_High = threshold_Low;

		// update value in HITIPanel
		HC_analogDataWrite(ad_threshold_High, threshold_High);
	}
}
