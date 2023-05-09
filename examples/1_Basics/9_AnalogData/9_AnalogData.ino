/*
 HITIComm examples:  Basics / 8_AnalogData

 This sketch shows how to use HITIPanel software to:
   => turn on/off the on-board LED according to a control value (HITI Analog Data)

 - on-board LED     on pin 13
 - control value    on Analog Data 0
 - threshold value  on Analog Data 1

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

// pins assignment
const int pin_LED = LED_BUILTIN;

// Analog Data assignment
const int ad_control = 0;
const int ad_threshold = 1;

// threshold value
int threshold = 1500;

void setup()
{
    // initialize library
    HC_begin();

    // set pins mode
    pinMode(pin_LED, OUTPUT);
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();

    // get setpoint value from HITIPanel (Command Panel 0)
    int control = (int)HC_analogDataRead(ad_control);

    // if the received value is bigger than the threshold value
    if (control >= threshold)
        digitalWrite(pin_LED, HIGH); // turn ON the LED
    else
        digitalWrite(pin_LED, LOW);  // turn OFF the LED

    // display threshold value in HITIPanel (Command Panel 1)
    HC_analogDataWrite(ad_threshold, threshold);
}
