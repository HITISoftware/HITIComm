/*
 HITIComm examples:  Timing / 2_TimerManualReset

 This sketch shows how to use HITIPanel software to:
   => trigger the lighting of the on-board LED for a specified duration using a HITI Timer

 - on-board LED   on pin 13

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

// pins assignment
const int pin_LED = LED_BUILTIN;

// Digital Data assignment
const int dd_ResetButton = 0;

// Analog Data assignment
const int ad_StartTime = 0;
const int ad_ElapsedTime = 1;

// HITI Timer
HC_Timer timer;

void setup()
{
    // initialize library
    HC_begin();

    // set pins mode
    pinMode(pin_LED, OUTPUT);

    // set Timer to manual reset mode
    timer.manualReset();
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();

    // when Timer starts
    if (timer.isStarting())
        digitalWrite(pin_LED, HIGH);

    // after 2000 ms
    if (timer.delay(2000))
        digitalWrite(pin_LED, LOW);

    // if the Button is clicked (from Command Panel DD0) 
    if (HC_digitalDataRead_click(dd_ResetButton) == HIGH)
        // reset Timer
        timer.reset();

    // display Start Time and Elapsed Time (in Command Panels AD0 and AD1)
    HC_analogDataWrite(ad_StartTime, timer.getStartTime());
    HC_analogDataWrite(ad_ElapsedTime, timer.getElapsedTime());
}
