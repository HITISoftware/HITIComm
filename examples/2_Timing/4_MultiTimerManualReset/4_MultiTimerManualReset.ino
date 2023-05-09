/*
 HITIComm examples:  Timing / 4_MultiTimerManualReset

 This sketch shows how to use HITIPanel software to:
   => use a HITI MultiTimer to trigger emission of a complex coded light signal by the on-board LED

 - on-board LED   on pin 13

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_MultiTimer.h>

// pins assignment
const int pin_LED = LED_BUILTIN;

// Digital Data assignment
const int dd_ResetSwitch = 0;

// HITI MultiTimer with 5 Timers
HC_MultiTimer timers(5);

void setup()
{
    // initialize library
    HC_begin();

    // set pins mode
    pinMode(pin_LED, OUTPUT);

    // configure MultiTimer for manual reset
    timers.manualReset();
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();


    // when Timer 0 starts
    if (timers.isStarting())
        digitalWrite(pin_LED, HIGH); // LED is turned ON

    // after 250 ms (when Timer 0 ends)
    if (timers.delay(0, 250))
        digitalWrite(pin_LED, LOW); // LED is turned OFF

    // after 500 ms (when Timer 1 ends)
    if (timers.delay(1, 500))
        digitalWrite(pin_LED, HIGH); // LED is turned ON

    // after 1000 ms (when Timer 2 ends)
    if (timers.delay(2, 1000))
        digitalWrite(pin_LED, LOW); // LED is turned OFF

    // after 1500 ms (when Timer 3 ends), do nothing
    timers.delay(3, 1500);

    // if the Button is activated (from Command Panel 0) 
    if (HC_digitalDataRead(dd_ResetSwitch) == HIGH)
    {
        // deactivate the Button
        HC_digitalDataWrite(dd_ResetSwitch, LOW);

        // reset MultiTimer
        timers.reset();
    }

    // display Start Times and Elapsed Times (Timers 0 to 3 => AD0 to AD3)
    for (byte i = 0; i <= 3; i++)
    {
        HC_analogDataWrite(i, timers.getTimer(i).getStartTime());
        HC_analogDataWrite(i + 4, timers.getTimer(i).getElapsedTime());
    }
}
