/*
 HITIComm examples:  Timing / 3_MultiTimerAutoReset

 This sketch shows how to use HITIPanel software to:
   => use a HITI MultiTimer to create a complex coded light signal emitted by the on-board LED

 - on-board LED   on pin 13

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_MultiTimer.h>

// pins assignment
const int pin_LED = LED_BUILTIN;

// HITI MultiTimer with 4 Timers
HC_MultiTimer timers(4);

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

    // display Elapsed Times (Timers 0 to 3 => AD0 to AD3)
    for (byte i = 0; i <= 3; i++)
        HC_analogDataWrite(i, timers.getTimer(i).getElapsedTime());
}
