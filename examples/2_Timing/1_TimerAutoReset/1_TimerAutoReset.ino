/*
 HITIComm examples:  Timing / 1_TimerAutoReset

 This sketch shows how to use HITIPanel software to:
   => control the blinking of the on-board LED using a HITI Timer

 - on-board LED   on pin 13

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

// pins assignment
const int pin_LED = LED_BUILTIN;

// HITI Timer
HC_Timer timer;

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

    // run Timer
    if (timer.delay(500))
    {
        // when Timer ends (every 0.5s)  => toggle LED state
        if (digitalRead(pin_LED) == LOW)
            digitalWrite(pin_LED, HIGH);
        else
            digitalWrite(pin_LED, LOW);
    }
}
