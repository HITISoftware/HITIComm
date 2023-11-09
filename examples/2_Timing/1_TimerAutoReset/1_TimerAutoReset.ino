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

// LED state (ON/OFF)
bool LED_state;

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
        // when Timer ends (every 0.5s)  => toggle LED state
        LED_state = !LED_state;

    digitalWrite(pin_LED, LED_state);
}
