/*
 HITIComm examples:  Basics / 1_DigitalInputOutput

 This sketch shows how to use HITIPanel software to:
  => monitor a switch
  => switch on/off a LED
 
 - switch         on pin 3
 - on-board LED   on pin 13

 Copyright © 2021 Christophe LANDRET
 MIT License
 */


#include <HITIComm.h>

const int pin_Switch = 3;
const int pin_LED    = LED_BUILTIN;

void setup()
{
    // initialize library
    HC_begin(); 
   
    // pins mode
    pinMode(pin_Switch,  INPUT);  // pin 3  -> INPUT
    pinMode(pin_LED,     OUTPUT); // pin 13 -> OUTPUT
    
    // switch ON the on-board LED
    digitalWrite(pin_LED, HIGH);
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate(); 
}
