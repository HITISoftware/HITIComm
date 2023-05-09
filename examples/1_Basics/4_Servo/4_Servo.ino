/*
 HITIComm examples:  Basics / 4_Servo

 This sketch shows how to use HITIPanel software to:
   => control a servo

 - Servo on pin 8

 Copyright © 2021 Christophe LANDRET
 MIT License
 */

#include <HITIComm.h>

void setup()
{
    // initialize library
    HC_begin();

    // attach servo to pin 6. Initial position is 53.7°. 
    // (if not specified in parameters, default initial position is 90°)
    HC_attachServo(6, 53700);
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();
}
