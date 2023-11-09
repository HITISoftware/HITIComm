/*
 HITIComm examples:  MotionControl / 2_ServoMotionSequence

 This sketch shows how to perform a complex motion sequence using a HITI Servo.
 The motion sequence is divided into several steps.

 - Servo                   on pin 8
 - Start (virtual switch)  on Digital Data 0
 - Stop (virtual switch)   on Digital Data 1
 - Sequence step           on Analog Data 0
 - Position                on Analog Data 1

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_Servo.h>

// pins assignment
const int pin_Servo   = 8;

// Analog Data assignment
const int ad_Step     = 0; // metric (sequence step)
const int ad_Position = 1; // metric (servo position in °)

// Digital Data assignment
const int dd_Start    = 0; // virtual switch
const int dd_Stop     = 1; // virtual switch

// motion sequence in 5 steps: 0 (Ready), 1-4 (Motions)
int step = 0;

// HITI Servo
HC_Servo servo;


void setup()
{
    // initialize library
    HC_begin();
    
    // initialize HITI Servo
    servo.init(
            0,         // ID = 0
            pin_Servo, // attached to pin 8
            true,      // true => invert direction
            -4,        // position offset = -4°
            20.0);     // initial position = 20°
}


void loop()
{
    // communicate with HITIPanel (to place at the beginning)
    HC_communicate();
    
    
    // start/stop motion sequence --------------------------------------------
    
    // step 0: ready to start sequence with the START button
    if (HC_digitalDataRead_click(dd_Start) && (step == 0))
        step = 1; // start sequence
    
    // at any step: stop sequence with the STOP button
    if(HC_digitalDataRead_click(dd_Stop))
    {
        servo.stopNow();  // stop servo
        step = 0;         // reset sequence
    }
    
    // step 1:
    else if (step == 1)
        doStep(165, 0.5); // move to position 165° in 0.5s

    // step 2
    else if (step == 2)
        doStep(60, 5);    // move to position 30° in 5s

    // step 3
    else if (step == 3)
        doStep(120, 2.5); // move to position 120° in 2.5s

    // step 4
    else if (step == 4)
        doStep(20,  1.5); // move back to position 20° in 1.5s
    
        
    // display data in HITIPanel -----------------------------------------------
    
    HC_analogDataWrite(ad_Step,     step);
    HC_analogDataWrite(ad_Position, servo.getCurrentPosition());
}


// called by steps 1 to 4
void doStep(float position, float time)
{
    // move to requested position in requested time
    servo.absolutePosition(position);
    servo.motionTime(time);
    servo.moveNow();
    
    // when motion ends
    if (servo.isEnding())
        onMotionDone(); // update step
}


// called each time a motion ends
void onMotionDone()
{
    // at end of steps 0-3
    if (step < 4)
        step++;   // go to next step

    // at end of step 4
    else
        step = 0; // go to step 0
}
