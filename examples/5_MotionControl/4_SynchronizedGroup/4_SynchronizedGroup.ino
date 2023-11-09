/*
 HITIComm examples:  MotionControl / 4_SynchronizedGroup

 This sketch shows how to use a HITI Motor Group to synchronize motions of 3 HITI Servos.
 (All Servos added to this Group are automatically synchronized)

 The motion sequence in this example moves the servos back-and-forth:
    - step 0 : ready to start. Wait for the Start
    - step 1 : move Group (servos 1, 2, 3)
    - step 2 : move Group (servos 1, 2, 3) back to initial position

 - Servos 1-3                     on pin 8-10
 - Start/Stop (virtual switches)  on Digital Data 0-1
 - Group state                    on Digital Data 2-3
 - Sequence step                  on Analog Data 0
 - Current position (Servo 1-3)   on Analog Data 1-3
 
 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_Servo.h>
#include <HC_MotorGroup.h>

// pins assignment 
const int pin_Servo_1 = 8;  // servos
const int pin_Servo_2 = 9;
const int pin_Servo_3 = 10;

// Analog Data assignment
const int ad_Step         = 0; // metric (sequence step)
const int ad_Position_S1  = 1; // metric (position in °)
const int ad_Position_S2  = 2; // metric (position in °)
const int ad_Position_S3  = 3; // metric (position in °)
const int ad_maxSpeed_S1  = 4; // setpoint (max speed of servo 1 in °/s)
const int ad_motionTime_G = 5; // metric (motion time of the Group in s)

// Digital Data assignment
const int dd_Start        = 0; // virtual button
const int dd_Stop         = 1; // virtual button
const int dd_isReady_G    = 2; // indicator
const int dd_isMoving_G   = 3; // indicator

// motion sequence in 5 steps: 0 (Ready), 1-2 (Motions)
int step = 0;

// max speed (servo 1)
float maxSpeed_S1 = 100.0;

// HITI Motor Group
HC_MotorGroup group;

// HITI Servo
HC_Servo servo_1;
HC_Servo servo_2;
HC_Servo servo_3;


void setup()
{
    // 1) initialize library
    HC_begin();

    // 2) initialize HITI Servos
    // param: custom id, pin, invert direction, position offset, absolute init position
    servo_1.init(1, pin_Servo_1, true,  3,  20);
    servo_2.init(2, pin_Servo_2, false, -1, 40);
    servo_3.init(3, pin_Servo_3, false, 0,  60);
    
    // 3) initialize HITI Motor Group and add Servos to it
    group.init(3);
    group.add(&servo_1);
    group.add(&servo_2);
    group.add(&servo_3);

    // 4) set max speed of Servos, if required (affect motion time)
    servo_1.maxSpeed(100); // °/s

    // 5) display initial values of control parameters in HITIPanel
    HC_analogDataWrite(ad_maxSpeed_S1, maxSpeed_S1); // max speed (servo 1)
}


void loop()
{
    // communicate with HITIPanel
    HC_communicate();


    // set new control parameters --------------------------------------------

    // read from HITIPanel
    maxSpeed_S1 = HC_analogDataRead(ad_maxSpeed_S1);

    // set max speed (servo 1)
    servo_1.maxSpeed(maxSpeed_S1);

   
    // start/stop motion sequence --------------------------------------------
    
    // step 0: ready to start sequence with the START button
    if(HC_digitalDataRead_click(dd_Start) && (step == 0))
        step = 1; // start sequence
    
    // at any step: stop sequence with the STOP button
    if(HC_digitalDataRead_click(dd_Stop))
    {
        group.stopNow(); // stop group
        step = 0;        // reset sequence
    }
    
    // step 1
    else if(step == 1)
    {
        // move group
        group.relativePosition(1, 10);  // Servo 1: +10°
        group.relativePosition(2, 50);  // Servo 2: +50°
        group.relativePosition(3, 90);  // Servo 3: +90°
        group.motionTime(1.5);          // 1.5s
        group.moveNow();
    
        // when motion ends
        if(group.isEnding())
            onMotionDone();
    }
    
    // step 2
    else if(step == 2)
    {
        // move group
        group.relativePosition(1, -10); // Servo 1: -10°
        group.relativePosition(2, -50); // Servo 2: -50°
        group.relativePosition(3, -90); // Servo 3: -90°
        group.motionTime(4);            // 4s
        group.moveNow();
    
        // when motion ends
        if(group.isEnding())
            onMotionDone();
    }
    
    
    // display data in HITIPanel ---------------------------------------------
    
    // sequence step
    HC_analogDataWrite(ad_Step, step);
    
    // current positions (servos)
    HC_analogDataWrite(ad_Position_S1, servo_1.getCurrentPosition());
    HC_analogDataWrite(ad_Position_S2, servo_2.getCurrentPosition());
    HC_analogDataWrite(ad_Position_S3, servo_3.getCurrentPosition());
    
    // motion time (group)
    HC_analogDataWrite(ad_motionTime_G, group.getMotionTime());
    
    // state (group)
    HC_digitalDataWrite(dd_isReady_G,  group.isReady());
    HC_digitalDataWrite(dd_isMoving_G, group.isMoving());
}


// called each time a motion ends
void onMotionDone()
{
    // at end of steps 0-1
    if (step < 2)
        step++;   // go to next step
    // at end of step 2
    else
        step = 0; // go to step 0
}
