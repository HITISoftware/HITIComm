/*
 HITIComm examples:  MotionControl / 4_SynchronizedGroup

 This sketch shows how to use HITI Motor Groups to synchronize motions of several servos.
 All motors belonging to a Group are synchronized. In this example:
   => group 1 contains servos 1 and 2
   => group 2 contains servos 3 and 4

 Motion sequence performed (back-and-forth motions):
    - step 0 : ready to start. Wait for the Start
    - step 1 : move group 1 (servos 1, 2)
    - step 2 : move group 2 (servos 3, 4)
    - step 3 : move group 1 (servos 1, 2) back to initial position
    - step 4 : move group 2 (servos 3, 4) back to initial position

 - Servos                         on pin 8-11
 - Start/Stop (virtual switches)  on Digital Data 0-1
 - State (Group 1-2)              on Digital Data 2-5
 - Sequence step                  on Analog Data 0
 - Current position (Servo 1-4)   on Analog Data 1-4
 
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
const int pin_Servo_4 = 11;

// Analog Data assignment
const int ad_Step          = 0; // sequence step
const int ad_Position_S1   = 1; // current positions (Servo 1-4)
const int ad_Position_S2   = 2;
const int ad_Position_S3   = 3;
const int ad_Position_S4   = 4;
const int ad_maxSpeed_S1   = 5; // max speed (Servo 1)
const int ad_motionTime_G1 = 6; // motion times (Group 1-2)
const int ad_motionTime_G2 = 7; 

// Digital Data assignment
const int dd_Start         = 0; // virtual switches
const int dd_Stop          = 1;
const int dd_isReady_G1    = 2; // group 1 state
const int dd_isMoving_G1   = 3;
const int dd_isReady_G2    = 4; // group 2 state
const int dd_isMoving_G2   = 5;

// motion sequence in 5 steps: 0 (Ready), 1-4 (Motions)
int step = 0;

// initial servos positions
const float initPos_S1 = 100.5;
const float initPos_S2 = 160;
const float initPos_S3 = 20;
const float initPos_S4 = 90.2;

// max speed (servo 1)
float maxSpeed_S1 = 100.0;

// HITI Motor Groups
HC_MotorGroup group_1;
HC_MotorGroup group_2;

// HITI Servo
HC_Servo servo_1;
HC_Servo servo_2;
HC_Servo servo_3;
HC_Servo servo_4;


// run once at power on
void setup()
{
    // 1) initialize library
    HC_begin();

    // 2) initialize HITI Servos
    // param: custom id, pin, invert direction, position offset, absolute init position
    servo_1.init(1, pin_Servo_1, true,  3,  initPos_S1);
    servo_2.init(2, pin_Servo_2, false, -1, initPos_S2);
    servo_3.init(3, pin_Servo_3, false, 0,  initPos_S3);
    servo_4.init(4, pin_Servo_4, true,  5,  initPos_S4);
    
    // 3) initialize group 1 and assign it 2 servos (servos 1, 2)
    group_1.init(2);
    group_1.add(&servo_1);
    group_1.add(&servo_2);

    // 4) initialize group 2 and assign it 2 servos (servos 1, 2)
    group_2.init(2);
    group_2.add(&servo_3);
    group_2.add(&servo_4);

    // 5) set max speed of Servos, if required (affect motion time)
    servo_1.maxSpeed(100); // °/s

    // 6) send initial values of control parameters to HITIPanel
    HC_analogDataWrite(ad_maxSpeed_S1, maxSpeed_S1); // max speed (servo 1)
}


// run repeatedly after setup()
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
    
    // step 0: ready to start sequence with the START virtual switch
    if((step == 0) && HC_digitalDataRead(dd_Start))
    {
        step = 1; // start sequence
    }
    
    // stop motion sequence on Trigger Stop
    if(HC_digitalDataRead(dd_Stop))
    {
        group_1.stopNow(); // stop groups
        group_2.stopNow();
    
        step = 0;          // reset sequence
    }
    
    // step 1
    else if(step == 1)
    {
        // move servos of group 1
        group_1.relativePosition(1, 40);   // Servo 1: +40°
        group_1.relativePosition(2, -10);  // Servo 2: -10°
        group_1.motionTime(1.5);           // 1.5s
        group_1.moveNow();
    
        // when motion ends
        if(group_1.isEnding())
        {
            onMotionDone();
        }
    }
    
    // step 2
    else if(step == 2)
    {
        // move servos of group 2
        group_2.relativePosition(3, 20);     // Servo 3: +20°
        group_2.relativePosition(4, -60);    // Servo 4: -60.2°
        group_2.motionTime(2.5);             // 2.5s
    
        // move now all motors of the group
        group_2.moveNow();
    
        // when motion ends
        if(group_2.isEnding()) 
        {
            onMotionDone();
        }
    }
    
    // step 3
    else if(step == 3)
    {
        // move servos of group 1
        group_1.absolutePosition(1, initPos_S1); // Servo 1: 10°
        group_1.absolutePosition(2, initPos_S2); // Servo 2: 160°
        group_1.motionTime(4);                   // 4s
        group_1.moveNow();
    
        // when motion ends
        if(group_1.isEnding())
        {
            onMotionDone();
        }
    }
    
    // step 4
    else if(step == 4)
    {
        // move servos of group 2
        group_2.absolutePosition(3, initPos_S3); // Servo 3: 80°
        group_2.absolutePosition(4, initPos_S4); // Servo 4: 120°
        group_2.motionTime(2);                   // 2s
        group_2.moveNow();
    
        // when motion ends
        if(group_2.isEnding())
        {
            onMotionDone();
        }
    }
    
    // reset trigger
    HC_digitalDataWrite(dd_Start, false);
    HC_digitalDataWrite(dd_Stop,  false);
    
    
    // send data to HITIPanel ------------------------------------------------
    
    // sequence step
    HC_analogDataWrite(ad_Step, step);
    
    // current positions (servos)
    HC_analogDataWrite(ad_Position_S1, servo_1.getCurrentPosition());
    HC_analogDataWrite(ad_Position_S2, servo_2.getCurrentPosition());
    HC_analogDataWrite(ad_Position_S3, servo_3.getCurrentPosition());
    HC_analogDataWrite(ad_Position_S4, servo_4.getCurrentPosition());
    
    // motion times (groups)
    HC_analogDataWrite(ad_motionTime_G1, group_1.getMotionTime());
    HC_analogDataWrite(ad_motionTime_G2, group_2.getMotionTime());
    
    // states (groups)
    HC_digitalDataWrite(dd_isReady_G1,  group_1.isReady());
    HC_digitalDataWrite(dd_isMoving_G1, group_1.isMoving());
    HC_digitalDataWrite(dd_isReady_G2,  group_2.isReady());
    HC_digitalDataWrite(dd_isMoving_G2, group_2.isMoving());
}


// called each time a motion ends
void onMotionDone()
{
    // at end of steps 0-3
    if (step < 4)
    {
        step++;   // go to next step
    }
    // at end of step 4
    else
    {
        step = 0; // go to step 0
    }
}
