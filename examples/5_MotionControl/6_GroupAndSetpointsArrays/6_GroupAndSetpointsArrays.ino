/*
 HITIComm examples:  MotionControl / 6_GroupAndSetpointsArrays

 This sketch shows how to create a complex motion sequence of several synchronized
 servos by using a HITI Motor Group together with arrays of setpoints.

 Motion sequence performed:
    - step 0 : ready to start. Wait for the Start
    - step 1 : move group (all servos) to position 1
    - step 2 : move group (all servos) to position 2
    - step 3 : move group (all servos) to position 3
    - step 4 : move group (all servos) to position 4
    - step 5 : move group (all servos) back to initial position

 - Servos 1-4                      on pin 8-11
 - Start/Stop (virtual switches)   on Digital Data 0-1
 - Sequence step                   on Analog Data 0
 - Current position (Servos 1-4)   on Analog Data 1-6

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
const int ad_Step        = 0; // metric (sequence step)
const int ad_Position_S1 = 1; // metric (position in °)
const int ad_Position_S2 = 2; // metric (position in °)
const int ad_Position_S3 = 3; // metric (position in °)
const int ad_Position_S4 = 4; // metric (position in °)

// Digital Data assignment
const int dd_Start = 0; // virtual button
const int dd_Stop  = 1; // virtual button

// motion sequence in 6 steps: 0 (Ready), 1-5 (Motions)
int step = 0;

// Initial position
const int initPos_S1 = 70;
const int initPos_S2 = 80;
const int initPos_S3 = 90;
const int initPos_S4 = 100;

// Positions setpoints              Step 1  Step 2  Step 3  Step 4  Step 5
const float positionArray_S1[5] = { 10,     40,     40,     20,     initPos_S1 };
const float positionArray_S2[5] = { 60,     50,     100,    70,     initPos_S2 };
const float positionArray_S3[5] = { 110,    110,    60,     140,    initPos_S3 };
const float positionArray_S4[5] = { 160,    160,    120,    160,    initPos_S4 };

// Motion Times setpoints
const float motionTimeArray[5]  = { 1,      2.5,    3.2,    0.8,    4 };

// HITI Motor Group
HC_MotorGroup group;

// HITI Servo
HC_Servo servo_1;
HC_Servo servo_2;
HC_Servo servo_3;
HC_Servo servo_4;


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

    // 3) initialize group and assign it all servos
    group.init(4);
    group.add(&servo_1);
    group.add(&servo_2);
    group.add(&servo_3);
    group.add(&servo_4);
}


void loop()
{
    // communicate with HITIPanel
    HC_communicate();


    // start/stop motion sequence --------------------------------------------

    // step 0: ready to start sequence with the START button
    if (HC_digitalDataRead_click(dd_Start) && (step == 0))
        step = 1; // start sequence

    // at any step: stop sequence with the STOP button
    if (HC_digitalDataRead_click(dd_Stop))
    {
        group.stopNow(); // stop group
        step = 0;        // reset sequence
    }

    // step 1 to 5
    else if ((1 <= step) && (step <= 5))
    {
        // move servos of the group 
        group.absolutePosition(1, positionArray_S1[step - 1]);
        group.absolutePosition(2, positionArray_S2[step - 1]);
        group.absolutePosition(3, positionArray_S3[step - 1]);
        group.absolutePosition(4, positionArray_S4[step - 1]);
        group.motionTime(motionTimeArray[step - 1]);
        group.moveNow();

        // when motion ends
        if (group.isEnding())
            onMotionDone();
    }


    // display data in HITIPanel ---------------------------------------------

    // sequence step
    HC_analogDataWrite(ad_Step, step);

    // current positions (servos)
    HC_analogDataWrite(ad_Position_S1, servo_1.getCurrentPosition());
    HC_analogDataWrite(ad_Position_S2, servo_2.getCurrentPosition());
    HC_analogDataWrite(ad_Position_S3, servo_3.getCurrentPosition());
    HC_analogDataWrite(ad_Position_S4, servo_4.getCurrentPosition());
}


// called each time a motion ends
void onMotionDone()
{
    // at end of steps 0-4
    if (step < 5)
        step++;   // go to next step

    // at end of step 5
    else
        step = 0; // go to step 0
}
