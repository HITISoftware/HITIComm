/*
 HITIComm examples:  MotionControl / 5_GroupRearranging

 This sketch shows how to create a complex motion sequence of several synchronized
 servos by rearranging the same HITI Motor Group. The composition of the Group is 
 changed at each sequence step.

 Motion sequence performed:
    - step 0 : ready to start. Wait for the Start
    - step 1 : move group (servos 1, 4)
    - step 2 : move group (servos 2, 3)
    - step 3 : move group (servos 1, 2, 3)
    - step 4 : move group (servos 1, 3, 4)
    - step 5 : move group (all servos) back to initial position

 - Servos 1-4                      on pin 8-11
 - Start/Stop (virtual switches)   on Digital Data 0-1
 - Sequence step                   on Analog Data 0
 - Current position (Servos 1-4)   on Analog Data 1-4

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
const int ad_Step        = 0; // sequence step
const int ad_Position_S1 = 1; // current positions (Servo 1-4)
const int ad_Position_S2 = 2;
const int ad_Position_S3 = 3;
const int ad_Position_S4 = 4;

// Digital Data assignment
const int dd_Start       = 0; // virtual buttons
const int dd_Stop        = 1;

// motion sequence in 6 steps: 0 (Ready), 1-5 (Motions)
int step = 0;

// Initial servos positions
const float initPos_S1 = 10;
const float initPos_S2 = 20;
const float initPos_S3 = 30;
const float initPos_S4 = 40;

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
}


void loop()
{
    // communicate with HITIPanel
    HC_communicate();


    // start/stop motion sequence --------------------------------------------

    // step 0: ready to start sequence with the START button
    if ((step == 0) && HC_digitalDataRead(dd_Start))
        step = 1; // start sequence

    // at any step: stop sequence with the STOP button
    if (HC_digitalDataRead(dd_Stop))
    {
        group.stopNow(); // stop group
        step = 0;        // reset sequence
    }

    // step 1
    else if (step == 1)
    {
        // init group and assign it 4 servos (servos 1, 2, 3, 4)
        group.init(4);
        group.add(&servo_1);
        group.add(&servo_2);
        group.add(&servo_3);
        group.add(&servo_4);

        // move servos of the group 
        group.absolutePosition(1, 70);  // servo 1: 70°
        group.absolutePosition(2, 100); // servo 2: 100°
        group.absolutePosition(3, 130); // servo 3: 130°
        group.absolutePosition(4, 160); // servo 4: 160°
        group.motionTime(1.5);          // 1.5s
        group.moveNow();

        // when motion ends
        if (group.isEnding())
            onMotionDone();
    }

    // step 2
    else if (step == 2)
    {
        // init group and assign it 2 servos (servos 1, 2)
        group.init(2);
        group.add(&servo_1);
        group.add(&servo_2);

        // move servos of the group
        group.absolutePosition(1, 40); // servo 1: 40°
        group.absolutePosition(2, 60); // servo 2: 60°
        group.motionTime(2.5);         // 2.5s
        group.moveNow();

        // when motion ends
        if (group.isEnding())
            onMotionDone();
    }

    // step 3
    else if (step == 3)
    {
        // init group and assign it 3 servos (servos 2, 3, 4)
        group.init(3);
        group.add(&servo_2);
        group.add(&servo_3);
        group.add(&servo_4);

        // move servos of the group
        group.absolutePosition(2, 80);  // servo 2: 80°
        group.absolutePosition(3, 60);  // servo 3: 60°
        group.absolutePosition(4, 100); // servo 4: 100°
        group.motionTime(5);            // 5s
        group.moveNow();

        // when motion ends
        if (group.isEnding())
            onMotionDone();
    }

    // step 4
    else if (step == 4)
    {
        // init group and assign it 3 servos (servos 1, 4)
        group.init(2);
        group.add(&servo_1);
        group.add(&servo_4);

        // move servos of the group
        group.absolutePosition(1, 20);  // servo 1: 20°
        group.absolutePosition(4, 160); // servo 4: 160°
        group.motionTime(0.5);          // 0.5s
        group.moveNow();

        // when motion ends
        if (group.isEnding())
            onMotionDone();
    }

    // step 5
    else if (step == 5)
    {
        // init group and assign it 4 servos (servos 1, 2, 3, 4)
        group.init(4);
        group.add(&servo_1);
        group.add(&servo_2);
        group.add(&servo_3);
        group.add(&servo_4);

        // move servos of the group
        group.absolutePosition(1, initPos_S1); // servo 1: 10°
        group.absolutePosition(2, initPos_S2); // servo 2: 20°
        group.absolutePosition(3, initPos_S3); // servo 3: 30°
        group.absolutePosition(4, initPos_S4); // servo 4: 40°
        group.motionTime(4);                   // 4s
        group.moveNow();

        // when motion ends
        if (group.isEnding())
            onMotionDone();
    }

    // deactivate the Virtual Buttons
    HC_digitalDataWrite(dd_Start, false);
    HC_digitalDataWrite(dd_Stop, false);


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
