/*
 HITIComm examples:  MotionControl / 1_ServoParameters

 This sketch is an introduction to servo control with a HITI Servo.

 - Servo                            on pin 8

 - Start/Stop (virtual switches)    on Digital Data 2-3
 - Servo state                      on Digital Data 4-5
 - Travel limits -/+                on Digital Data 6-7

 - Position control                 on Digital Data 0 and Analog Data 0-2
 - Speed control                    on Digital Data 1 and Analog Data 3-6
 - Motion time                      on Analog Data 7

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_Servo.h>

// pins assignment 
const int pin_Servo = 8;

// Analog Data assignment
const int ad_PosSetpoint       = 0; // position (setpoint, target, current)
const int ad_PosTarget         = 1;
const int ad_PosCurrent        = 2;
const int ad_SpeSetpoint       = 3; // speed (setpoint, max, target, current)
const int ad_SpeMax            = 4;
const int ad_SpeTarget         = 5;
const int ad_SpeCurrent        = 6;
const int ad_MotionTime        = 7; // motion time (target)

// Digital Data assignment
const int dd_PosType           = 0; // position (setpoint type)
const int dd_SpeType           = 1; // speed (setpoint type)
const int dd_Start             = 2; // virtual switches
const int dd_Stop              = 3; 
const int dd_ServoIsReady      = 4; // servo state
const int dd_ServoIsMoving     = 5;
const int dd_NegLimitIsReached = 6; // travel limits
const int dd_PosLimitIsReached = 7;

// position control
bool  posType     = LOW; // LOW: absolute    /  HIGH: relative
float posSetpoint = 30;  // 30° absolute

// speed control
bool  speType     = LOW; // LOW: speed(°/s)  /  HIGH: motion time(s)
float speSetpoint = 10;  // 10°/s
float speMax      = 100; // 100 (°/s)

// HITI Servo
HC_Servo servo;


void setup()
{
    // initialize library
    HC_begin();

    // initialize HITI Servo
    servo.init(
            0,           // ID = 0
            pin_Servo,   // attached to pin 8
            true,        // true => invert direction
            -4.3,        // position offset = -4.3°
            70.0);       // initial position = 70°
    
    // set travel limits -/+ (°)
    servo.travelLimits(17.0, 165.0);
    
    // display initial values of control parameters in HITIPanel
    HC_digitalDataWrite(dd_PosType,    posType);     // position
    HC_analogDataWrite(ad_PosSetpoint, posSetpoint);
    HC_digitalDataWrite(dd_SpeType,    speType);     // speed
    HC_analogDataWrite(ad_SpeSetpoint, speSetpoint);
    HC_analogDataWrite(ad_SpeMax,      speMax);
}


void loop()
{
    // communicate with HITIPanel
    HC_communicate();


    // set new control parameters ---------------------------------------------
    
    // read from HITIPanel
    posType     = HC_digitalDataRead(dd_PosType);
    posSetpoint = HC_analogDataRead(ad_PosSetpoint);
    speType     = HC_digitalDataRead(dd_SpeType);
    speSetpoint = HC_analogDataRead(ad_SpeSetpoint);
    speMax      = HC_analogDataRead(ad_SpeMax);

    // 1) set position and speed setpoints
    if (posType == HIGH)
        servo.relativePosition(posSetpoint);
    else
        servo.absolutePosition(posSetpoint);

    if (speType == HIGH)
        servo.motionTime(speSetpoint);
    else
        servo.speed(speSetpoint);

    // 2) set max speed
    servo.maxSpeed(speMax);


    // start/stop motion -----------------------------------------------------
    
    // move/stop on trigger (START/STOP buttons)
    servo.moveOnTrigger(HC_digitalDataRead(dd_Start));
    servo.stopOnTrigger(HC_digitalDataRead(dd_Stop));

    // immediately deactivate Buttons
    HC_digitalDataWrite(dd_Start, false);
    HC_digitalDataWrite(dd_Stop,  false);


    // display data in HITIPanel ---------------------------------------------

    // control parameters
    HC_analogDataWrite(ad_PosTarget,  servo.getTargetPosition());  // position
    HC_analogDataWrite(ad_PosCurrent, servo.getCurrentPosition()); 
    HC_analogDataWrite(ad_SpeTarget,  servo.getTargetSpeed());     // speed
    HC_analogDataWrite(ad_SpeCurrent, servo.getCurrentSpeed());
    HC_analogDataWrite(ad_MotionTime, servo.getMotionTime());      // motion time

    // servo state
    HC_digitalDataWrite(dd_ServoIsReady,  servo.isReady());
    HC_digitalDataWrite(dd_ServoIsMoving, servo.isMoving());

    // has the servo reached a travel limit?
    HC_digitalDataWrite(dd_NegLimitIsReached, servo.isNegLimitReached());
    HC_digitalDataWrite(dd_PosLimitIsReached, servo.isPosLimitReached());
}
