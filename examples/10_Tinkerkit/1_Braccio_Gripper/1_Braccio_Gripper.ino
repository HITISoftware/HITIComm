/*
HITIComm examples:  Amazing / 8_Tinkerkit_ServoParam

 This sketch is an introduction to servo control with a HITI Servo
 using the Gripper of the Tinkerkit Braccio arm

 - 6 Servos                         on pin 3,5,6,9,10,11

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
#include <Braccio.h>


// -----------------------------------------------------------------------------
// Global Variables ------------------------------------------------------------
// -----------------------------------------------------------------------------

// Pins assignment 
// Servo
const int pin_Servo_Base     = 11;
const int pin_Servo_Shoulder = 10;
const int pin_Servo_Elbow    = 9;
const int pin_Servo_WristVer = 6;
const int pin_Servo_WristRot = 5;
const int pin_Servo_Gripper  = 3;
// Braccio : Soft Start Control pin
const int pin_SoftStartControl = SOFT_START_CONTROL_PIN; // pin 12: internally used by Braccio

// Analog Data assignment
const int ad_PosSetpoint       = 0; // setpoint (in °)
const int ad_PosTarget         = 1; // metric (in °)
const int ad_PosCurrent        = 2; // metric (in °)
const int ad_SpeSetpoint       = 3; // setpoint (in °/s or in s)
const int ad_SpeMax            = 4; // setpoint (in °/s)
const int ad_SpeTarget         = 5; // metric (in °/s)
const int ad_SpeCurrent        = 6; // metric (in °/s)
const int ad_MotionTime        = 7; // metric (in s)

// Digital Data assignment
const int dd_PosType           = 0; // virtual switch (position setpoint type)
const int dd_SpeType           = 1; // virtual switch (speed setpoint type)
const int dd_Start             = 2; // virtual button
const int dd_Stop              = 3; // virtual button
const int dd_ServoIsReady      = 4; // indicator (servo state)
const int dd_ServoIsMoving     = 5; // indicator (servo state)
const int dd_NegLimitIsReached = 6; // indicator (travel limit)
const int dd_PosLimitIsReached = 7; // indicator (travel limit)

// position control
bool  posType     = LOW; // LOW: absolute    /  HIGH: relative
float posSetpoint = 30;  // 30° absolute

// speed control
bool  speType     = LOW; // LOW: speed(°/s)  /  HIGH: motion time(s)
float speSetpoint = 10;  // 10°/s
float speMax      = 100; // 100 (°/s)

// HITIComm Servo
HC_Servo servo_Base;
HC_Servo servo_Shoulder;
HC_Servo servo_Elbow;
HC_Servo servo_WristVer;
HC_Servo servo_WristRot;
HC_Servo servo_Gripper;

// Servo objects created by user and directly used by Braccio library (variable names must left unchanged)
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

// Initial position
const int initPos_Base     = 0;
const int initPos_Shoulder = 35;
const int initPos_Elbow    = 180;
const int initPos_WristVer = 0;
const int initPos_WristRot = 0;
const int initPos_Gripper  = 73;


// -----------------------------------------------------------------------------
// Setup (called once at power on) ---------------------------------------------
// -----------------------------------------------------------------------------

void setup()
{
	// HITIComm initialization
	HC_begin();

	// Braccio initialization 
	// (Braccio Shield v4 : all motors are safely placed in initial position using low voltage)
	Braccio.begin();

	// HC_Servo initialization
	// param: custom id, pin, invert direction, position offset, absolute initial position, servo
	servo_Base.    init(0, pin_Servo_Base,     false, 7.0, initPos_Base,     &base);
	servo_Shoulder.init(1, pin_Servo_Shoulder, false, 2.5, initPos_Shoulder, &shoulder);
	servo_Elbow.   init(2, pin_Servo_Elbow,    false, 4.0, initPos_Elbow,    &elbow);
	servo_WristVer.init(3, pin_Servo_WristVer, false, 7.0, initPos_WristVer, &wrist_ver);
	servo_WristRot.init(4, pin_Servo_WristRot, true,  0.0, initPos_WristRot, &wrist_rot);
	servo_Gripper. init(5, pin_Servo_Gripper,  false, 2.0, initPos_Gripper,  &gripper);

	// set Min and Max absolute position limits (min, max) (°)
	servo_Base.    travelLimits(0.0,  180.0);
	servo_Shoulder.travelLimits(17.0, 163.0);
	servo_Elbow.   travelLimits(0.0,  180.0);
	servo_WristVer.travelLimits(0.0,  180.0);
	servo_WristRot.travelLimits(0.0,  125.0);
	servo_Gripper. travelLimits(13.0, 77.0);

    // display initial values of control parameters in HITIPanel
    HC_digitalDataWrite(dd_PosType,    posType);     // position
    HC_analogDataWrite(ad_PosSetpoint, posSetpoint);
    HC_digitalDataWrite(dd_SpeType,    speType);     // speed
    HC_analogDataWrite(ad_SpeSetpoint, speSetpoint);
    HC_analogDataWrite(ad_SpeMax,      speMax);
}


// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
	// communicate with HITIPanel (to place at the beginning)
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
        servo_Gripper.relativePosition(posSetpoint);
    else
        servo_Gripper.absolutePosition(posSetpoint);

    if (speType == HIGH)
        servo_Gripper.motionTime(speSetpoint);
    else
        servo_Gripper.speed(speSetpoint);

    // 2) set max speed
    servo_Gripper.maxSpeed(speMax);


    // start/stop motion -----------------------------------------------------
    
    // move/stop on trigger (START/STOP buttons)
    servo_Gripper.moveOnTrigger(HC_digitalDataRead_click(dd_Start));
    servo_Gripper.stopOnTrigger(HC_digitalDataRead_click(dd_Stop));


    // display data in HITIPanel ---------------------------------------------

    // control parameters
    HC_analogDataWrite(ad_PosTarget,  servo_Gripper.getTargetPosition());  // position
    HC_analogDataWrite(ad_PosCurrent, servo_Gripper.getCurrentPosition());
    HC_analogDataWrite(ad_SpeTarget,  servo_Gripper.getTargetSpeed());     // speed
    HC_analogDataWrite(ad_SpeCurrent, servo_Gripper.getCurrentSpeed());
    HC_analogDataWrite(ad_MotionTime, servo_Gripper.getMotionTime());      // motion time

    // servo state
    HC_digitalDataWrite(dd_ServoIsReady,  servo_Gripper.isReady());
    HC_digitalDataWrite(dd_ServoIsMoving, servo_Gripper.isMoving());

    // has the servo reached a travel limit?
    HC_digitalDataWrite(dd_NegLimitIsReached, servo_Gripper.isNegLimitReached());
    HC_digitalDataWrite(dd_PosLimitIsReached, servo_Gripper.isPosLimitReached());
}
