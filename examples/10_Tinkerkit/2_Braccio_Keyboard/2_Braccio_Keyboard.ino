/*
HITIComm examples:  Amazing / 1_Tinkerkit_Braccio

 This sketch is an introduction to servo control with a HITI Servo
 using the clamp of the Tinkerkit Braccio arm

 - 6 Servos              on pin 3,5,6,9,10,11

 - Start/Stop (virtual switches)    on Digital Data 2-3
 - Servo state                      on Digital Data 4-5
 - Travel limits -/+                on Digital Data 6-7

 - Position control                 on Digital Data 0 and Analog Data 0-2
 - Speed control                    on Digital Data 1 and Analog Data 3-6
 - Motion time                      on Analog Data 7

This sketch shows how to use HITIPanel software to:
   => control a Tinkerkit Braccio arm using computer keyboard

 - 6 Servos              on pin 3,5,6,9,10,11
 - Trigger Go Forward    on Digital Data 0,2,4,6,8,10
 - Trigger Go Backward   on Digital Data 1,3,5,7,9,11
 - Position limits +/-   on Digital Data 12,13,14,15
 - Motion parameters     on Analog Data 0
 - Position              on Analog Data 4-9

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
// motion control
const int AD_ContinuousSpeedSetpoint = 0; // setpoint (in °/s)
// motion monitoring
const int AD_PositionRead_Base       = 4; // metric (in °)
const int AD_PositionRead_Shoulder   = 5; // metric (in °)
const int AD_PositionRead_Elbow      = 6; // metric (in °)
const int AD_PositionRead_WristVer   = 7; // metric (in °)
const int AD_PositionRead_WristRot   = 8; // metric (in °)
const int AD_PositionRead_Gripper    = 9; // metric (in °)

// Digital Data assignment
// motion triggers
const int DD_GoForward_Base             = 0; // virtual switch
const int DD_GoBackward_Base            = 1; // virtual switch
const int DD_GoForward_Shoulder         = 2; // virtual switch
const int DD_GoBackward_Shoulder        = 3; // virtual switch
const int DD_GoForward_Elbow            = 4; // virtual switch
const int DD_GoBackward_Elbow           = 5; // virtual switch
const int DD_GoForward_WristVer         = 6; // virtual switch
const int DD_GoBackward_WristVer        = 7; // virtual switch
const int DD_GoForward_WristRot         = 8; // virtual switch
const int DD_GoBackward_WristRot        = 9; // virtual switch
const int DD_GoForward_Gripper          = 10; // virtual switch
const int DD_GoBackward_Gripper         = 11; // virtual switch

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

// Continuous Motion control
float motion_continuousSpeedSetpoint = 30; // speed setpoint (°/s)

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
	HC_analogDataWrite(AD_ContinuousSpeedSetpoint, motion_continuousSpeedSetpoint);
}


// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
	// communicate with HITIPanel (to place at the beginning)
	HC_communicate();


	// receive and set new motion parameters**********************************

	// continuous speed setpoint (°/s)
	motion_continuousSpeedSetpoint = HC_analogDataRead(AD_ContinuousSpeedSetpoint);

	servo_Base.    continuousSpeed(motion_continuousSpeedSetpoint);
	servo_Shoulder.continuousSpeed(motion_continuousSpeedSetpoint);
	servo_Elbow.   continuousSpeed(motion_continuousSpeedSetpoint);
	servo_WristVer.continuousSpeed(motion_continuousSpeedSetpoint);
	servo_WristRot.continuousSpeed(motion_continuousSpeedSetpoint);
	servo_Gripper. continuousSpeed(motion_continuousSpeedSetpoint);


	// trigger motion*********************************************************

	// move forward or backward (if querried)
	servo_Base.    movePositive(HC_digitalDataRead(DD_GoForward_Base));
	servo_Base.    moveNegative(HC_digitalDataRead(DD_GoBackward_Base));
	servo_Shoulder.movePositive(HC_digitalDataRead(DD_GoForward_Shoulder));
	servo_Shoulder.moveNegative(HC_digitalDataRead(DD_GoBackward_Shoulder));
	servo_Elbow.   movePositive(HC_digitalDataRead(DD_GoForward_Elbow));
	servo_Elbow.   moveNegative(HC_digitalDataRead(DD_GoBackward_Elbow));
	servo_WristVer.movePositive(HC_digitalDataRead(DD_GoForward_WristVer));
	servo_WristVer.moveNegative(HC_digitalDataRead(DD_GoBackward_WristVer));
	servo_WristRot.movePositive(HC_digitalDataRead(DD_GoForward_WristRot));
	servo_WristRot.moveNegative(HC_digitalDataRead(DD_GoBackward_WristRot));
	servo_Gripper. movePositive(HC_digitalDataRead(DD_GoForward_Gripper));
	servo_Gripper. moveNegative(HC_digitalDataRead(DD_GoBackward_Gripper));

	// when motion ends, reset servo state so that it can start a new move
	if (servo_Base.    isDone())	servo_Base    .reset();
	if (servo_Shoulder.isDone())	servo_Shoulder.reset();
	if (servo_Elbow.   isDone())	servo_Elbow   .reset();
	if (servo_WristVer.isDone())	servo_WristVer.reset();
	if (servo_WristRot.isDone())	servo_WristRot.reset();
	if (servo_Gripper. isDone())	servo_Gripper .reset();


	// servo position monitoring**********************************************

	// Position
	HC_analogDataWrite(AD_PositionRead_Base,     servo_Base    .getCurrentPosition());
	HC_analogDataWrite(AD_PositionRead_Shoulder, servo_Shoulder.getCurrentPosition());
	HC_analogDataWrite(AD_PositionRead_Elbow,    servo_Elbow   .getCurrentPosition());
	HC_analogDataWrite(AD_PositionRead_WristVer, servo_WristVer.getCurrentPosition());
	HC_analogDataWrite(AD_PositionRead_WristRot, servo_WristRot.getCurrentPosition());
	HC_analogDataWrite(AD_PositionRead_Gripper,  servo_Gripper .getCurrentPosition());
}
