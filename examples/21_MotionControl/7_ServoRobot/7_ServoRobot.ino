/*
HITIComm examples:  Motion Control / 7_ServoRobot

 This sketch shows how to perform:
   => SYNCHRONIZED complex moves of 1 Group by defining ARRAYS OF SETPOINT
 
 - Servo 1         on pin 8
 - Servo 2         on pin 9
 - Servo 3         on pin 10
 - Servo 4         on pin 11
 - Trigger Start   on HITI Data 0
 - Trigger Stop    on HITI Data 1
 - Motion state    on HITI Data A0
 - Positions       on HITI Data A1-A4
 
 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HITICommServoRobot.h>


// -----------------------------------------------------------------------------
// Global Variables ------------------------------------------------------------
// -----------------------------------------------------------------------------

// Pins assignment 
// Servo
const int pin_Servo_1 = 8;
const int pin_Servo_2 = 9;
const int pin_Servo_3 = 10;
const int pin_Servo_4 = 11;

// HITI Analog Data assignment
// motion monitoring
const int AD_MotionState    = 0;
const int AD_PositionRead_1 = 1;
const int AD_PositionRead_2 = 2;
const int AD_PositionRead_3 = 3;
const int AD_PositionRead_4 = 4;

// HITI Digital Data assignment
// motion trigger
const int dd_Trigger_Start  = 0;
const int dd_Trigger_Stop   = 1;

// Servo Robot
// enum giving an ID to each servo of the Robot
// (when using HC_ServoRobot, ID must be used as an index)
enum servos
{
	servo_ID_1 = 0,
	servo_ID_2 = 1,
	servo_ID_3 = 2,
	servo_ID_4 = 3,

	SERVO_QTY  = 4
};

// enum giving an ID to each group of the Robot
// (when using HC_ServoRobot, ID must be used as an index)
enum groups
{
	group_ID_1 = 0,

	GROUP_QTY  = 1  
};

// Servo Robot
HC_ServoRobot robot;

// In this example a complex synchronized motion is performed for 4 servos when receiving a Trigger
// This complex motion is broken down into different motion_state:
// - 0 : ready to start. Wait for the Trigger
// - 1 : move motors 1, 4
// - 2 : move motors 2, 3
// - 3 : move motors 1, 2, 3 
// - 4 : move motors 1, 3, 4
// - 5 : move all motors back to initial position
// - 6 : motion ended
byte motion_state = 0;

// Initial position
const int initPos_1 = 10;
const int initPos_2 = 70;
const int initPos_3 = 30;
const int initPos_4 = 125;

// Target positions          Pos 1       Pos 2    Pos 3    Pos 4    Init Pos
const int targetPos_1[5]  = {120,        120,     50,      130,     initPos_1};
const int targetPos_2[5]  = {initPos_2,  40,      160,     160,     initPos_2};
const int targetPos_3[5]  = {initPos_3,  110,     150,     150,     initPos_3};
const int targetPos_4[5]  = {20,         20,      20,      100,     initPos_4};

// Motion Times
const float motionTime[5] = {0.5,        0.5,     2.0,     4.0,     2.0};


// -----------------------------------------------------------------------------
// Setup (called once at power on) ---------------------------------------------
// -----------------------------------------------------------------------------

void setup()
{
	// HITIComm initialization
	HC_begin();

	// 1) Robot initialization
	// param: robot ID, servo quantity, group quantity
	robot.init(1, SERVO_QTY, GROUP_QTY);

	// 2) Servo initialization
	// param: motor ID, pin, invert direction, position offset, absolute initial position
	robot.initServo(servo_ID_1, pin_Servo_1, true,  3,  initPos_1);
	robot.initServo(servo_ID_2, pin_Servo_2, false, -1, initPos_2);
	robot.initServo(servo_ID_3, pin_Servo_3, false, 0,  initPos_3);
	robot.initServo(servo_ID_4, pin_Servo_4, true,  5,  initPos_4);           
		
	// 3) Group initialization
	// param: group ID, servo quantity
	robot.initMotorGroup(group_ID_1, 4);

	// 4) Motors assignment
	robot.addMotorToGroup(group_ID_1, servo_ID_1);
	robot.addMotorToGroup(group_ID_1, servo_ID_2);
	robot.addMotorToGroup(group_ID_1, servo_ID_3);
	robot.addMotorToGroup(group_ID_1, servo_ID_4);

	// Motion setpoint mode
	// (also possible to config each group or servo individually)
	robot.setMotionMode(HC_POSITIONMODE_ABSOLUTE, HC_SPEEDMODE_TIME); 
}


// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
	// communicate with HITIPanel (to place at the beginning)
	HC_communicate();


	// trigger motion*********************************************************

	// start motion sequence on Trigger Start
	// (must be ready to start (state 0))
	if((motion_state == 0) && HC_digitalDataRead(dd_Trigger_Start))
		motion_state++;

	// stop motion sequence on Trigger Stop
	if(HC_digitalDataRead(dd_Trigger_Stop))
	{
		robot.stopNow(group_ID_1);

		// stop motion sequence
		motion_state = 6;

		// reset Motion Profile, reset Triggers
		onMotionDone();
	}

	// motion 1 to 5
	else if((1 <= motion_state) && (motion_state <= 5))
	{    
		// Motion parameters setting 
		robot.setMotionSetpoint(servo_ID_1, targetPos_1[motion_state-1], motionTime[motion_state-1]);
		robot.setMotionSetpoint(servo_ID_2, targetPos_2[motion_state-1], motionTime[motion_state-1]);
		robot.setMotionSetpoint(servo_ID_3, targetPos_3[motion_state-1], motionTime[motion_state-1]);
		robot.setMotionSetpoint(servo_ID_4, targetPos_4[motion_state-1], motionTime[motion_state-1]);

		// move now
		robot.moveNow(group_ID_1);

		// once motion has ended for all motors of the group
		if(robot.isDone(group_ID_1)) onMotionDone();
	}


	// servo position monitoring************************************************

	// Motion State
	HC_analogDataWrite(AD_MotionState, motion_state);

	// Position
	HC_analogDataWrite(AD_PositionRead_1, robot.getServo(servo_ID_1).getPosition());
	HC_analogDataWrite(AD_PositionRead_2, robot.getServo(servo_ID_2).getPosition());
	HC_analogDataWrite(AD_PositionRead_3, robot.getServo(servo_ID_3).getPosition());
	HC_analogDataWrite(AD_PositionRead_4, robot.getServo(servo_ID_4).getPosition());
}


// -----------------------------------------------------------------------------
// Function called when Motion Ends --------------------------------------------
// -----------------------------------------------------------------------------

// Use this function to reset the motion trigger and/or state flags
void onMotionDone()
{
	// reset Group state, so that it can start a new move
	robot.reset(group_ID_1);

	// update motion state
	motion_state++;

	if(motion_state >= 6)
	{       
		// reset motion state
		motion_state = 0;

		// reset trigger
		HC_digitalDataWrite(dd_Trigger_Start, false);
		HC_digitalDataWrite(dd_Trigger_Stop,  false);
	}
}
