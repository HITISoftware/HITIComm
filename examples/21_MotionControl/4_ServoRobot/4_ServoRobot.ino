/*
HITIComm examples:  MotionControl / 4_ServoRobot

 This sketch shows how to use a HITI Servo Robot to perform:
   => a synchronized motion sequence of 2 Groups of 2 Servos

 - Servos                         on pin 8-11

 - Start/Stop (virtual switches)  on Digital Data 0-1
 - Robot state                    on Digital Data 2-4

 - Sequence step                  on Analog Data 0
 - Current position (Servo 1-4)   on Analog Data 1-4
 
 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_ServoRobot.h>


// Pins assignment 
const int pin_Servo_1 = 8;  // servos
const int pin_Servo_2 = 9;
const int pin_Servo_3 = 10;
const int pin_Servo_4 = 11;

// Analog Data assignment
const int ad_Step       = 0; // sequence step
const int ad_Position_1 = 1; // servos positions
const int ad_Position_2 = 2;
const int ad_Position_3 = 3;
const int ad_Position_4 = 4;

// Digital Data assignment
const int dd_Start          = 0; // virtual switches
const int dd_Stop           = 1;
const int dd_RobotIsReady   = 2; // robot state
const int dd_RobotIsMoving  = 3;
const int dd_RobotIsDone    = 4;

// motion sequence in 5 steps: 0 (Ready), 1-4 (Motions)
int step = 0;

const int initPos_1 = 10;
const int initPos_2 = 160;
const int initPos_3 = 80;
const int initPos_4 = 120;

// HITI Servo Robot
HC_ServoRobot robot;


// run once at power on
void setup()
{
	// initialize library
	HC_begin();

	// 1) initialize HITI Servo Robot
	// param: robot ID, servo quantity, group quantity
	robot.init(1, 4, 3);

	// 2) initialize HITI Servos
	// param: motor ID, pin, invert direction, position offset, absolute initial position
	robot.initServo(0, pin_Servo_0, true,  3,  initPos_0);
	robot.initServo(1, pin_Servo_1, false, -1, initPos_1);
	robot.initServo(2, pin_Servo_2, false, 0,  initPos_2);
	robot.initServo(3, pin_Servo_3, true,  5,  initPos_3);           
		
	// 3) initialize Groups
	// param: group ID, servo quantity
	robot.initMotorGroup(0, 2);
	robot.initMotorGroup(1, 2);

	// 4) Motors assignment to Groups
	// param: group ID, servo ID
	robot.addMotorToGroup(0, 0); // Group 0: Servos 0 and 1
	robot.addMotorToGroup(0, 1);
	robot.addMotorToGroup(1, 2); // Group 1: Servos 2 and 3
	robot.addMotorToGroup(1, 3);

	// set modes for all groups (also possible to configure each group separately)
	robot.setMotionMode(HC_POSITIONMODE_ABSOLUTE, HC_SPEEDMODE_TIME); 
}


// run repeatedly after setup()
void loop()
{
	// communicate with HITIPanel
	HC_communicate();


	// start/stop motion sequence --------------------------------------------

	// step 0: ready to start sequence with the START virtual switch
	if((step == 0) && HC_digitalDataRead(dd_Start))
	{
		step = 1; // start sequence
	}

	// at any step: stop sequence with the STOP virtual switch
	if(HC_digitalDataRead(dd_Stop))
	{
		// stop groups 0 and 1
		robot.stopNow(0);
		robot.stopNow(1);

		// set step
		step = 5;

		// reset Motion Profile, reset Triggers
		onMotionDone();
	}

	// step 1
	else if(step == 1)
	{
		// Motion parameters setting 
		robot.setMotionSetpoint(0, 150, 0.5); // target position 150°, motion time 0.5s
		robot.setMotionSetpoint(1, 10,  0.5); // absolute position setpoint 10°, motion performed in 0.5s

		// move group 0
		robot.moveNow(0);

		// once motion has ended for all motors of group 0
		if(robot.isDone(0))
		{
			onMotionDone();
		}
	}

	// step 2
	else if(step == 2)
	{
		// Motion parameters setting 
		robot.setMotionSetpoint(2, 150, 0.5); // absolute position setpoint 150°, motion performed in 0.5s
		robot.setMotionSetpoint(3, 10,  0.5); // absolute position setpoint 10°, motion performed in 0.5s

		// move group 1
		robot.moveNow(1);

		// once motion has ended for all motors of group 1
		if(robot.isDone(1))
		{
			onMotionDone();
		}
	}    

	// step 3
	else if(step == 3)
	{
		// Motion parameters setting 
		robot.setMotionSetpoint(0, initPos_1, 2); // absolute position setpoint 80°, motion performed in 2s
		robot.setMotionSetpoint(1, initPos_2, 2); // absolute position setpoint 160°, motion performed in 2s        

		// move group 0
		robot.moveNow(0);

		// once motion has ended for all motors of group 0
		if(robot.isDone(0))
		{
			onMotionDone();
		}
	}

	// step 4
	else if(step == 4)
	{
		// Motion parameters setting 
		robot.setMotionSetpoint(2, initPos_3, 2); // absolute position setpoint 80°, motion performed in 2s
		robot.setMotionSetpoint(3, initPos_4, 2); // absolute position setpoint 160°, motion performed in 2s 

		// move group 1
		robot.moveNow(1);

		// once motion has ended for all motors of group 1
		if(robot.isDone(1))
		{
			onMotionDone();
		}
	}

	// immediately deactivate the Virtual Switches
	HC_digitalDataWrite(dd_Start, false);
	HC_digitalDataWrite(dd_Stop, false);


	// servo position monitoring************************************************

	// Sequence step
	HC_analogDataWrite(ad_Step, step);

	// Position
	HC_analogDataWrite(ad_Position_1, robot.getServo(servo_ID_1).getPosition());
	HC_analogDataWrite(ad_Position_2, robot.getServo(servo_ID_2).getPosition());
	HC_analogDataWrite(ad_Position_3, robot.getServo(servo_ID_3).getPosition());
	HC_analogDataWrite(AD_PositionRead_4, robot.getServo(servo_ID_4).getPosition());

	// send Robot State
	HC_digitalDataWrite(dd_RobotIsReady,  robot.isReady());
	HC_digitalDataWrite(dd_RobotIsMoving, robot.isMoving());
	HC_digitalDataWrite(dd_RobotIsDone,   robot.isDone());
}


// called each time a motion ends
void onMotionDone()
{
	// reset Groups, so that it can start a new move
	robot.reset(0);
	robot.reset(1);
			
	// for steps 0-3
	if (step < 4)
	{
		step++;   // go to next step
	}
	// for step 4
	else
	{
		step = 0; // go to step 0
	}
}
