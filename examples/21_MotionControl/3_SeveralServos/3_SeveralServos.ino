/*
HITIComm examples:  Motion Control / 3_ServoControl

 This sketch shows how to perform SYNCHRONIZED complex moves of 4 Servos using HITI Servos.
 
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
#include <HITICommServo.h>


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
const int dd_Trigger_Start = 0;
const int dd_Trigger_Stop  = 1;

// HITIComm Servo
HC_Servo servo_1;
HC_Servo servo_2;
HC_Servo servo_3;
HC_Servo servo_4;

// Servo objects created by user
Servo servo3;
Servo servo4;

// Motion state
byte motion_state = 0;

// Initial position
const int initPos_1 = 10;
const int initPos_2 = 70;
const int initPos_3 = 30;
const int initPos_4 = 125;


// -----------------------------------------------------------------------------
// Setup (called once at power on) ---------------------------------------------
// -----------------------------------------------------------------------------

void setup()
{
	// HITIComm initialization
	HC_begin();

	// HC_Servo initialization
	// param: custom id, pin, invert direction, position offset, absolute initial position (, optional : servo)
	servo_1.init(0, pin_Servo_1, true,  3,  initPos_1         );  // Servo 1 is auto-created
	servo_2.init(1, pin_Servo_2, false, -1, initPos_2         );  // Servo 2 is auto-created
	servo_3.init(2, pin_Servo_3, false, 0,  initPos_3, &servo3);  // Servo 3 is supplied by user
	servo_4.init(3, pin_Servo_4, true,  5,  initPos_4, &servo4);  // Servo 4 is supplied by user

	// Motion setpoint mode
	servo_1.setMotionMode(HC_POSITIONMODE_ABSOLUTE, HC_SPEEDMODE_TIME);
	servo_2.setMotionMode(HC_POSITIONMODE_ABSOLUTE, HC_SPEEDMODE_TIME);      
	servo_3.setMotionMode(HC_POSITIONMODE_ABSOLUTE, HC_SPEEDMODE_TIME); 
	servo_4.setMotionMode(HC_POSITIONMODE_ABSOLUTE, HC_SPEEDMODE_TIME);         
}


// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
	// communicate with HITIPanel (to place at the beginning)
	HC_communicate();


	// trigger motion*****************************************************

	// start motion sequence on Trigger Start
	// (must be ready to start (state 0))
	if((motion_state == 0) && HC_digitalDataRead(dd_Trigger_Start))
		motion_state++;

	// stop motion sequence on Trigger Stop
	if(HC_digitalDataRead(dd_Trigger_Stop))
	{
		servo_1.stopNow();
		servo_2.stopNow();
		servo_3.stopNow();
		servo_4.stopNow();

		// stop motion sequence
		motion_state = 6;

		// reset Motion Profile, reset Triggers
		onMotionDone();
	}

	// motion 1
	else if(motion_state == 1)
	{
		// Motion parameters setting 
		servo_1.setMotionSetpoint(120, 0.5); // absolute position setpoint 150°, motion performed in 0.5s
		servo_4.setMotionSetpoint(20,  0.5); // absolute position setpoint 10°, motion performed in 0.5s
		   
		// move now
		servo_1.moveNow();
		servo_4.moveNow();

		// once motion has ended for all servos
		if(servo_1.isDone() && servo_4.isDone()) onMotionDone();
	}

	// motion 2
	else if(motion_state == 2)
	{
		// Motion parameters setting 
		servo_2.setMotionSetpoint(40,  0.5); // absolute position setpoint 150°, motion performed in 0.5s
		servo_3.setMotionSetpoint(110, 0.5); // absolute position setpoint 10°, motion performed in 0.5s
		   
		// move now
		servo_2.moveNow();
		servo_3.moveNow();

		// once motion has ended for all servos
		if(servo_2.isDone() && servo_3.isDone()) onMotionDone();
	}    

	// motion 3
	else if(motion_state == 3)
	{
		// Motion parameters setting 
		servo_1.setMotionSetpoint(50,  2); // absolute position setpoint 80°, motion performed in 2s
		servo_2.setMotionSetpoint(160, 2); // absolute position setpoint 160°, motion performed in 2s
		servo_3.setMotionSetpoint(150, 2); // absolute position setpoint 160°, motion performed in 2s
		   
		// move now
		servo_1.moveNow();
		servo_2.moveNow();
		servo_3.moveNow();

		// once motion has ended for all servos
		if(servo_1.isDone() && servo_2.isDone() && servo_3.isDone()) onMotionDone();
	}

	// motion 4
	else if(motion_state == 4)
	{
		// Motion parameters setting 
		servo_1.setMotionSetpoint(130, 4); // absolute position setpoint 80°, motion performed in 4s
		servo_3.setMotionSetpoint(10,  4); // absolute position setpoint 160°, motion performed in 4s
		servo_4.setMotionSetpoint(100, 4); // absolute position setpoint 160°, motion performed in 4s
		   
		// move now
		servo_1.moveNow();
		servo_3.moveNow();
		servo_4.moveNow();

		// once motion has ended for all servos
		if(servo_1.isDone() && servo_3.isDone() && servo_4.isDone()) onMotionDone();
	}

	// motion 5
	else if(motion_state == 5)
	{
		// Motion parameters setting 
		servo_1.setMotionSetpoint(initPos_1, 0.5); // absolute init position, motion performed in 0.5s
		servo_2.setMotionSetpoint(initPos_2, 0.5); // absolute init position, motion performed in 0.5s
		servo_3.setMotionSetpoint(initPos_3, 0.5); // absolute init position, motion performed in 0.5s
		servo_4.setMotionSetpoint(initPos_4, 0.5); // absolute init position, motion performed in 0.5s

		// move now
		servo_1.moveNow();
		servo_2.moveNow();
		servo_3.moveNow();
		servo_4.moveNow();		

		// once motion has ended for all servos
		if(servo_1.isDone() && servo_2.isDone() && servo_3.isDone() && servo_4.isDone()) onMotionDone();
	}


	// Motion monitoring******************************************************

	// Motion State
	HC_analogDataWrite(AD_MotionState, motion_state);

	// Positions
	HC_analogDataWrite(AD_PositionRead_1, servo_1.getPosition());
	HC_analogDataWrite(AD_PositionRead_2, servo_2.getPosition());
	HC_analogDataWrite(AD_PositionRead_3, servo_3.getPosition());
	HC_analogDataWrite(AD_PositionRead_4, servo_4.getPosition());		
}


// -----------------------------------------------------------------------------
// Function called when Motion Ends --------------------------------------------
// -----------------------------------------------------------------------------

// Use this function to reset the motion trigger and/or state flags
void onMotionDone()
{
	// reset servos state, so that they can start a new move
	servo_1.reset();
	servo_2.reset();
	servo_3.reset();
	servo_4.reset();
			
	// update motion state
	motion_state++;

	if(motion_state >= 6)
	{       
		// reset motion state
		motion_state = 0;
			
		// reset trigger
		HC_digitalDataWrite(dd_Trigger_Start, false);
		HC_digitalDataWrite(dd_Trigger_Stop, false);
	}
}
