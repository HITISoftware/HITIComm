/*
 Example2_LinearMotionManager
 
 GOAL: Introduction to motion sequence
 
 HARDWARE: - Servo           on pin 8
           - Trigger Start   on HITI Data 0
           - Trigger Stop    on HITI Data 1
           - Motion state    on HITI Data A0
           - Position        on HITI Data A1

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HITICommLinearMotionManager.h>


// -----------------------------------------------------------------------------
// Global Variables ------------------------------------------------------------
// -----------------------------------------------------------------------------

// Pins assignment 
// Servo
const int pin_Servo = 8;

// General-Purpose Analog Data assignment
// motion monitoring
const int AD_MotionState  = 0;
const int AD_PositionRead = 1;

// General-Purpose Digital Data assignment
// motion trigger
const int dd_Trigger_Start = 0;
const int dd_Trigger_Stop  = 1;

// Linear Motion Profile
HC_LinearMotionManager motion_manager = HC_LinearMotionManager();

// In this example a back and forth motion is performed when receiving a Trigger.
// This complex motion is broken down into different state:
// - 0 : ready to start. Wait for the Trigger
// - 1 : move to target
// - 2 : go back to initial position
// - 3 : motion ended
byte motion_state = 0;

// Initial position
const int initPos = 20;


// -----------------------------------------------------------------------------
// Setup (called once at power on) ---------------------------------------------
// -----------------------------------------------------------------------------

void setup()
{
    // HITIComm initialization
    HC_begin();
    
    // Pins mode
    pinMode(pin_Servo, OUTPUT);
    
    // Servo mode
    HC_servoMode(pin_Servo, ATTACH);

    // Motion setpoint mode
    motion_manager.setMotionMode(
            HC_POSITIONMODE_ABSOLUTE,  // position setpoint mode: ABSOLUTE
            HC_SPEEDMODE_TIME);        // speed setpoint mode: TIME
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
		motion_manager.stopNow();
		
		// stop motion sequence
        motion_state = 3;
        
		// reset Motion Profile, reset Triggers
        onMotionDone();
	}
	
    // motion 1
    else if(motion_state == 1)
    {
        // Motion parameters setting 
        motion_manager.setMotionSetpoint(
                ((float) HC_readServo_millidegrees(pin_Servo)) / 1000.0, // absolute initial position (°)
                150,                                                           // position setpoint: absolute (°)
                0.5);                                                          // speed setpoint: motion time (s)

        motion_manager.moveNow(moveMotor);

        // once motion ends, reset Motion Profile and update motion state
        if(motion_manager.isDone()) onMotionDone();
    }

    // motion 2
    else if(motion_state == 2)
    {
        // Motion parameters setting 
        motion_manager.setMotionSetpoint(
                ((float) HC_readServo_millidegrees(pin_Servo)) / 1000.0, // absolute initial position (°)
                initPos,                                                       // position setpoint: absolute (°)
                2);                                                            // speed setpoint: motion time (s)
    
        motion_manager.moveNow(moveMotor);

        // once motion ends, reset Motion Profile, update motion state and reset trigger
        if(motion_manager.isDone()) onMotionDone();
    }


    // servo position monitoring************************************************
    
    // Motion State
    HC_analogDataWrite(AD_MotionState, motion_state);
		
    // Position
    HC_analogDataWrite(
            AD_PositionRead,                                                // Analog Data 1
            ((float) HC_readServo_millidegrees(pin_Servo)) / 1000.0); // read absolute position
}


// -----------------------------------------------------------------------------
// Callback Function to redefine -----------------------------------------------
// -----------------------------------------------------------------------------

// The profile generates a new position at a regular rate (motion cycle).
// Each time a new position is made available, this function is called.
// Use this function to move your motor at this new position.
void moveMotor()
{
    // move Servo to this new position
    HC_writeServo_millidegrees(pin_Servo, (unsigned long)((motion_manager.getNewPosition() * 1000.0)));
}


// Function called when Motion Ends
// => Use this function to reset the motion trigger and/or state flags
void onMotionDone()
{
    // reset motion profile, so that it can start a new move
    motion_manager.reset();
        
    // update motion_state
    motion_state++;
    
    if(motion_state >= 3)
    {       
        // reset motion_state
        motion_state = 0;
        
        // reset trigger
        HC_digitalDataWrite(dd_Trigger_Start, false);
		HC_digitalDataWrite(dd_Trigger_Stop,  false);
    }
}
