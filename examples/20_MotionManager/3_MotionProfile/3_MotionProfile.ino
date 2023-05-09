/*
 Example3_LinearMotionManager 

 Version 1.2.1
 
 GOAL: Motion control using computer keyboard

 HARDWARE: - Servo                 on pin 8
           - Trigger Go Forward    on HITI Data 0
           - Trigger Go Backward   on HITI Data 1
           - Position limits +/-   on HITI Data 2-3
           - Motion parameters     on HITI Data A0-A1
           - Position              on HITI Data A5

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

// HITI Analog Data assignment
// motion control
const int AD_ContinuousCycleTime     = 0; 
const int AD_ContinuousSpeedSetpoint = 1; 
// motion monitoring
const int AD_PositionRead            = 2;

// HITI Digital Data assignment
// motion triggers
const int DD_GoForward           = 0;
const int DD_GoBackward          = 1;
// motion monitoring
const int DD_NegLimitIsReached   = 2;
const int DD_PosLimitIsReached   = 3;

// Linear Motion Profile
HC_LinearMotionManager motion_manager = HC_LinearMotionManager();

// Continuous Motion control
float motion_continuousSpeedSetpoint = 100; // speed setpoint (°/s)

// Continuous Motion cycle
// (Motions are performed in small increments. 1 increment is done per motion cycle)
float motion_continuousCycleTime = 10;  // in ms (min value 10ms)


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
	
	// set Min and Max position limits
	motion_manager.setPositionLimits(
            10.0,   // min absolute position (°)
            170.0); // max absolute position (°)

    // send motion control parameters (Analog Data 0 to 1)
    HC_analogDataWrite(AD_ContinuousCycleTime,     motion_continuousCycleTime);
    HC_analogDataWrite(AD_ContinuousSpeedSetpoint, motion_continuousSpeedSetpoint);
}



// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
    // communicate with HITIPanel (to place at the beginning)
    HC_communicate();
    

    // set new motion parameters**********************************************
    
    // receive motion control parameters (Analog Data 0 to 1)
    motion_continuousCycleTime     = HC_analogDataRead(AD_ContinuousCycleTime);
    motion_continuousSpeedSetpoint = HC_analogDataRead(AD_ContinuousSpeedSetpoint);
	
  	// set motion parameters 
    motion_manager.setContinuousSpeedSetpoint(motion_continuousSpeedSetpoint); // continuous speed setpoint (°/s)
    motion_manager.setContinuousCycleTime(motion_continuousCycleTime);         // continuous cycle time (min value is 10ms)
	
	
    // trigger motion*********************************************************

    // move backward (if querried)
    motion_manager.moveContinuously(
            HC_digitalDataRead(DD_GoBackward),                             // Command : Go Backward
            false,                                                         // negative direction
            ((float) HC_readServo_millidegrees(pin_Servo)) / 1000.0, // absolute initial position (°)
            moveMotor);                                                    // callback function to define by user
					
    // move forward	(if querried)				
    motion_manager.moveContinuously(
            HC_digitalDataRead(DD_GoForward),                              // Command : Go Forward
            true,                                                          // positive direction
            ((float) HC_readServo_millidegrees(pin_Servo)) / 1000.0, // absolute initial position (°)
            moveMotor);                                                    // callback function to define by user
		
    // when motion ends, reset profile and trigger
    if(motion_manager.isDone()) onMotionDone();


    // servo position monitoring************************************************
    
    // Cycle Time (min value is 10ms)
    HC_analogDataWrite(AD_ContinuousCycleTime, motion_manager.getContinuousCycleTime());
	
    // Position
    HC_analogDataWrite(AD_PositionRead, ((float) HC_readServo_millidegrees(pin_Servo)) / 1000.0);
	
    // Check if Negative/Positive limits has been reached
    motion_manager.checkLimits(
            ((float) HC_readServo_millidegrees(pin_Servo)) / 1000.0, // read absolute position
            0.1);                                                          // sensitivity (°)
			
    HC_digitalDataWrite(DD_NegLimitIsReached, motion_manager.isNegativeLimitReached());
    HC_digitalDataWrite(DD_PosLimitIsReached, motion_manager.isPositiveLimitReached());
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
}
