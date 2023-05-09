/*
 HITIComm examples:  LinearMotionManager / 1_LinearMotionProfile
 
 This sketch is an introduction to servo control with a Linear Motion Manager.

 - Servo                          on pin 8
 - Start/Stop (virtual switches)  on Digital Data 0-1
 - Servo state                    on Digital Data 2-3
 - Travel limits +/-              on Digital Data 4-5
 - Motion parameters              on Analog Data 0-4
 - Safety parameter               on Analog Data 5
 - Position                       on Analog Data 6

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_MotionManager.h>


// pins assignment
const int pin_Servo = 8;

// Analog Data assignment
const int ad_PositionSetpoint = 0; // control parameters (setpoints)
const int ad_SpeedSetpoint    = 1;
const int ad_CycleTime        = 2;
const int ad_MaxSpeed         = 3; // motion outputs
const int ad_MotionTime       = 4; 
const int ad_Position         = 5;
const int ad_Speed            = 6;

// Digital Data assignment
const int dd_PositionMode        = 0; // control parameters (modes)
const int dd_SpeedMode           = 1;
const int dd_Start               = 2; // virtual switches
const int dd_Stop                = 3;
const int dd_ProfileIsReady      = 4; // manager state
const int dd_ProfileIsGenerating = 5;
const int dd_ProfileIsDone       = 6;
const int dd_NegLimitIsReached   = 7; // virtual travel limits
const int dd_PosLimitIsReached   = 8;

// control parameters
bool  positionMode     = LOW; // LOW: absolute   / HIGH: relative
bool  speedMode        = LOW; // LOW: speed(°/s) / HIGH: time(s)
float positionSetpoint = 30;  // 30° absolute
float speedSetpoint    = 10;  // 10°/s
float cycleTime        = 10;  // in ms (min value 10ms)

// safety parameter
float maxSpeed = 100;  // °/s

// Motion Manager
HC_MotionManager manager = HC_MotionManager();


// run once at power on
void setup()
{
    // initialize library
    HC_begin();
    

    // pins mode
    pinMode(pin_Servo, OUTPUT);

    // servo mode
    HC_servoMode(pin_Servo, ATTACH);

    // set position limits
    manager.setTravelLimits(
            10.0,   // limit - (°)
            170.0); // limit + (°)

    // send initial control parameters to HITIPanel
    HC_digitalDataWrite(dd_PositionMode, positionMode);
    HC_digitalDataWrite(dd_SpeedMode, speedMode);
    HC_analogDataWrite(ad_PositionSetpoint, positionSetpoint);
    HC_analogDataWrite(ad_SpeedSetpoint,    speedSetpoint);
    HC_analogDataWrite(ad_CycleTime,        cycleTime);
	HC_analogDataWrite(ad_MaxSpeed,         maxSpeed);
}


// run repeatedly after setup()
void loop()
{
    // communicate with HITIPanel
    HC_communicate();
    

	// set new parameters ----------------------------------------------------

	// read from HITIPanel
	positionMode     = HC_digitalDataRead(dd_PositionMode);
	speedMode        = HC_digitalDataRead(dd_SpeedMode);
	positionSetpoint = HC_analogDataRead(ad_PositionSetpoint);
	speedSetpoint    = HC_analogDataRead(ad_SpeedSetpoint);
    cycleTime        = HC_analogDataRead(ad_CycleTime);
	maxSpeed         = HC_analogDataRead(ad_MaxSpeed);
	
    // set parameters
    if (positionMode == HIGH)
        manager.relativePos();
    else
        manager.absolutePos();

    if (speedMode == HIGH)
        manager.timeMode();
    else
        manager.speedMode();

  	manager.setMotionSetpoints(
            ((float) HC_readServo(pin_Servo)) / 1000.0, // absolute initial position (°)
            positionSetpoint,                           // position setpoint: absolute (°)
            speedSetpoint);                             // speed setpoint: speed (°/s)
    manager.setCycleTime(cycleTime);                    // cycle time (min value is 10ms)
	manager.setMaxSpeed(maxSpeed);                      // max speed (°/s)
	

	// start/stop motion -----------------------------------------------------
    
	// move/stop on trigger (START/STOP virtual switches)
    manager.moveOnTrigger(
            HC_digitalDataRead(dd_Start), // virtual switch
            moveMotor);                   // callback function to define by user

    manager.stopOnTrigger(HC_digitalDataRead(dd_Stop));

	// reset triggers
    HC_digitalDataWrite(dd_Start, false);
    HC_digitalDataWrite(dd_Stop,  false);

    // servo position monitoring************************************************
    
    // motion outputs
    HC_analogDataWrite(ad_CycleTime, manager.getCycleTime());
    HC_analogDataWrite(ad_MotionTime, manager.getMotionTime());
    HC_analogDataWrite(ad_Position, ((float)HC_readServo(pin_Servo)) / 1000.0);
    HC_analogDataWrite(ad_Speed, manager.getMotionSpeed());
    
    // Motion Profile State
    HC_digitalDataWrite(dd_ProfileIsReady,      manager.isReady());
    HC_digitalDataWrite(dd_ProfileIsGenerating, manager.isGenerating());
    HC_digitalDataWrite(dd_ProfileIsDone,       manager.isDone());

	// Check if Negative/Positive limits has been reached
    manager.checkTravelLimits(
            ((float) HC_readServo(pin_Servo)) / 1000.0, // read absolute position
            0.1);                                       // sensitivity (°)

    HC_digitalDataWrite(dd_NegLimitIsReached, manager.isNegLimitReached());
    HC_digitalDataWrite(dd_PosLimitIsReached, manager.isPosLimitReached());
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
    HC_writeServo(pin_Servo, (unsigned long) ((manager.getNewPosition()*1000.0)));
}
