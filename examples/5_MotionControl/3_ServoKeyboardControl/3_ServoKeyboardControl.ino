/*
HITIComm examples:  Motion Control / 3_ServoKeyboardControl

 This sketch shows how to use HITIPanel software to: 
   => control a servo using computer keyboard

 - Servo                         on pin 8
 - Move -/+ (virtual switches)   on Digital Data 0-1
 - Travel limits -/+             on Digital Data 2-3
 - Position control              on Analog Data 1
 - Continuous speed control      on Analog Data 0

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_Servo.h>

// pins assignment 
const int pin_Servo = 8;

// Analog Data assignment
const int ad_PosCurrent        = 0; // metric (position in °)
const int ad_ConSpeSetpoint    = 1; // setpoint (continuous speed in °/s)
const int ad_ConSpeMax         = 2; // setpoint (continuous speed in °/s)
const int ad_ConSpeTarget      = 3; // metric (continuous speed in °/s)
const int ad_ConSpeCurrent     = 4; // metric (continuous speed in °/s)

// Digital Data assignment
const int dd_MoveNeg           = 0; // virtual switch
const int dd_MovePos           = 1; // virtual switch
const int dd_NegLimitIsReached = 2; // indicator (travel limit)
const int dd_PosLimitIsReached = 3; // indicator (travel limit)

// continuous speed control
float conSpeSetpoint = 10.0; // °/s
float conSpeMax      = 100;  // °/s

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
            false,       // false => normal direction
            3.2,         // position offset = +3.2°
            90.0);       // initial position = 90°

    // set virtual travel limits -/+ (°)
    servo.travelLimits(21.0, 167.5);

    // display initial values of control parameters in HITIPanel
    HC_analogDataWrite(ad_ConSpeSetpoint, conSpeSetpoint);
    HC_analogDataWrite(ad_ConSpeMax,      conSpeMax);
}


void loop()
{
    // communicate with HITIPanel
    HC_communicate();


    // set new control parameters --------------------------------------------

    // read from HITIPanel
    conSpeSetpoint = HC_analogDataRead(ad_ConSpeSetpoint); // speed
    conSpeMax      = HC_analogDataRead(ad_ConSpeMax);

    // set speed setpoint and max value
    servo.continuousSpeed(conSpeSetpoint);
    servo.maxSpeed(conSpeMax);


    // trigger motion --------------------------------------------------------

    // move -/+ on trigger (virtual switches)
    servo.moveNegative(HC_digitalDataRead(dd_MoveNeg));
    servo.movePositive(HC_digitalDataRead(dd_MovePos));


    // display data in HITIPanel ---------------------------------------------

    // control parameters
    HC_analogDataWrite(ad_PosCurrent,    servo.getCurrentPosition());       // position
    HC_analogDataWrite(ad_ConSpeTarget,  servo.getContinuousTargetSpeed()); // speed
    HC_analogDataWrite(ad_ConSpeCurrent, servo.getContinuousCurrentSpeed());

    // has the servo reached a travel limit?
    HC_digitalDataWrite(dd_NegLimitIsReached, servo.isNegLimitReached());
    HC_digitalDataWrite(dd_PosLimitIsReached, servo.isPosLimitReached());
}
