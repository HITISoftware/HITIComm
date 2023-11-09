/*
 HITIComm examples:  DFRobot / 1_DFRobot_Devastator

 This sketch shows how to use HITIPanel to:
   => control a DFRobot Devastator vehicule (Romeo BLE board) using computer keyboard

 - 2 DC Motors:
    * Direction Control     on pin 4,7
    * Speed Control         on pin 5,6
    * Trigger Go Forward    on Digital Data 0
    * Trigger Go Backward   on Digital Data 1
    * Trigger Turn Left     on Digital Data 2
    * Trigger Turn Right    on Digital Data 3
    * Speed                 on Analog Data 0

 - 1 Pan & Tilt (Pan = ROB0114 Base Rotate Kit, Tilt = FIT0004 Pan & Tilt Kit)
    * Servo 1 (Pan)         on pin 8
    * Servo 2 (Tilt)        on pin 9
    * Trigger Go Forward    on Digital Data 4,6
    * Trigger Go Backward   on Digital Data 5,7
    * Speed                 on Analog Data 1
    * Position              on Analog Data 2,3

 - 1 Distance sensor URM37
    * Raw value             on pin A0
    * Converted in cm       on Analog Data 4

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_Servo.h>



// -----------------------------------------------------------------------------
// Global Variables ------------------------------------------------------------
// -----------------------------------------------------------------------------

// Pins assignment 
// - 2 DC Motors
const int pin_MotorR_DirectionControl = 4;
const int pin_MotorL_DirectionControl = 7;
const int pin_MotorR_SpeedControl     = 5;
const int pin_MotorL_SpeedControl     = 6;
// 1 Pan & Tilt
const int pin_ServoP                  = 8;
const int pin_ServoT                  = 9;
// 1 Distance sensor
const int pin_distanceSensor          = A0;

// Analog Data assignment
const int ad_Motors_Speed             = 0; // Setpoint (no unit)
const int ad_Servos_Speed             = 1; // Setpoint (in °/s)
const int ad_ServoP_Position          = 2; // Metric (in °)
const int ad_ServoT_Position          = 3; // Metric (in °)
const int ad_distance                 = 4; // Metric (in cm)

// Digital Data assignment
// motion triggers
const int dd_GoForward_Motors         = 0; // Virtual Switch
const int dd_GoBackward_Motors        = 1; // Virtual Switch
const int dd_TurnLeft_Motors          = 2; // Virtual Switch
const int dd_TurnRight_Motors         = 3; // Virtual Switch
const int dd_GoForward_ServoP         = 4; // Virtual Switch
const int dd_GoBackward_ServoP        = 5; // Virtual Switch
const int dd_GoForward_ServoT         = 6; // Virtual Switch
const int dd_GoBackward_ServoT        = 7; // Virtual Switch
const int dd_NegLimit_ServoP          = 8;  // Indicator
const int dd_PosLimit_ServoP          = 9;  // Indicator
const int dd_NegLimit_ServoT          = 10; // Indicator
const int dd_PosLimit_ServoT          = 11; // Indicator

// HITIComm Servo
HC_Servo servoP;
HC_Servo servoT;

// Continuous Motion control
byte motors_speedSetpoint = 100; // speed setpoint (no unit)
float servos_speedSetpoint = 20;  // speed setpoint (°/s)

// Initial position
const int initPos_servoP = 90;
const int initPos_servoT = 90;

// DC motors threshold speed (Min Speed Control value required to start moving DC motors)
const int motors_threshold = 45;



// -----------------------------------------------------------------------------
// Setup (called once at power on) ---------------------------------------------
// -----------------------------------------------------------------------------

void setup()
{
	// HITIComm initialization
	HC_begin(115200);

	// Pin mode
	pinMode(pin_MotorR_DirectionControl, OUTPUT);
	pinMode(pin_MotorL_DirectionControl, OUTPUT);
	pinMode(pin_MotorR_SpeedControl,     OUTPUT);
	pinMode(pin_MotorL_SpeedControl,     OUTPUT);

	// Output mode
	HC_outputType(pin_MotorR_DirectionControl, LOW);  // DO
	HC_outputType(pin_MotorL_DirectionControl, LOW);  // DO
	HC_outputType(pin_MotorR_SpeedControl,     HIGH); // PWM
	HC_outputType(pin_MotorL_SpeedControl,     HIGH); // PWM

	// HC_Servo initialization
	// param: custom id, pin, invert direction, position offset, absolute initial position
	servoP.init(0, pin_ServoP, false, 0, initPos_servoP);
	servoT.init(1, pin_ServoT, false, 0, initPos_servoT);
			
	// set Min and Max absolute position limits (min, max) (°)
	servoP.travelLimits(0.0,  180.0);
	servoT.travelLimits(52.0, 175.0);

	// display initial values of motion control parameters in HITIPanel
	HC_analogDataWrite(ad_Motors_Speed, motors_speedSetpoint);
	HC_analogDataWrite(ad_Servos_Speed, servos_speedSetpoint);
}



// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
	// communicate with HITIPanel (to place at the beginning)
	HC_communicate();

	
	// receive and set new motion parameters**********************************

	// continuous speed setpoint
	motors_speedSetpoint = (char) HC_analogDataRead_setpoint(ad_Motors_Speed, 0, 255);
	servos_speedSetpoint = HC_analogDataRead_setpoint(ad_Servos_Speed, 0, 100.0);

	// set Servos speed
	servoP.continuousSpeed(servos_speedSetpoint);
	servoT.continuousSpeed(servos_speedSetpoint);


	// trigger motion*********************************************************

	if      (HC_digitalDataRead(dd_GoForward_Motors))  go_forward (motors_speedSetpoint);
	else if (HC_digitalDataRead(dd_GoBackward_Motors)) go_backward(motors_speedSetpoint);
	else if (HC_digitalDataRead(dd_TurnLeft_Motors))   turn_left  (motors_speedSetpoint);
	else if (HC_digitalDataRead(dd_TurnRight_Motors))  turn_right (motors_speedSetpoint);
	else                                               stop();

	// move forward or backward (if querried)
	servoP.movePositive(HC_digitalDataRead(dd_GoForward_ServoP));
	servoP.moveNegative(HC_digitalDataRead(dd_GoBackward_ServoP));
	servoT.movePositive(HC_digitalDataRead(dd_GoForward_ServoT));
	servoT.moveNegative(HC_digitalDataRead(dd_GoBackward_ServoT));


	// limited speed monitoring **********************************************

	HC_analogDataWrite(ad_Motors_Speed, motors_speedSetpoint);
	HC_analogDataWrite(ad_Servos_Speed, servos_speedSetpoint);


	// Servo position monitoring *********************************************

  HC_digitalDataWrite(dd_NegLimit_ServoP, servoP.isNegLimitReached());
  HC_digitalDataWrite(dd_PosLimit_ServoP, servoP.isPosLimitReached());
  HC_digitalDataWrite(dd_NegLimit_ServoT, servoT.isNegLimitReached());
  HC_digitalDataWrite(dd_PosLimit_ServoT, servoT.isPosLimitReached());

	HC_analogDataWrite(ad_ServoP_Position, servoP.getCurrentPosition());
	HC_analogDataWrite(ad_ServoT_Position, servoT.getCurrentPosition());


	// Distance Sensor (in cm)************************************************

	// read sensor
	int sensorValue = analogRead(pin_distanceSensor);

	// convert to cm (4.125mV /cm)
	//   => (sensorValue * 5000 / 1024 )   ->   value in V
	//   => value in V / 4.125             ->   value in cm
	//       => sensorValue * 1.1
	HC_analogDataWrite(ad_distance, (1.1*((double)sensorValue)));
}



// -----------------------------------------------------------------------------
// Methods ---------------------------------------------------------------------
// -----------------------------------------------------------------------------

void stop()                   
{
  digitalWrite(pin_MotorR_SpeedControl,     0);
  digitalWrite(pin_MotorR_DirectionControl, LOW);
  digitalWrite(pin_MotorL_SpeedControl,     0);
  digitalWrite(pin_MotorL_DirectionControl, LOW);
}

void go_forward(byte speedL, byte speedR)
{
  analogWrite (pin_MotorR_SpeedControl,     map(speedR, 0, 255, motors_threshold, 255));
  digitalWrite(pin_MotorR_DirectionControl, HIGH);
  analogWrite (pin_MotorL_SpeedControl,     map(speedL, 0, 255, motors_threshold, 255));
  digitalWrite(pin_MotorL_DirectionControl, HIGH);
}

void go_forward(byte a)
{
  go_forward(a, a);
}

void go_backward(byte speedL, byte speedR)
{
  analogWrite (pin_MotorR_SpeedControl,     map(speedR, 0, 255, motors_threshold, 255));
  digitalWrite(pin_MotorR_DirectionControl, LOW);
  analogWrite (pin_MotorL_SpeedControl,     map(speedL, 0, 255, motors_threshold, 255));
  digitalWrite(pin_MotorL_DirectionControl, LOW);
}

void go_backward(byte speed)
{
	go_backward(speed, speed);
}

void turn_left(byte speedL, byte speedR)
{
  analogWrite (pin_MotorR_SpeedControl,     map(speedR, 0, 255, motors_threshold, 255));
  digitalWrite(pin_MotorR_DirectionControl, HIGH);
  analogWrite (pin_MotorL_SpeedControl,     map(speedL, 0, 255, motors_threshold, 255));
  digitalWrite(pin_MotorL_DirectionControl, LOW);
}

void turn_left(byte speed)
{
	turn_left(speed, speed);
}

void turn_right(byte speedL, byte speedR)
{
  analogWrite (pin_MotorR_SpeedControl,     map(speedR, 0, 255, motors_threshold, 255));
  digitalWrite(pin_MotorR_DirectionControl, LOW);
  analogWrite (pin_MotorL_SpeedControl,     map(speedL, 0, 255, motors_threshold, 255));
  digitalWrite(pin_MotorL_DirectionControl, HIGH);
}

void turn_right(byte speed)
{
  turn_right(speed, speed);
}
