/*
HITIComm examples:  Velleman / 1_ALLBOT_VR408

 This sketch shows how to use HITIPanel software to:
   => control an ALLBOT VR408 4-legged robot using computer keyboard

 - 8 Servos        pins assignment depends on boards
 - Buzzer          on pin 13
 - 16 Commands     on Digital Data 0-15
 - Speed control   on Analog Data 0
 - Positions       on Analog Data 2-9

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_Servo.h>
#include <HC_MotorGroup.h>



// -----------------------------------------------------------------------------
// Global Variables ------------------------------------------------------------
// -----------------------------------------------------------------------------

// Pins assignment ***********************************************************

// Buzzer (mounted on VRSSM board)
const int pin_Sounder = 13;


// Digital Data assignment ***************************************************

// enum giving an ID to each motion sequence of the Robot.
//   This ID is also the index of the Digital Data used to trigger the motion sequence.
enum motionSequence
{
	// walk (Forward, Backward, Left, Right)
	walk_F      = 0,
	walk_B      = 1,
	walk_L      = 2,
	walk_R      = 3,

	// turn (Left, Right)
	turn_L      = 4,
	turn_R      = 5,

	// lean (Forward, Backward, Left, Right)
	lean_F      = 6,
	lean_B      = 7,
	lean_L      = 8,
	lean_R      = 9,

	// look (Left, Right)
	look_L      = 10,
	look_R      = 11,

	// wave front (Left, Right)
	waveFront_L = 12,
	waveFront_R = 13,    

	// wave rear (Left, Right)
	waveRear_L  = 14,
	waveRear_R  = 15, 
	
	// emotions
	scared      = 16,
	chirping    = 17,

	// initialization
	initMotion  = 18,

	MOTION_QTY  = 19
};


// Analog Data assignment ****************************************************

// motion speed
byte AD_motion_time = 0;

// servo position monitoring
byte AD_Servo_HipFront_L     = 2;
byte AD_Servo_HipFront_R     = 3;
byte AD_Servo_HipRear_L      = 4;
byte AD_Servo_HipRear_R      = 5;
byte AD_Servo_KneeFront_L    = 6;
byte AD_Servo_KneeFront_R    = 7;
byte AD_Servo_KneeRear_L     = 8;
byte AD_Servo_KneeRear_R     = 9;


// HITI Servos ***************************************************************

// enum giving an ID to each servo of the Robot
//   (when using HC_ServoRobot, ID must be used as an index)
enum servos
{
  hipFront_L  = 0,
  hipFront_R  = 1,
  hipRear_L   = 2,
  hipRear_R   = 3,

  kneeFront_L = 4,
  kneeFront_R = 5,
  kneeRear_L  = 6,
  kneeRear_R  = 7,

  SERVO_QTY   = 8
};

HC_Servo servo_hipFront_L;
HC_Servo servo_hipFront_R;
HC_Servo servo_hipRear_L;
HC_Servo servo_hipRear_R;

HC_Servo servo_kneeFront_L;
HC_Servo servo_kneeFront_R;
HC_Servo servo_kneeRear_L;
HC_Servo servo_kneeRear_R;


// HITI Motor Group **********************************************************

HC_MotorGroup group;

    
// Servos setting ************************************************************

// setting mode
//   - to set manually before uploading program. Also possible to use a Digital Data
//     to activate this mode
//   - true: angle offset setting  /  false: normal operation
bool settingModeIsActivated = false; 

// init position for all servos
int initPosition = 90; // °

// init is complete (flag)
bool isStarted = false;


// Motion control ************************************************************

// There are 19 robot moves: walk, look...
//   Each moves is made of a sequence of several moves. An associated motion_state number
//   indicates where we are in the motion sequence
byte motion_state[MOTION_QTY] = {0};	// array of motion sequence states: 

// motion time (s)
float motion_time = 1;

// HITI Timer for pausing motion (lean and look) 
HC_Timer motion_timer;       
  
// Cyclic For Loops index
byte loop_index = 0;


// Chirp control *************************************************************

// Chirp building state
//   2 chirp building sequences are used:
//   - 0 : create an individual sound
//   - 1 : sing using individual sounds
byte chirp_state[2] = {0};

// HITI Timer for chirping
HC_Timer chirp_timer;

// Cyclic For Loops index
byte chirp_loop_index = 0;



// -----------------------------------------------------------------------------
// Setup (called once at power on) ---------------------------------------------
// -----------------------------------------------------------------------------

void setup()
{
	// initialize library
	HC_begin();

	// pin mode
	// Buzzer
	pinMode(pin_Sounder, OUTPUT);

	// set initial positions **************************************************
	if(settingModeIsActivated)
		initPosition = 90; // position offset setting
	else
		initPosition = 45; // normal operation
      
	// initialize HITI Servos *************************************************
	// param: custom ID, pin, invert direction, position offset, absolute init position
	// for Mega Boards
	#if HC_VARIANT == HC_VARIANT_MEGA 
		servo_hipFront_L.init(hipFront_L,  A1, false, 0, initPosition);	// A1
		servo_hipFront_R.init(hipFront_R,  A0, true,  0, initPosition);	// A0
		servo_hipRear_L .init(hipRear_L,   30, true,  0, initPosition);	// SV 9
		servo_hipRear_R .init(hipRear_R,   25, false, 0, initPosition);	// SV 4

		servo_kneeFront_L.init(kneeFront_L, 32, true,  0, initPosition);	// SV 11
		servo_kneeFront_R.init(kneeFront_R, 23, false, 0, initPosition);	// SV 2
		servo_kneeRear_L .init(kneeRear_L,  31, true,  0, initPosition);	// SV 10
		servo_kneeRear_R .init(kneeRear_R,  24, false, 0, initPosition);	// SV 3
	// for Other Boards
	#else
		servo_hipFront_L.init(hipFront_L,  6,  false, 0, initPosition);	// SV 6
		servo_hipFront_R.init(hipFront_R,  9,  true,  0, initPosition);	// SV 9
		servo_hipRear_L .init(hipRear_L,   4,  true,  0, initPosition);	// SV 4
		servo_hipRear_R .init(hipRear_R,   11, false, 0, initPosition);	// SV 11

		servo_kneeFront_L.init(kneeFront_L, 5,  true,  0, initPosition);	// SV 5
		servo_kneeFront_R.init(kneeFront_R, 10, false, 0, initPosition);	// SV 10
		servo_kneeRear_L .init(kneeRear_L,  3,  true,  0, initPosition);	// SV 3
		servo_kneeRear_R .init(kneeRear_R,  12, false, 0, initPosition);	// SV 12
	#endif

	// display initial values in HITIPanel ******************************************
	// motion control parameters (Analog Data 1)
	HC_analogDataWrite(AD_motion_time, motion_time);

	// motion monitoring (Analog Data 2 to 9)
	if(settingModeIsActivated)
	{
		HC_analogDataWrite(AD_Servo_HipFront_L,  initPosition);
		HC_analogDataWrite(AD_Servo_HipFront_L,  initPosition);
		HC_analogDataWrite(AD_Servo_HipFront_R,  initPosition);
		HC_analogDataWrite(AD_Servo_HipRear_L,   initPosition);
		HC_analogDataWrite(AD_Servo_HipRear_R,   initPosition);
		HC_analogDataWrite(AD_Servo_KneeFront_L, initPosition);
		HC_analogDataWrite(AD_Servo_KneeFront_R, initPosition);
		HC_analogDataWrite(AD_Servo_KneeRear_L,  initPosition);
		HC_analogDataWrite(AD_Servo_KneeRear_R,  initPosition);
	}
}



// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
	// communicate with HITIPanel
	HC_communicate();

	// set new motion parameters *********************************************
	// receive motion control parameters (Analog Data 1)
	motion_time = HC_analogDataRead(AD_motion_time);

	// chirp at startup
	if(!isStarted)
	{
		  if(sing())
		      isStarted = true;
	}
		
	// Position offset setting mode ******************************************
	if(settingModeIsActivated)
	{
		// Receive positions setpoint (Analog Data 2 to 9)
		// => for each servo, place it at its optimal middle position. Note the offset you added
		//    and put it in the servo.init() inside the setup()
		setAndMoveGroup(initMotion, 1,
				servo_hipFront_L,  (int) HC_analogDataRead(AD_Servo_HipFront_L),
				servo_hipFront_R,  (int) HC_analogDataRead(AD_Servo_HipFront_R),
				servo_hipRear_L,   (int) HC_analogDataRead(AD_Servo_HipRear_L),
				servo_hipRear_R,   (int) HC_analogDataRead(AD_Servo_HipRear_R),
				servo_kneeFront_L, (int) HC_analogDataRead(AD_Servo_KneeFront_L),
				servo_kneeFront_R, (int) HC_analogDataRead(AD_Servo_KneeFront_R),
				servo_kneeRear_L,  (int) HC_analogDataRead(AD_Servo_KneeRear_L),
				servo_kneeRear_R,  (int) HC_analogDataRead(AD_Servo_KneeRear_R));
	}

	// Normal operation mode *************************************************
	else
	{
		// motion sequences
		walk_forward();
		walk_backward();
		walk_left();
		walk_right();

		turn_left();
		turn_right();

		lean_forward();
		lean_backward();
		lean_left();
		lean_right();

		look_left();
		look_right();

		waveFront_left();
		waveFront_right();

		waveRear_left();
		waveRear_right();

		isScared();
		singOnDemand();

		// Motion monitoring**************************************************
		HC_analogDataWrite(AD_Servo_HipFront_L,  servo_hipFront_L .getCurrentPosition());
		HC_analogDataWrite(AD_Servo_HipFront_R,  servo_hipFront_R .getCurrentPosition());
		HC_analogDataWrite(AD_Servo_HipRear_L,   servo_hipRear_L  .getCurrentPosition());
		HC_analogDataWrite(AD_Servo_HipRear_R,   servo_hipRear_R  .getCurrentPosition());
		HC_analogDataWrite(AD_Servo_KneeFront_L, servo_kneeFront_L.getCurrentPosition());
		HC_analogDataWrite(AD_Servo_KneeFront_R, servo_kneeFront_R.getCurrentPosition());
		HC_analogDataWrite(AD_Servo_KneeRear_L,  servo_kneeRear_L .getCurrentPosition());
		HC_analogDataWrite(AD_Servo_KneeRear_R,  servo_kneeRear_R .getCurrentPosition());
	}
}



// -----------------------------------------------------------------------------
// Robot Functions -------------------------------------------------------------
// -----------------------------------------------------------------------------

// Motion state ----------------------------------------------------------------

// return true if robot is currently performing at least 1 motion sequence
boolean isBusy()
{
	for(byte i = 0; i < MOTION_QTY; i++)
	{
		if((motion_state[i] != 0) && (i != chirping))
			  return true;
	}

	return false;
}


// Motion functions ------------------------------------------------------------

// init group and move 1 motor
void setAndMoveGroup(
		byte motionSequence_ID, byte motionSequence_motion_statesQuantity,
		HC_Servo &servo1, int servo1_positionSetpoint)
{
	// init group
	group.init(1);
	group.add(&servo1);

	// set absolute position 
	group.absolutePosition(servo1.getID(), servo1_positionSetpoint);

	// move group
	moveGroup(motionSequence_ID, motionSequence_motion_statesQuantity);
}


// init group and move 2 motors
void setAndMoveGroup(
		byte motionSequence_ID, byte motionSequence_motion_statesQuantity,
		HC_Servo &servo1, int servo1_positionSetpoint,
		HC_Servo &servo2, int servo2_positionSetpoint)
{
	// init group
	group.init(2);
	group.add(&servo1);
	group.add(&servo2);

    // set absolute position 
    group.absolutePosition(servo1.getID(), servo1_positionSetpoint);
    group.absolutePosition(servo2.getID(), servo2_positionSetpoint);
	
	// move group
	moveGroup(motionSequence_ID, motionSequence_motion_statesQuantity);
}


// init group and move 4 motors
void setAndMoveGroup(
		byte motionSequence_ID, byte motionSequence_motion_statesQuantity,
		HC_Servo &servo1, int servo1_positionSetpoint,
		HC_Servo &servo2, int servo2_positionSetpoint,
		HC_Servo &servo3, int servo3_positionSetpoint,
		HC_Servo &servo4, int servo4_positionSetpoint)
{
	// init group
	group.init(4);
	group.add(&servo1);
	group.add(&servo2);
	group.add(&servo3);
	group.add(&servo4);	

    // set absolute position 
    group.absolutePosition(servo1.getID(), servo1_positionSetpoint);
    group.absolutePosition(servo2.getID(), servo2_positionSetpoint);
    group.absolutePosition(servo3.getID(), servo3_positionSetpoint);
    group.absolutePosition(servo4.getID(), servo4_positionSetpoint);
	
	// move group
	moveGroup(motionSequence_ID, motionSequence_motion_statesQuantity);
}


// init group and move 8 motors
void setAndMoveGroup(
		byte motionSequence_ID, byte motionSequence_motion_statesQuantity,
		HC_Servo &servo1, int servo1_positionSetpoint,
		HC_Servo &servo2, int servo2_positionSetpoint,
		HC_Servo &servo3, int servo3_positionSetpoint,
		HC_Servo &servo4, int servo4_positionSetpoint,       
		HC_Servo &servo5, int servo5_positionSetpoint,
		HC_Servo &servo6, int servo6_positionSetpoint,
		HC_Servo &servo7, int servo7_positionSetpoint,
		HC_Servo &servo8, int servo8_positionSetpoint)
{
	// init group
	group.init(8);
	group.add(&servo1);
	group.add(&servo2);
	group.add(&servo3);
	group.add(&servo4);
	group.add(&servo5);
	group.add(&servo6);
	group.add(&servo7);        
	group.add(&servo8);
		
    // set absolute position 
    group.absolutePosition(servo1.getID(), servo1_positionSetpoint);
    group.absolutePosition(servo2.getID(), servo2_positionSetpoint);
    group.absolutePosition(servo3.getID(), servo3_positionSetpoint);
    group.absolutePosition(servo4.getID(), servo4_positionSetpoint);
    group.absolutePosition(servo5.getID(), servo5_positionSetpoint);
    group.absolutePosition(servo6.getID(), servo6_positionSetpoint);
    group.absolutePosition(servo7.getID(), servo7_positionSetpoint);
    group.absolutePosition(servo8.getID(), servo8_positionSetpoint);
	
	// move group
	moveGroup(motionSequence_ID, motionSequence_motion_statesQuantity);
}


// move group
void moveGroup(
		byte motionSequence_ID, byte motionSequence_motion_statesQuantity)
{
  // set motion time for the whole motion
  group.motionTime(motion_time/motionSequence_motion_statesQuantity);

	// move group
	group.moveNow();

	// when motion ends
	if(group.isEnding())
	{
		// increment motion sequence motion_state
		motion_state[motionSequence_ID]++;

		if(motion_state[motionSequence_ID] > motionSequence_motion_statesQuantity)
		{
			// reset motion sequence motion_state
			motion_state[motionSequence_ID] = 0;
		}
	}    
}


// Wait function ---------------------------------------------------------------

void wait(byte motionSequence_ID)
{
	// wait half the motion time, then move to next motion_state
	if(motion_timer.delay((int) (motion_time/2)))
		motion_state[motionSequence_ID]++;
}


// Walk ------------------------------------------------------------------------

void walk_forward()
{
	byte motion_states_qty = 10;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(walk_F) && (motion_state[walk_F] == 0) && !isBusy())
		motion_state[walk_F]++;

	else if((motion_state[walk_F] == 1))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_kneeRear_R,  80,
				servo_kneeFront_L, 80);
				
	else if((motion_state[walk_F] == 2))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_hipRear_R,  80,
				servo_hipFront_L, 20);
				
	else if((motion_state[walk_F] == 3))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_kneeRear_R,  30,
				servo_kneeFront_L, 30);
				
	else if((motion_state[walk_F] == 4))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_hipRear_R,  45,
				servo_hipFront_L, 45);  
				
	else if((motion_state[walk_F] == 5))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_kneeRear_R,  45,
				servo_kneeFront_L, 45);  
				
	else if((motion_state[walk_F] == 6))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_kneeRear_L,  80,
				servo_kneeFront_R, 80);  
	else if((motion_state[walk_F] == 7))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_hipRear_L,  80,
				servo_hipFront_R, 20);  
				
	else if((motion_state[walk_F] == 8))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_kneeRear_L,  30,
				servo_kneeFront_R, 30);
				
	else if((motion_state[walk_F] == 9))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_hipRear_L,  45,
				servo_hipFront_R, 45);
				
	else if((motion_state[walk_F] == 10))
		setAndMoveGroup(walk_F, motion_states_qty,
				servo_kneeRear_L,  45,
				servo_kneeFront_R, 45);
}

void walk_backward()
{
	byte motion_states_qty = 10;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(walk_B) && (motion_state[walk_B] == 0) && !isBusy())
		motion_state[walk_B]++;

	else if((motion_state[walk_B] == 1))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_kneeRear_R,  80,
				servo_kneeFront_L, 80);
				
	else if((motion_state[walk_B] == 2))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_hipRear_R,  20,
				servo_hipFront_L, 80);
				
	else if((motion_state[walk_B] == 3))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_kneeRear_R,  30,
				servo_kneeFront_L, 30);

	else if((motion_state[walk_B] == 4))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_hipRear_R,  45,
				servo_hipFront_L, 45);  

	else if((motion_state[walk_B] == 5))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_kneeRear_R,  45,
				servo_kneeFront_L, 45);  
					
	else if((motion_state[walk_B] == 6))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_kneeRear_L,  80,
				servo_kneeFront_R, 80);  

	else if((motion_state[walk_B] == 7))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_hipRear_L,  20,
				servo_hipFront_R, 80);  

	else if((motion_state[walk_B] == 8))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_kneeRear_L,  30,
				servo_kneeFront_R, 30);
				
	else if((motion_state[walk_B] == 9))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_hipRear_L,  45,
				servo_hipFront_R, 45);
				
	else if((motion_state[walk_B] == 10))
		setAndMoveGroup(walk_B, motion_states_qty,
				servo_kneeRear_L,  45,
				servo_kneeFront_R, 45);
}

void walk_left()
{
	byte motion_states_qty = 10;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(walk_L) && (motion_state[walk_L] == 0) && !isBusy())
		motion_state[walk_L]++;

	else if((motion_state[walk_L] == 1))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_kneeRear_R,  80,
				servo_kneeFront_L, 80);
				
	else if((motion_state[walk_L] == 2))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_hipRear_R,  0,
				servo_hipFront_L, 90);
				
	else if((motion_state[walk_L] == 3))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_kneeRear_R,  30,
				servo_kneeFront_L, 30);

	else if((motion_state[walk_L] == 4))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_hipRear_R,  45,
				servo_hipFront_L, 45);  

	else if((motion_state[walk_L] == 5))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_kneeRear_R,  45,
				servo_kneeFront_L, 45);  
					
	else if((motion_state[walk_L] == 6))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_kneeRear_L,  80,
				servo_kneeFront_R, 80);  

	else if((motion_state[walk_L] == 7))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_hipRear_L,  90,
				servo_hipFront_R, 0);  

	else if((motion_state[walk_L] == 8))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_kneeRear_L,  30,
				servo_kneeFront_R, 30);
				
	else if((motion_state[walk_L] == 9))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_hipRear_L,  45,
				servo_hipFront_R, 45);
				
	else if((motion_state[walk_L] == 10))
		setAndMoveGroup(walk_L, motion_states_qty,
				servo_kneeRear_L,  45,
				servo_kneeFront_R, 45);
}

void walk_right()
{
	byte motion_states_qty = 10;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(walk_R) && (motion_state[walk_R] == 0) && !isBusy())
		motion_state[walk_R]++;

	else if((motion_state[walk_R] == 1))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_kneeRear_L,  80,
				servo_kneeFront_R, 80);
				
	else if((motion_state[walk_R] == 2))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_hipRear_L,  0,
				servo_hipFront_R, 90);
				
	else if((motion_state[walk_R] == 3))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_kneeRear_L,  30,
				servo_kneeFront_R, 30);

	else if((motion_state[walk_R] == 4))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_hipRear_L,  45,
				servo_hipFront_R, 45);  

	else if((motion_state[walk_R] == 5))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_kneeRear_L,  45,
				servo_kneeFront_R, 45);  
					
	else if((motion_state[walk_R] == 6))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_kneeRear_R,  80,
				servo_kneeFront_L, 80);  

	else if((motion_state[walk_R] == 7))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_hipRear_R,  90,
				servo_hipFront_L, 0);  

	else if((motion_state[walk_R] == 8))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_kneeRear_R,  30,
				servo_kneeFront_L, 30);
				
	else if((motion_state[walk_R] == 9))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_hipRear_R,  45,
				servo_hipFront_L, 45);
				
	else if((motion_state[walk_R] == 10))
		setAndMoveGroup(walk_R, motion_states_qty,
				servo_kneeRear_R,  45,
				servo_kneeFront_L, 45);
}


// Turn ------------------------------------------------------------------------

void turn_left()
{
	byte motion_states_qty = 10;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(turn_L) && (motion_state[turn_L] == 0) && !isBusy())
		motion_state[turn_L]++;

	else if((motion_state[turn_L] == 1))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_kneeRear_R,  80,
				servo_kneeFront_L, 80);
				
	else if((motion_state[turn_L] == 2))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_hipRear_R,  90,
				servo_hipFront_L, 90);
				
	else if((motion_state[turn_L] == 3))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_kneeRear_R,  30,
				servo_kneeFront_L, 30);

	else if((motion_state[turn_L] == 4))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_hipRear_R,  45,
				servo_hipFront_L, 45);  

	else if((motion_state[turn_L] == 5))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_kneeRear_R,  45,
				servo_kneeFront_L, 45);  
					
	else if((motion_state[turn_L] == 6))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_kneeRear_L,  80,
				servo_kneeFront_R, 80);  

	else if((motion_state[turn_L] == 7))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_hipRear_L,  0,
				servo_hipFront_R, 0);  

	else if((motion_state[turn_L] == 8))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_kneeRear_L,  30,
				servo_kneeFront_R, 30);
				
	else if((motion_state[turn_L] == 9))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_hipRear_L,  45,
				servo_hipFront_R, 45);
				
	else if((motion_state[turn_L] == 10))
		setAndMoveGroup(turn_L, motion_states_qty,
				servo_kneeRear_L,  45,
				servo_kneeFront_R, 45);
}

void turn_right()
{
	byte motion_states_qty = 10;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(turn_R) && (motion_state[turn_R] == 0) && !isBusy())
		motion_state[turn_R]++;

	else if((motion_state[turn_R] == 1))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_kneeRear_R,  80,
				servo_kneeFront_L, 80);
				
	else if((motion_state[turn_R] == 2))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_hipRear_R,  0,
				servo_hipFront_L, 0);
				
	else if((motion_state[turn_R] == 3))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_kneeRear_R,  30,
				servo_kneeFront_L, 30);

	else if((motion_state[turn_R] == 4))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_hipRear_R,  45,
				servo_hipFront_L, 45);  

	else if((motion_state[turn_R] == 5))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_kneeRear_R,  45,
				servo_kneeFront_L, 45);  
					
	else if((motion_state[turn_R] == 6))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_kneeRear_L,  80,
				servo_kneeFront_R, 80);  

	else if((motion_state[turn_R] == 7))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_hipRear_L,  90,
				servo_hipFront_R, 90);  

	else if((motion_state[turn_R] == 8))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_kneeRear_L,  30,
				servo_kneeFront_R, 30);
				
	else if((motion_state[turn_R] == 9))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_hipRear_L,  45,
				servo_hipFront_R, 45);
				
	else if((motion_state[turn_R] == 10))
		setAndMoveGroup(turn_R, motion_states_qty,
				servo_kneeRear_L,  45,
				servo_kneeFront_R, 45);
}


// Lean ------------------------------------------------------------------------

void lean_forward()
{
	byte motion_states_qty = 3;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(lean_F) && (motion_state[lean_F] == 0) && !isBusy())
		motion_state[lean_F]++;

	else if((motion_state[lean_F] == 1))
		setAndMoveGroup(lean_F, motion_states_qty,
				servo_kneeFront_L, 90,
				servo_kneeFront_R, 90);
		
	else if((motion_state[lean_F] == 2))
		wait(lean_F);
				
	else if((motion_state[lean_F] == 3))
		setAndMoveGroup(lean_F, motion_states_qty,
				servo_kneeFront_L, 45,
				servo_kneeFront_R, 45);		
}

void lean_backward()
{
	byte motion_states_qty = 3;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(lean_B) && (motion_state[lean_B] == 0) && !isBusy())
		motion_state[lean_B]++;

	else if((motion_state[lean_B] == 1))
		setAndMoveGroup(lean_B, motion_states_qty,
				servo_kneeRear_L, 90,
				servo_kneeRear_R, 90);

	else if((motion_state[lean_B] == 2))
		wait(lean_B);

	else if((motion_state[lean_B] == 3))
		setAndMoveGroup(lean_B, motion_states_qty,
				servo_kneeRear_L, 45,
				servo_kneeRear_R, 45);
}

void lean_left()
{
	byte motion_states_qty = 3;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(lean_L) && (motion_state[lean_L] == 0) && !isBusy())
		motion_state[lean_L]++;

	else if((motion_state[lean_L] == 1))
		setAndMoveGroup(lean_L, motion_states_qty,
				servo_kneeFront_L, 90,
				servo_kneeRear_L,  90);

	else if((motion_state[lean_L] == 2))
		wait(lean_L);

	else if((motion_state[lean_L] == 3))
		setAndMoveGroup(lean_L, motion_states_qty,
				servo_kneeFront_L, 45,
				servo_kneeRear_L,  45);		
}

void lean_right()
{
	byte motion_states_qty = 3;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(lean_R) && (motion_state[lean_R] == 0) && !isBusy())
		motion_state[lean_R]++;

	else if((motion_state[lean_R] == 1))
		setAndMoveGroup(lean_R, motion_states_qty,
				servo_kneeFront_R, 90,
				servo_kneeRear_R,  90);

	else if((motion_state[lean_R] == 2))
		wait(lean_R);

	else if((motion_state[lean_R] == 3))
		setAndMoveGroup(lean_R, motion_states_qty,
				servo_kneeFront_R, 45,
				servo_kneeRear_R,  45);		
}


// Look ------------------------------------------------------------------------

void look_left()
{
	byte motion_states_qty = 3;

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(look_L) && (motion_state[look_L] == 0) && !isBusy())
		motion_state[look_L]++;

	else if((motion_state[look_L] == 1))
		setAndMoveGroup(look_L, motion_states_qty,
				servo_hipRear_L,  80,
				servo_hipRear_R,  10,
				servo_hipFront_L, 10,
				servo_hipFront_R, 80);
				
	else if((motion_state[look_L] == 2))
		wait(look_L);			
				
	else if((motion_state[look_L] == 3))
		setAndMoveGroup(look_L, motion_states_qty,
				servo_hipRear_R,  45,
				servo_hipRear_L,  45,
				servo_hipFront_R, 45,
				servo_hipFront_L, 45);		
}

void look_right()
{
	byte motion_states_qty = 3;

	// set delay duration
	motion_timer.setDuration((int) (motion_time/2));

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(look_R) && (motion_state[look_R] == 0) && !isBusy())
		motion_state[look_R]++;

	else if((motion_state[look_R] == 1))
		setAndMoveGroup(look_R, motion_states_qty,
				servo_hipRear_R,  80,
				servo_hipRear_L,  10,
				servo_hipFront_R, 10,
				servo_hipFront_L, 80);
				
	else if((motion_state[look_R] == 2))
		wait(look_R);			
				
	else if((motion_state[look_R] == 3))
		setAndMoveGroup(look_R, motion_states_qty,
				servo_hipRear_R,  45,
				servo_hipRear_L,  45,
				servo_hipFront_R, 45,
				servo_hipFront_L, 45);		
}


// Wave front ------------------------------------------------------------------

void waveFront_left()
{
	// FOR LOOP: number of times (>= 1)
	byte loop_times = 3;

	byte motion_states_qty = (2 + loop_times*5);

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(waveFront_L) && (motion_state[waveFront_L] == 0) && !isBusy())
		motion_state[waveFront_L]++;

	else if(motion_state[waveFront_L] == 1)
		setAndMoveGroup(waveFront_L, motion_states_qty,
				servo_kneeFront_L, 180);

	// start FOR LOOP****************************************
	else if(motion_state[waveFront_L] == (2 + loop_index*5))
		setAndMoveGroup(waveFront_L, motion_states_qty,
				servo_hipFront_L,  0);	

	else if(motion_state[waveFront_L] == (3 + loop_index*5))
		setAndMoveGroup(waveFront_L, motion_states_qty,
				servo_hipFront_L,  65);

	else if(motion_state[waveFront_L] == (4 + loop_index*5))
		setAndMoveGroup(waveFront_L, motion_states_qty,
				servo_hipFront_L,  0);

	else if(motion_state[waveFront_L] == (5 + loop_index*5))
		setAndMoveGroup(waveFront_L, motion_states_qty,
				servo_hipFront_L,  45);		

	else if(motion_state[waveFront_L] == (6 + loop_index*5))
	{ 
		// this step must not include setAndMoveGroup function
		// increment index at end of 1 loop
		loop_index ++;
		
		// reset FOR loop 
		if(loop_index >= loop_times)
			loop_index = 0;

		// increment motion_state
		motion_state[waveFront_L]++;
	}
	  // end FOR LOOP*****************************************

	else if(motion_state[waveFront_L] == (2 + loop_times*5))
		setAndMoveGroup(waveFront_L, motion_states_qty,
				servo_kneeFront_L, 45);				
}

void waveFront_right()
{
	// FOR LOOP: number of times (>= 1)
	byte loop_times = 3;

	byte motion_states_qty = (2 + loop_times*5);

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(waveFront_R) && (motion_state[waveFront_R] == 0) && !isBusy())
		motion_state[waveFront_R]++;

	else if(motion_state[waveFront_R] == 1)
		setAndMoveGroup(waveFront_R, motion_states_qty,
				servo_kneeFront_R, 180);

	// start FOR LOOP****************************************
	else if(motion_state[waveFront_R] == (2 + loop_index*5))
		setAndMoveGroup(waveFront_R, motion_states_qty,
				servo_hipFront_R,  0);	

	else if(motion_state[waveFront_R] == (3 + loop_index*5))
		setAndMoveGroup(waveFront_R, motion_states_qty,
				servo_hipFront_R,  65);

	else if(motion_state[waveFront_R] == (4 + loop_index*5))
		setAndMoveGroup(waveFront_R, motion_states_qty,
				servo_hipFront_R,  0);

	else if(motion_state[waveFront_R] == (5 + loop_index*5))
		setAndMoveGroup(waveFront_R, motion_states_qty,
				servo_hipFront_R,  45);		

	else if(motion_state[waveFront_R] == (6 + loop_index*5))
	{ 
		// this step must not include setAndMoveGroup function
		// increment index at end of 1 loop
		loop_index ++;
		
		// reset FOR loop 
		if(loop_index >= loop_times)
			loop_index = 0;

		// increment motion_state
		motion_state[waveFront_R]++;
	}
	// end FOR LOOP*****************************************     
						  
	else if(motion_state[waveFront_R] == (2 + loop_times*5))
		setAndMoveGroup(waveFront_R, motion_states_qty,
				servo_kneeFront_R, 45);
}


// Wave rear -------------------------------------------------------------------

void waveRear_left()
{
	// FOR LOOP: number of times (>= 1)
	byte loop_times = 3;

	byte motion_states_qty = (2 + loop_times*5);

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(waveRear_L) && (motion_state[waveRear_L] == 0) && !isBusy())
		motion_state[waveRear_L]++;

	else if(motion_state[waveRear_L] == 1)
		setAndMoveGroup(waveRear_L, motion_states_qty,
				servo_kneeRear_L, 180);

	// start FOR LOOP****************************************
	else if(motion_state[waveRear_L] == (2 + loop_index*5))
		setAndMoveGroup(waveRear_L, motion_states_qty,
				servo_hipRear_L,  0);	

	else if(motion_state[waveRear_L] == (3 + loop_index*5))
		setAndMoveGroup(waveRear_L, motion_states_qty,
				servo_hipRear_L,  65);

	else if(motion_state[waveRear_L] == (4 + loop_index*5))
		setAndMoveGroup(waveRear_L, motion_states_qty,
				servo_hipRear_L,  0);

	else if(motion_state[waveRear_L] == (5 + loop_index*5))
		setAndMoveGroup(waveRear_L, motion_states_qty,
				servo_hipRear_L,  45);	
				
	else if(motion_state[waveRear_L] == (6 + loop_index*5))
	{ 
		// this step must not include setAndMoveGroup function
		// increment index at end of 1 loop
		loop_index ++;
		
		// reset FOR loop 
		if(loop_index >= loop_times)
			loop_index = 0;

		// increment motion_state
		motion_state[waveRear_L]++;
	}
	// end FOR LOOP*****************************************    

	else if(motion_state[waveRear_L] == (2 + loop_times*5))
		setAndMoveGroup(waveRear_L, motion_states_qty,
				servo_kneeRear_L, 45);				
}

void waveRear_right()
{
	// FOR LOOP: number of times (>= 1)
	byte loop_times = 3;

	byte motion_states_qty = (2 + loop_times*5);

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(waveRear_R) && (motion_state[waveRear_R] == 0) && !isBusy())
		motion_state[waveRear_R]++;

	else if(motion_state[waveRear_R] == 1)
		setAndMoveGroup(waveRear_R, motion_states_qty,
				servo_kneeRear_R, 180);

	// start FOR LOOP****************************************
	else if(motion_state[waveRear_R] == (2 + loop_index*5))
		setAndMoveGroup(waveRear_R, motion_states_qty,
				servo_hipRear_R,  0);	
				
	else if(motion_state[waveRear_R] == (3 + loop_index*5))
		setAndMoveGroup(waveRear_R, motion_states_qty,
				servo_hipRear_R,  65);

	else if(motion_state[waveRear_R] == (4 + loop_index*5))
		setAndMoveGroup(waveRear_R, motion_states_qty,
				servo_hipRear_R,  0);

	else if(motion_state[waveRear_R] == (5 + loop_index*5))
		setAndMoveGroup(waveRear_R, motion_states_qty,
				servo_hipRear_R,  45);

	else if(motion_state[waveRear_R] == (6 + loop_index*5))
	{ 
		// this step must not include setAndMoveGroup function
		// increment index at end of 1 loop
		loop_index ++;
		
		// reset FOR loop 
		if(loop_index >= loop_times)
			loop_index = 0;

		// increment motion_state
		motion_state[waveRear_R]++;
	}
	// end FOR LOOP*****************************************    

	else if(motion_state[waveRear_R] == (2 + loop_times*5))
		setAndMoveGroup(waveRear_R, motion_states_qty,
				servo_kneeRear_R, 45);				
}


// Scared ----------------------------------------------------------------------

void isScared()
{
	// FOR LOOP: number of times (>= 1)
	byte loop_times = 3; 

	byte motion_states_qty = (2 + loop_times*3);

	// Start motion sequence if trigger received (must be in motion state 0 with no other motion sequence being performed)
	if(HC_digitalDataRead(scared) && (motion_state[scared] == 0) && !isBusy())
		motion_state[scared]++;

	else if(motion_state[scared] == 1)
		setAndMoveGroup(scared, motion_states_qty,
				servo_kneeFront_R, 0,
				servo_kneeRear_R,  0,
				servo_kneeFront_L, 0,
				servo_kneeRear_L,  0);

	// start FOR LOOP****************************************
	else if(motion_state[scared] == (2 + loop_index*3))
		setAndMoveGroup(scared, motion_states_qty,
				servo_hipRear_R,  80,
				servo_hipRear_L,  10,
				servo_hipFront_R, 10,
				servo_hipFront_L, 80);

	else if(motion_state[scared] == (3 + loop_index*3))
		setAndMoveGroup(scared, motion_states_qty,
				servo_hipRear_R,  10,
				servo_hipRear_L,  80,
				servo_hipFront_R, 80,
				servo_hipFront_L, 10);

	else if(motion_state[scared] == (4 + loop_index*3))
	{
		// increment index at end of 1 loop
		loop_index ++;

		// reset FOR loop	
		if(loop_index >= loop_times)
			loop_index = 0;

		// increment motion_state
		motion_state[scared]++;
	}
	// end FOR LOOP*****************************************     

	else if(motion_state[scared] == (2 + loop_times*3))
		setAndMoveGroup(scared, motion_states_qty,
				servo_kneeFront_R, 45,
				servo_kneeRear_R,  45,
				servo_kneeFront_L, 45,
				servo_kneeRear_L,  45,	
				servo_hipRear_R,   45,
				servo_hipRear_L,   45,
				servo_hipFront_R,  45,
				servo_hipFront_L,  45);				
}


// Chirp -----------------------------------------------------------------------

void singOnDemand()
{
	// Start motion sequence if trigger received (must be in motion state 0)
	if(HC_digitalDataRead(chirping) && (motion_state[chirping] == 0))
		motion_state[chirping]++;
	  
	else if(motion_state[chirping] == 1)
	{
		if(sing())
		{
			// reset sequence
			motion_state[chirping] = 0;
			
			// reset trigger
			HC_digitalDataWrite(chirping, LOW);
		}
	}
}


bool sing()
{
	// Start chirp building sequence (must be in chirp state 0)
	if(chirp_state[1] == 0)
	{
		if(chirp(1, 50))
			chirp_state[1]++;
	}
		
	else if((chirp_state[1] == 1))
	{
		if(chirp(1, 255))
			chirp_state[1]++;
	}
		
	else if((chirp_state[1] == 2))
	{
		if(chirp(3, 0))
		{
			// reset sequence
			chirp_state[1] = 0;
  
			return true;
		}
	}

	return false;
}


// chirp building
// return true when finished
bool chirp(int beeps, int speedms)
{
	// FOR LOOP: number of times (>= 1)
	byte loop_times = beeps; 

	// Start chirp building sequence (must be in chirp state 0)
	if(chirp_state[0] == 0)
		chirp_state[0]++;

	// start FOR LOOP****************************************
	else if(chirp_state[0] == (1 + chirp_loop_index*2))
	{
		// use of delayMicroseconds() is required here:
		// => not possible to use HC_Timer(DELAYMODE_MICROSECOND) as this object depends on 
		//    Loop Cycle Time, which is here around 1ms or more due to Serial Communication.
		for (int i = 0; i < 255; i++)
		{
			digitalWrite(pin_Sounder, HIGH);
			delayMicroseconds((355-i)+ speedms*2);
			digitalWrite(pin_Sounder, LOW);
			delayMicroseconds((355-i)+ speedms*2);
		}

		chirp_state[0]++;
	}

	else if(chirp_state[0] == (2 + chirp_loop_index*2))
	{
		// After 30ms
		if(chirp_timer.delay(30))
		{
			// increment index of the FOR LOOP
			chirp_loop_index++;

			if(chirp_loop_index >= loop_times)
			{
				// reset the FOR LOOP
				chirp_loop_index = 0;
				
				// reset sequence chirp state
				chirp_state[0] = 0;

				return true;
			}
			else
				chirp_state[0]++;
		}
	}
	// end FOR LOOP*****************************************  

	return false;
}
