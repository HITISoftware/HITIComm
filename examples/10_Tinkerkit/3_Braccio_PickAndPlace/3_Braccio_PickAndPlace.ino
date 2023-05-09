/*
HITIComm examples:  Amazing / 9_Tinkerkit_Groups

 This sketch shows how to create a complex motion sequence of several synchronized
 servos by using a HITI Motor Group together with arrays of setpoints.
 This example is applied to the Tinkerkit Braccio arm.

 Motion sequence performed:
    - step 0 : ready to start. Wait for the Start
    - step 1 : move group (all servos) to position 1
    - step 2 : move group (all servos) to position 2
    - step 3 : move group (all servos) to position 3
    - step 4 : move group (all servos) to position 4
    - step 5 : move group (all servos) back to initial position

 - 6 Servos                        on pin 3,5,6,9,10,11
 - Start/Stop (virtual switches)   on Digital Data 0-1
 - Sequence step                   on Analog Data 0
 - Current position (Servos 1-4)   on Analog Data 1-4

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_Servo.h>
#include <HC_MotorGroup.h>
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
const int ad_Step              = 0; // main sequence step
const int ad_SubStep           = 1; // sub sequence step
const int ad_Position_Base     = 2; // current positions
const int ad_Position_Shoulder = 3;
const int ad_Position_Elbow    = 4;
const int ad_Position_WristVer = 5;
const int ad_Position_WristRot = 6;
const int ad_Position_Gripper  = 7;


// Digital Data assignment
const int dd_Start = 0; // virtual switches
const int dd_Stop  = 1;

// main motion sequence
byte step          = 0;
byte lastStep      = 3;

// sub motion sequence
byte sub1_step     = 0;
byte sub1_lastStep = 3;
byte sub2_step     = 0;
byte sub2_lastStep = 1;
byte sub3_step     = 0;
byte sub3_lastStep = 4;

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

// Servos index
const int index_Base     = 1;
const int index_Shoulder = 2;
const int index_Elbow    = 3;
const int index_WristVer = 4;
const int index_WristRot = 5;
const int index_Gripper  = 6;

// Position: initial
const int iPos_Base     = 0;
const int iPos_Shoulder = 35;
const int iPos_Elbow    = 180;
const int iPos_WristVer = 0;
const int iPos_WristRot = 0;
const int iPos_Gripper  = 73;

// Base position
const int Base[3] = { 0, 90, 180 };

// Gripper position
const int opened = 13;
const int closed = 45;

// object 4 is on top
// Object i position  Obj 0   Obj 1   Obj 2   Obj 3   Obj 4
const int o_Sh[5] = { 76,     81,     83,     81,     81 };
const int o_El[5] = { 0,      0,      6,      21,     28 };
const int o_Wv[5] = { 59,     58,     50,     31,     27 };

// hover 1 is the highest clearance position
// Hover i position   Hov 0   Hov 1  
const int h_Sh[2] = { 74,     67 };
const int h_El[2] = { 46,     68 };
const int h_Wv[2] = { 14,     0 };

// Speed (°/s)
const int slow = 10;
const int fast = 120;

// Motion time (s)
const int motionTime = 2;

// HITI Motor Group
HC_MotorGroup group;

// HITI Timer
HC_Timer timer;


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
	servo_Base.    init(index_Base,     pin_Servo_Base,     false, 7.0, iPos_Base,     &base);
	servo_Shoulder.init(index_Shoulder, pin_Servo_Shoulder, false, 2.5, iPos_Shoulder, &shoulder);
	servo_Elbow.   init(index_Elbow,    pin_Servo_Elbow,    false, 4.0, iPos_Elbow,    &elbow);
	servo_WristVer.init(index_WristVer, pin_Servo_WristVer, false, 7.0, iPos_WristVer, &wrist_ver);
	servo_WristRot.init(index_WristRot, pin_Servo_WristRot, true,  0.0, iPos_WristRot, &wrist_rot);
	servo_Gripper. init(index_Gripper,  pin_Servo_Gripper,  false, 2.0, iPos_Gripper,  &gripper);

	// set Min and Max absolute position limits (min, max) (°)
	servo_Base.    travelLimits(0.0,  180.0);
	servo_Shoulder.travelLimits(17.0, 163.0);
	servo_Elbow.   travelLimits(0.0,  180.0);
	servo_WristVer.travelLimits(0.0,  180.0);
	servo_WristRot.travelLimits(0.0,  125.0);
	servo_Gripper. travelLimits(13.0, 77.0);
}


// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
	// communicate with HITIPanel
	HC_communicate();
   

    // start/stop motion sequence ------------------------------------------

    // step 0: ready to start sequence with the START button
    if ((step == 0) && HC_digitalDataRead(dd_Start))
    {
        step = 1;        // start sequence
    }

    // at any step: stop sequence with the STOP button
    if (HC_digitalDataRead(dd_Stop))
    {
        group.stopNow(); // stop group
        timer.reset();   // reset timer
        step = 0;        // reset sequence
        sub1_step = 0;   // reset sub-sequence
        sub2_step = 0;   // reset sub-sequence
        sub3_step = 0;   // reset sub-sequence
    }

    else if (step == 1) move_init();       // Init positioning: Base 0
    else if (step == 2) moveStack(0, 2);   // Move stack: base 0 -> 2
    else if (step == 3) moveStack(2, 1);   // Move stack: base 2 -> 1

    // deactivate the Virtual Buttons
    HC_digitalDataWrite(dd_Start, false);
    HC_digitalDataWrite(dd_Stop, false);


    // display data in HITIPanel ---------------------------------------------

    // sequence step
    HC_analogDataWrite(ad_Step, step);
    HC_analogDataWrite(ad_SubStep, sub1_step);

    // current positions (servos)
    HC_analogDataWrite(ad_Position_Base    , servo_Base    .getCurrentPosition());
    HC_analogDataWrite(ad_Position_Shoulder, servo_Shoulder.getCurrentPosition());
    HC_analogDataWrite(ad_Position_Elbow   , servo_Elbow   .getCurrentPosition());
    HC_analogDataWrite(ad_Position_WristVer, servo_WristVer.getCurrentPosition());
    HC_analogDataWrite(ad_Position_WristRot, servo_WristRot.getCurrentPosition());
    HC_analogDataWrite(ad_Position_Gripper , servo_Gripper .getCurrentPosition());
}


// -----------------------------------------------------------------------------
// Methods ---------------------------------------------------------------------
// -----------------------------------------------------------------------------


// main motion sequence *****************************************************

// called each time a motion ends
void onMotionDone()
{
    // at end of steps 0-1
    if (step < lastStep)
        step++;   // go to next step

    // at end of step 2
    else
        step = 0; // go to step 0
}


// sub-sequences ************************************************************

// called each time a sub-motion ends
void onSub1MotionDone()
{
    // at end of steps 0-2
    if (sub1_step < sub1_lastStep)
        sub1_step++;   // go to next step

    // at end of step 3
    else
    {
        sub1_step = 0; // go to step 0
        onSub2MotionDone(); // increment upper level motion step
    }
}

// called each time a sub-motion ends
void onSub2MotionDone()
{
    // at end of steps 0
    if (sub2_step < sub2_lastStep)
        sub2_step++;   // go to next step

    // at end of step 1
    else
    {
        sub2_step = 0;  // go to step 0
        onSub3MotionDone(); // increment upper level motion step
    }
}

// called each time a sub-motion ends
void onSub3MotionDone()
{
    // at end of steps 0
    if (sub3_step < sub3_lastStep)
        sub3_step++;   // go to next step

    // at end of step 1
    else
    {
        sub3_step = 0;  // go to step 0
        onMotionDone(); // move to next main step
    }
}


// move to initial position ************************************************
void move_init()
{
    // init group
    group.init(5);
    group.add(&servo_Base);
    group.add(&servo_Shoulder);
    group.add(&servo_Elbow);
    group.add(&servo_WristVer);
    group.add(&servo_Gripper);

    // move servos of the group 
    group.absolutePosition(index_Base,     Base[0]);
    group.absolutePosition(index_Shoulder, h_Sh[1]);
    group.absolutePosition(index_Elbow,    h_El[1]);
    group.absolutePosition(index_WristVer, h_Wv[1]);
    group.absolutePosition(index_Gripper,  opened); 
    group.motionTime(motionTime);
    group.moveNow();

    // when motion ends
    if (group.isEnding()) onMotionDone();
}


// move vertical ***********************************************************

void move_vertical(boolean ObjectOrHover, byte index)
{
    // init group
    group.init(3);
    group.add(&servo_Shoulder);
    group.add(&servo_Elbow);
    group.add(&servo_WristVer);

    // move servos of the group 
    group.absolutePosition(index_Shoulder, (ObjectOrHover ? h_Sh[index] : o_Sh[index]));
    group.absolutePosition(index_Elbow,    (ObjectOrHover ? h_El[index] : o_El[index]));
    group.absolutePosition(index_WristVer, (ObjectOrHover ? h_Wv[index] : o_Wv[index]));
    group.motionTime(motionTime);
    group.moveNow();

    // when motion ends
    if (group.isEnding()) onSub1MotionDone();
}

void move_toObject(byte index)
{
    if (index >= sizeof(o_Sh) / sizeof(int))
        index = (sizeof(o_Sh) / sizeof(int)) - 1;

    move_vertical(false, index);
}

void move_toHover(byte index)
{
    if (index >= sizeof(h_Sh) / sizeof(int))
        index = (sizeof(h_Sh) / sizeof(int)) - 1;

    move_vertical(true, index);
}


// move Base **************************************************************

void move_toBase(byte index)
{
    if (index >= sizeof(Base) / sizeof(int))
        index = (sizeof(Base) / sizeof(int)) - 1;

    servo_Base.autoReset();
    servo_Base.absolutePosition(Base[index]);
    servo_Base.speed(fast);
    servo_Base.moveNow();

    // when motion ends
    if (servo_Base.isEnding()) onSub1MotionDone();
}


// move gripper **********************************************************

void move_gripper(float position)
{
    servo_Gripper.autoReset();
    servo_Gripper.absolutePosition(position);
    servo_Gripper.speed(fast);
    servo_Gripper.moveNow();

    // when motion ends
    if (servo_Gripper.isEnding()) onSub1MotionDone();
}

// close gripper
void close()
{
    move_gripper(closed);
}

// open gripper
void open()
{
    move_gripper(opened);
}


// take/release *********************************************************

void take(byte base, byte object)
{
    if      (sub1_step == 0) move_toBase(base);   
    else if (sub1_step == 1) move_toObject(object);
    else if (sub1_step == 2) close();      
    else if (sub1_step == 3) move_toHover(1);
}

void release(byte base, byte object)
{
    if      (sub1_step == 0) move_toBase(base);
    else if (sub1_step == 1) move_toObject(object);
    else if (sub1_step == 2) open();         
    else if (sub1_step == 3) move_toHover(1);
}


// move object *********************************************************

void moveObject(byte base_source, byte base_dest, byte object_source, byte object_dest)
{
    if      (sub2_step == 0) take   (base_source, object_source);
    else if (sub2_step == 1) release(base_dest,   object_dest);  
}


// move stack **********************************************************

void moveStack(byte base_source, byte base_dest)
{
    if      (sub3_step == 0) moveObject(base_source, base_dest, 4, 0); // take O4, release O0
    else if (sub3_step == 1) moveObject(base_source, base_dest, 3, 1); // take O3, release O1
    else if (sub3_step == 2) moveObject(base_source, base_dest, 2, 2); // take O2, release O2
    else if (sub3_step == 3) moveObject(base_source, base_dest, 1, 3); // take O1, release O3
    else if (sub3_step == 4) moveObject(base_source, base_dest, 0, 4); // take O0, release O4
}