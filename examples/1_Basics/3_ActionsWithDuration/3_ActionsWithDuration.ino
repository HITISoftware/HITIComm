/*
 HITIComm examples:  1_Basics / 3_ActionsWithDuration

 This sketch shows how to use :
 => Virtual Switches to trigger continuous actions (can be linked to Emotiv headset commands in HITIBrain)
 => Virtual Buttons to trigger punctual actions 
 => Rising Edges to trigger punctual actions (can be linked to Emotiv headset commands in HITIBrain)
 => HITI Timers to control actions duration

 - Red LED          on pin 5
 - Blue LED         on pin 6
 - Servo            on pin 8
 - Virtual Switch   on Digital Data 0
 - Virtual Button   on Digital Data 1
 - Rising Edge      on Digital Data 2 
   
 Copyright © 2023 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>


// sketch ID
const char code_name[]    PROGMEM = "Actions With Duration";
const char code_version[] PROGMEM = "1.0.0";


// HITI Timers
HC_Timer myTimer1;
HC_Timer myTimer2;
HC_Timer myTimer3;


bool RedLED_state = false;
bool BlueLED_state = false;
unsigned long servo_position = 0;
bool doPunctualAction = false;


void setup()
{
    // initialize library
    HC_begin();

    // set sketch ID
    HC_codeName(code_name);
    HC_codeVersion(code_version);

    // set pin 5 and 6 as Digital Outputs
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);

    // set pin 8 as Servo Output (initial position is 12.8°)
    HC_attachServo(8, 12800);
}


void loop()
{
    // communicate with HITIBrain
    HC_communicate();
    

    // --------------------------------------------------------------- 
    // Continuous Action : 
    // => triggered by Virtual Switch on Digital Data 0 
    //    (action runs as long as the Virtual Switch is activated)
    if(HC_digitalDataRead(0))
    {
        // blink red LED (invert LED state every 1s)
        if(myTimer1.delay(1000))
            RedLED_state = !RedLED_state;
    }
    else 
        // turn off red LED
        RedLED_state = false;

    digitalWrite(5, RedLED_state);


    // -------------------------------------------------------------
    // Punctual Action : 
    // => triggered by Virtual Button on Digital Data 1
    //    (action is triggered when the Button is clicked)
    // => triggered by Rising Edge on Digital Data 2
    //    (action is triggered on a rising edge of the Digital Data boolean value)
    if(HC_digitalDataRead_click(1) || HC_digitalDataRead_risingEdge(2))
    {
        doPunctualAction = true;

        myTimer2.reset();
        myTimer3.reset();
    }

    if(doPunctualAction)
    {
        // blink blue LED (invert LED state every 0.25s)
        if(myTimer2.delay(250))
            BlueLED_state = !BlueLED_state;

        // move servo to position 169.3°
        servo_position = 169300;

        // this action lasts 2s
        if(myTimer3.delay(2000))
            doPunctualAction = false;
    }
    else 
    {
        // turn off blue LED
        BlueLED_state = false;

        // move servo back to its initial position 12.8°
        servo_position = 12800;
    }
    digitalWrite(6, BlueLED_state);
    HC_servoWrite(8, servo_position);
}
