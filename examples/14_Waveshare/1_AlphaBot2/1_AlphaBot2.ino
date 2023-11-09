/*
 HITIComm examples:  Waveshare / 1_Waveshare_AlphaBot2

 This sketch shows how to use HITIPanel to:
   => control and monitor a Waveshare AlphaBot2 mobile robot

 - 2 DC Motors:
    * H-bridge Control (IN)   on pin A0, A1, A2, A3
    * Speed Control (PWM)     on pin 5,6
    * Trigger Move Forward    on Digital Data 0
    * Trigger Move Backward   on Digital Data 1
    * Trigger Rotate Left     on Digital Data 2
    * Trigger Rotate Right    on Digital Data 3
    * Linear Speed            on Analog Data 0
    * Rotation Speed          on Analog Data 1

 - 1 Distance sensor HC-SR04
    * Echo                            on pin 2
    * Trig                            on pin 3
    * Converted in cm                 on Analog Data 2
    * Converted in cm, filtered       on Analog Data 3

 - I/O expander for I2C bus (PCF8574)
    * SDA                           on pin 17
    * SCL                           on pin 18
    * Joystick                      on pin P0 to P4
    * Buzzer                        on pin P5
    * Opto interrupters             on pin P6 to P7 
    * Joystick readings             on Digital Data 10 to 14
    * Activate Buzzer               on Digital Data 4
    * Opto interrupters readings    on Digital Data 5 to 6

 - Infrared Receiver
    * IR Receiver           on pin 4

 - 4 RGB LEDs (WS2812)
    * DIN                   on pin 7


 Copyright © 2023 Christophe LANDRET
 MIT License
*/


#include <HITIComm.h>
#include <HC_SignalFilter.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>



// -----------------------------------------------------------------------------
// Global Variables ------------------------------------------------------------
// -----------------------------------------------------------------------------


// sketch ID ***************************************************************
// (used to identify the sketch running on your Arduino)

const char code_name[]    PROGMEM = "AlphaBot2";
const char code_version[] PROGMEM = "1.0.0";


// Pins and Data ***********************************************************

// Pins assignment
const byte pin_ultrasonic_echo = 2;
const byte pin_ultrasonic_trig = 3;
const byte pin_IRreceiver   = 4;
const byte pin_motorR_speed = 5; // PWMB
const byte pin_motorL_speed = 6; // PWMA
const byte pin_RGB_control = 7;   // DIN
const byte pin_motorL_moveForward  = A0; // AIN2
const byte pin_motorL_moveBackward = A1; // AIN1
const byte pin_motorR_moveForward  = A2; // BIN1
const byte pin_motorR_moveBackward = A3; // BIN2

// Digital Data assignment
const byte dd_moveForward   = 0; // Virtual Switch
const byte dd_moveBackward  = 1; // Virtual Switch
const byte dd_rotateLeft    = 2; // Virtual Switch
const byte dd_rotateRight   = 3; // Virtual Switch
const byte dd_optoInterrupter_left  = 5; // Indicator
const byte dd_optoInterrupter_right = 6; // Indicator
const byte dd_activateBuzzer  = 8; // Virtual Switch
const byte dd_joystick_up     = 10; // Indicator
const byte dd_joystick_down   = 11; // Indicator
const byte dd_joystick_left   = 12; // Indicator
const byte dd_joystick_right  = 13; // Indicator
const byte dd_joystick_center = 14; // Indicator

// Analog Data assignment
const byte ad_linearSpeed_setpoint = 0;     // Setpoint (in %)
const byte ad_rotarySpeed_setpoint = 2;     // Setpoint (in %)
const byte ad_distance_metric = 5;          // Metric (in cm)
const byte ad_distance_filtered_metric = 6; // Metric (in cm)
const byte ad_RGB_rainbow_setpoint = 8;     // Setpoint (in %)


// Devices parameters ******************************************************

// Robot motion
byte linearSpeed = 30;
byte rotarySpeed = 30;

// Ultrasonic distance sensor (HC-SR04)
volatile unsigned long pulseLength_startTime = 0;
volatile bool distanceMeasurementOngoing = false;
volatile bool distanceMeasurementEnded = false;
volatile float distance = 0.0;
float distance_filtered = 0.0;
HC_SignalFilter distanceFilter;

// I/O expander for I2C bus (PCF8574)
const byte pcf8574_address = 0x20;
byte inputs_values = 0;

// 4 RGB LEDs in cascade (WS2812)
// - 800 kHz and GRB bitstream
Adafruit_NeoPixel RGB = Adafruit_NeoPixel(4, pin_RGB_control, NEO_GRB + NEO_KHZ800);
HC_Timer rainbow_timer;
unsigned long rainbow_forLoopIndex = 0;



// -----------------------------------------------------------------------------
// Setup -----------------------------------------------------------------------
// -----------------------------------------------------------------------------


void setup()
{
    // Initialize libraries and related objects *********************

    // HITIComm
    HC_begin();

    // Wire
    Wire.begin();


    // Sketch ID ****************************************************

    // set sketch ID
    HC_codeName(code_name);
    HC_codeVersion(code_version);


    // Motion *******************************************************

    // pin modes
    pinMode(pin_motorL_speed,OUTPUT);   
    pinMode(pin_motorR_speed,OUTPUT);
    HC_outputType(pin_motorL_speed, PWM);
    HC_outputType(pin_motorR_speed, PWM);

    pinMode(pin_motorL_moveBackward,OUTPUT);      
    pinMode(pin_motorL_moveForward,OUTPUT);
    pinMode(pin_motorR_moveForward,OUTPUT);     
    pinMode(pin_motorR_moveBackward,OUTPUT);

    // no motion at start
    stop();

    // initialize displayed speeds setpoints
    HC_analogDataWrite(ad_linearSpeed_setpoint, linearSpeed);
    HC_analogDataWrite(ad_rotarySpeed_setpoint, rotarySpeed);


    // Ultrasonic distance sensor (HC-SR04) *************************

    // pin modes
    pinMode(pin_ultrasonic_echo, INPUT);
    pinMode(pin_ultrasonic_trig, OUTPUT);

    // attach pin change interrupt on ECHO pin
    attachInterrupt(digitalPinToInterrupt(pin_ultrasonic_echo), echo_interrupt, CHANGE);

    // Average filter to smooth distance data
    // => increase buffer size to improve filtering (default: 10, max: 255)
    distanceFilter.setBufferSize(10);


    // 4 RGB LEDs (WS2812) ******************************************
    
    // initialize NeoPixel object
    RGB.begin();

    // Turn off all LEDs
    RGB_turnOff();
}



// -----------------------------------------------------------------------------
// Loop ------------------------------------------------------------------------
// -----------------------------------------------------------------------------


void loop()
{
    // communicate with HITIPanel software
    HC_communicate();


    // Motion ******************************************************

    // read speeds setpoints set by user (in %)
    // 1) constrain inputs in range 0-100%
    // 2) update Analog Data with constrained values (also update displayed value in HITIPanel)
    // 3) remap setpoint from range 0-100% to range 0-255
    linearSpeed = HC_analogDataRead_setpoint(ad_linearSpeed_setpoint, 0, 100, 0, 255);
    rotarySpeed = HC_analogDataRead_setpoint(ad_rotarySpeed_setpoint, 0, 100, 0, 255);

    // if Virtual Switches are activated, move robot
    if     (HC_digitalDataRead(dd_moveForward)) moveForward();
    else if(HC_digitalDataRead(dd_moveBackward)) moveBackward();
    else if(HC_digitalDataRead(dd_rotateLeft)) rotateLeft();
    else if(HC_digitalDataRead(dd_rotateRight)) rotateRight();
    // else stop robot
    else stop();


    // Ultrasonic distance sensor (HC-SR04) ************************

    measureDistance();
    
    // display distance values (unfiltered and filtered)
    HC_analogDataWrite(ad_distance_metric,          distance);
    HC_analogDataWrite(ad_distance_filtered_metric, distance_filtered);


    // I/O expander for I2C bus (PCF8574) **************************

    IOexpander_readInputs();

    // display inputs values (Joystick)
    HC_digitalDataWrite(dd_joystick_up,     readJoystick_up());
    HC_digitalDataWrite(dd_joystick_down,   readJoystick_down());
    HC_digitalDataWrite(dd_joystick_left,   readJoystick_left());
    HC_digitalDataWrite(dd_joystick_right,  readJoystick_right());
    HC_digitalDataWrite(dd_joystick_center, readJoystick_center());

    // display inputs values (2 infrared Opto Interrupters at front)
    HC_digitalDataWrite(dd_optoInterrupter_left,   readOptoInterrupter_left());
    HC_digitalDataWrite(dd_optoInterrupter_right,  readOptoInterrupter_right());
    
    // control buzzer from user interface
    activateBuzzer(HC_digitalDataRead(dd_activateBuzzer));


    // 4 RGB LEDs (WS2812) *****************************************
   

    // read RGB setpoint set by user (rainbow change rate in %)
    byte rainbowSetpoint = HC_analogDataRead_setpoint(ad_RGB_rainbow_setpoint, 0, 100);

    if(rainbowSetpoint == 0) 
        RGB_turnOff();
    else
    {
        RGB.setBrightness(rainbowSetpoint);

        // remap setpoint from range 0-100% to range 100-0ms
        rainbowSetpoint = map(rainbowSetpoint, 0, 100, 100, 0);
        RGB_rainbow(rainbowSetpoint);
    }
}



// -----------------------------------------------------------------------------
// Robot motions ---------------------------------------------------------------
// -----------------------------------------------------------------------------


void moveForward()
{
    analogWrite(pin_motorL_speed, linearSpeed);
    analogWrite(pin_motorR_speed, linearSpeed);
    digitalWrite(pin_motorL_moveBackward, LOW);
    digitalWrite(pin_motorL_moveForward,  HIGH);
    digitalWrite(pin_motorR_moveForward,  LOW);  
    digitalWrite(pin_motorR_moveBackward, HIGH); 
}

void moveBackward()
{
    analogWrite(pin_motorL_speed, linearSpeed);
    analogWrite(pin_motorR_speed, linearSpeed);
    digitalWrite(pin_motorL_moveBackward, HIGH);
    digitalWrite(pin_motorL_moveForward,  LOW);
    digitalWrite(pin_motorR_moveForward,  HIGH); 
    digitalWrite(pin_motorR_moveBackward, LOW);  
}

void rotateRight()
{
    analogWrite(pin_motorL_speed, rotarySpeed);
    analogWrite(pin_motorR_speed, rotarySpeed);
    digitalWrite(pin_motorL_moveBackward, LOW);
    digitalWrite(pin_motorL_moveForward,  HIGH);
    digitalWrite(pin_motorR_moveForward,  HIGH); 
    digitalWrite(pin_motorR_moveBackward, LOW);  
}

void rotateLeft()
{
    analogWrite(pin_motorL_speed, rotarySpeed);
    analogWrite(pin_motorR_speed, rotarySpeed);
    digitalWrite(pin_motorL_moveBackward, HIGH);
    digitalWrite(pin_motorL_moveForward,  LOW);
    digitalWrite(pin_motorR_moveForward,  LOW); 
    digitalWrite(pin_motorR_moveBackward, HIGH);  
}

void stop()
{
    analogWrite(pin_motorL_speed, 0);
    analogWrite(pin_motorR_speed, 0);
    digitalWrite(pin_motorL_moveBackward, LOW);
    digitalWrite(pin_motorL_moveForward,  LOW);
    digitalWrite(pin_motorR_moveForward,  LOW); 
    digitalWrite(pin_motorR_moveBackward, LOW);  
}



// -----------------------------------------------------------------------------
// Ultrasonic distance sensor (HC-SR04) ----------------------------------------
// -----------------------------------------------------------------------------


// on ECHO pin CHANGE interrupt
void echo_interrupt()
{
    // if rising edge
    if(digitalRead(pin_ultrasonic_echo))
        pulseLength_startTime = micros();

    // if falling edge
    else
    {
        distanceMeasurementOngoing = false;
        distanceMeasurementEnded = true;

        // calculate pulse length (in ms)
        unsigned long pulseLength = micros() - pulseLength_startTime;
        
        // convert pulse length to distance (in cm)
        // => sensor range : 2cm to 400cm
        distance = ((float) pulseLength) / 58.0;
    }
}

void measureDistance()
{
    // start a new measurement if no measurement is ongoing
    if(!distanceMeasurementOngoing)
    {
        distanceMeasurementOngoing = true;

        // reset TRIG for 2us
        digitalWrite(pin_ultrasonic_trig, LOW);
        delayMicroseconds(2);

        // set TRIG for at least 10us
        digitalWrite(pin_ultrasonic_trig, HIGH);
        delayMicroseconds(10);

        // reset TRIG
        digitalWrite(pin_ultrasonic_trig, LOW);
    }

    // when measurement ends
    if(distanceMeasurementEnded)
    {
        distanceMeasurementEnded = false;

        // calculate average value based on last 50 acquired data
        // => increase resolution
        // => smooth plotted curve
        distance_filtered = distanceFilter.median(distance);
    }
}



// -----------------------------------------------------------------------------
// I/O expander for I2C bus (PCF8574) ------------------------------------------
// -----------------------------------------------------------------------------


void IOexpander_writeData(byte data)
{
    Wire.beginTransmission(pcf8574_address);
    Wire.write(data);
    Wire.endTransmission(); 
}

byte IOexpander_readData()
{
    Wire.requestFrom(pcf8574_address, 1);
    if(Wire.available())
        return Wire.read();
    
    return -1;
}

// write Digital Output
// * Note: Output HIGH current is very weak (max -1mA). Output LOW current is higher (max 25mA).
// => Outputs must be activated on output LOW.
bool IOexpander_digitalWrite(byte index, bool value)
{
    if(value)
        IOexpander_writeData((0x01 << index) | IOexpander_readData());
    else
        IOexpander_writeData(~(0x01 << index) & IOexpander_readData());
}

// read Digital Inputs
// * Note: all PCF8574 I/O are outputs.
// * An output can be used as an input because the real value at an output can be read, AND:
//   - when this output is set to HIGH, applying a 0V at this output will force it to 0V
//   - when this output is set to LOW, applying a +5V at this output will NOT force it to +5V
// => So, in order to use an output as an input, this output must be set to HIGH beforehand.
void IOexpander_readInputs()
{
    // set outputs to HIGH for I/O to use as inputs (P1 to P7, except P5)
    IOexpander_writeData(0b11011111 | IOexpander_readData());

    // read real output values
    inputs_values = IOexpander_readData();
}

// to call after IOexpander_readInputs()
bool IOexpander_digitalRead(byte index)
{
    return bitRead(inputs_values, index);
}

// Buzzer on pin P5
void activateBuzzer(bool activate)
{
    IOexpander_digitalWrite(5, !activate);
}

// Joystick on pin P0 to P4 (active LOW)
bool readJoystick_up()      { return !IOexpander_digitalRead(0); }
bool readJoystick_right()   { return !IOexpander_digitalRead(1); }
bool readJoystick_left()    { return !IOexpander_digitalRead(2); }
bool readJoystick_down()    { return !IOexpander_digitalRead(3); }
bool readJoystick_center()  { return !IOexpander_digitalRead(4); }

// Infrared opto interrupter ST188 on pin P6 to P7 (active LOW)
bool readOptoInterrupter_right()  { return !IOexpander_digitalRead(6); }
bool readOptoInterrupter_left()   { return !IOexpander_digitalRead(7); }



// -----------------------------------------------------------------------------
// 4 RGB LEDs in cascade (WS2812) ----------------------------------------------
// -----------------------------------------------------------------------------


void RGB_turnOff()
{
    RGB.clear();
    RGB.show();
}


// Rainbow cycle along the 4 LEDs. Wait time in ms.
void RGB_rainbow(int wait) {
    // every xx ms, change rainbow
    if(rainbow_timer.delay(wait))
    {
        // argument is the first pixel hue of the rainbow
        RGB.rainbow(rainbow_forLoopIndex);
        RGB.show();

        rainbow_forLoopIndex += 1024;

        if(rainbow_forLoopIndex >= 5*65536)
            rainbow_forLoopIndex = 0;
    }
}