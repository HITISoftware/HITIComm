/*  Reference sketch for HITIPanel and HITIBrain software
 * 
 *    1) Upload the sketch to your Arduino
 *    2) Open the COM port in the software
 */
 
#include <HITIComm.h>


// sketch ID
// (used to identify the sketch running on your Arduino)
const char code_name[]    PROGMEM = "My sketch";
const char code_version[] PROGMEM = "1.0.0";

// button push counter
unsigned long pushCounter = 0;


void setup()
{
    // initialize HITIComm library
    HC_begin();

    // set sketch ID
    HC_codeName(code_name);
    HC_codeVersion(code_version);

    // set pin 3 as Digital Input (with internal pull-up)
    pinMode(3, INPUT_PULLUP);
    
    // set pin 5 as PWM Output
    HC_outputType(5, PWM);
    pinMode(5, OUTPUT);
    
    // set pin 6 as PWM Output
    HC_outputType(6, PWM);
    pinMode(6, OUTPUT);

    // set pin 8 as Servo Output (initial position is 12.8°)
    HC_attachServo(8, 12800);

    // set pin 13 as Digital Output
    pinMode(LED_BUILTIN, OUTPUT);
}


void loop()
{
    // communicate with HITIPanel software
    HC_communicate();


    // use Digital Data 0 as an Indicator (monitored from Command Panel D0)
    // => indicator turns on 10s after Arduino is powered on
    bool indicatorValue = (millis() > 10000);
    HC_digitalDataWrite(0, indicatorValue);


    // use Analog Data 0 as a Metric (monitored from Command Panel A0)
    // => display the elapsed time in ms since Arduino was powered on.
    float t = millis();
    HC_analogDataWrite(0, t);


    // use Digital Data 1 as a Switch (actuated from Command Panel D1)
    if(HC_digitalDataRead(1))
        // if Switch is activated, move Servo to position 169.3°
        HC_servoWrite(8, 169300);
    else
        // if Switch is deactivated, move Servo to position 12.8°
        HC_servoWrite(8, 12800);

    
    // use Analog Data 1 as a Setpoint (inputed from Command Panel A1)
    // => constrain Setpoint value to be within range 0.0 to 100.0
    // => update displayed value in Command Panel A1
    // => re-map Setpoint value to range 0 to 255
    // => use Setpoint to control the PWM Output on pin 5
    float setpoint = HC_analogDataRead(1);
    setpoint = constrain(setpoint, 0.0, 100.0);
    HC_analogDataWrite(1, setpoint);
    setpoint = map(setpoint, 0.0, 100.0, 0, 255);
    analogWrite(5, setpoint);

    
    // use Digital Data 2 as a Button (actuated from Command Panel D2)
    if(HC_digitalDataRead(2))
    {
        // when Button is pushed, increment counter
        pushCounter++;

        // release Button
        HC_digitalDataWrite(2, LOW);
    }


    // use Analog Data 2 as a Metric (monitored from Command Panel A2)
    // => display the number of times you pushed on the above Button
    HC_analogDataWrite(2, pushCounter);
}
