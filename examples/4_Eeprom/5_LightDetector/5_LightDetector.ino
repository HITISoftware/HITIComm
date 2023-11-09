/*
 HITIComm examples:  Eeprom / 5_LightDetector

 This sketch shows how to use HITIPanel software to:
   => create a hysteresis light detector using a photoresistor
   => trigger actions (servo motion) when detecting light
   => save/load hysteresis detector settings in Eeprom

 - servo                          on pin 8
 - on-board LED                   on pin 13
 - photoresistor (light sensor)   on pin A0
 - low threshold                  on Analog Data 0
 - high threshold                 on Analog Data 1

 Copyright © 2023 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>


// sketch ID
const char code_name[]    PROGMEM = "Light Detector";
const char code_version[] PROGMEM = "1.0.0";

// pins assignment
const int pin_servo = 8;
const int pin_LED = LED_BUILTIN;
const int pin_Photoresistor = A2;

// digital data assignment
const int dd_SaveSettings = 0; // Virtual Button

// Analog Data assignment:
const int ad_threshold_Low = 0;  // Setpoint (0-1023)
const int ad_threshold_High = 1; // Setpoint (0-1023)

// hysteresis detector settings
int threshold_Low = 0;
int threshold_High = 1023;


void setup()
{
    // initialize library
    HC_begin();

    // set sketch ID
    HC_codeName(code_name);
    HC_codeVersion(code_version);

    // set pin 13 as Digital Output
    pinMode(pin_LED, OUTPUT);

    // set pin 8 as Servo Output (initial position is 10.0°)
    HC_attachServo(pin_servo, 10000);

    // 1) load settings from Eeprom (Integers 0 and 1)
    // 2) initialize Analog Data with settings
    HC_analogDataWrite(ad_threshold_Low,  HC_eeprom.readInteger(0));
    HC_analogDataWrite(ad_threshold_High, HC_eeprom.readInteger(1));
}


void loop()
{
    // communicate with HITIPanel or HITIBrain
    HC_communicate();

    // read new settings set by user from HITIPanel or HITIBrain
    // (constrain input values between 0 and 1023)
    threshold_Low = (int) HC_analogDataRead_setpoint(ad_threshold_Low, 0, 1023);
    threshold_High = (int) HC_analogDataRead_setpoint(ad_threshold_High, 0, 1023);

    // make input values consistent
    // (High threshold must be higher or equal to Low threshold)
    correctInputs();

    // if light is detected
    if (analogRead(pin_Photoresistor) > threshold_High)
    {
        digitalWrite(pin_LED, HIGH);      // turn LED on
        HC_servoWrite(pin_servo, 170000); // move servo to position 170°
    }
    // if dark is detected
    else if (analogRead(pin_Photoresistor) < threshold_Low)
    {
        digitalWrite(pin_LED, LOW);       // turn LED off
        HC_servoWrite(pin_servo, 10000);  // move servo back to position 10°
    }

    // if virtual button SAVE SETTINGS is clicked in HITIPanel
    if(HC_digitalDataRead_click(dd_SaveSettings))
    {
        // save settings to Eeprom (Integers 0 and 1)
        HC_eeprom.writeInteger(0, threshold_Low);
        HC_eeprom.writeInteger(1, threshold_High);
    }
}


void correctInputs()
{
    if (threshold_Low > threshold_High)
    {
        threshold_Low = threshold_High;

        // update value in HITIPanel or HITIBrain
        HC_analogDataWrite(ad_threshold_Low, threshold_Low);
    }
    if (threshold_High < threshold_Low)
    {
        threshold_High = threshold_Low;

        // update value in HITIPanel or HITIBrain
        HC_analogDataWrite(ad_threshold_High, threshold_High);
    }
}
