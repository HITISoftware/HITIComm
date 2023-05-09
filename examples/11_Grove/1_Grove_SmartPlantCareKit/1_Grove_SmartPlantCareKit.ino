/*
 HITIComm examples:  Grove / 4_Grove_SmartPlantCareKit

 This sketch shows how to use HITIPanel to:
   => control and monitor a plant watering system based on sensors feedback

 - Grove OLED Display 0.96"       I2C
 - Grove Sunlight Sensor v1.0     I2C
 - Grove Water flow sensor        on pin 2
 - Grove Encoder                  on pin 3-4
 - Grove Button                   on pin 5
 - Grove Relay                    on pin 6
 - Grove temperature & humidity sensor   on pin A0
 - Grove Moisture sensor          on pin A1

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

#include <DHT.h>
#include <Encoder.h>
#include <SI114X.h>
//#include <U8g2lib.h> // this library eats at least 1200 bytes of Sram


// -----------------------------------------------------------------------------
// Global Variables ------------------------------------------------------------
// -----------------------------------------------------------------------------


// Pins and devices ********************************************************

// Pins assignment
const byte pin_flow      = 2;
const byte pin_encoder_A = 3;
const byte pin_encoder_B = 4;
const byte pin_button    = 5;
const byte pin_relay     = 6;
const byte pin_tempHumid = A0;
const byte pin_moisture  = A1;

// Analog Data assignment
const byte ad_watering_state              = 0; 
const byte ad_encoder                     = 1; // encoder counts
const byte ad_moisture_min                = 3; // moisture in %
const byte ad_moisture                    = 4; 
const byte ad_wateringTime                = 5; // watering time in s
const byte ad_absorptionTime              = 6; // absorption time (following watering) in s
const byte ad_flow_lmin                   = 7; // flow in L/min
const byte ad_dispensedWaterVolume        = 8; // in L
const byte ad_dispensedWaterVolume_perDay = 9; // in L

const byte ad_temperature_min      = 10; // temperature in °C
const byte ad_temperature_max      = 11;
const byte ad_temperature          = 12;
const byte ad_humidity_min         = 13; // humidity in %
const byte ad_humidity_max         = 14; 
const byte ad_humidity             = 15; 
const byte ad_sunlight_UVIndex_max = 16; // sunlight UV index
const byte ad_sunlight_UVIndex     = 17; 
const byte ad_sunlight_Visible     = 18; // sunlight Visible intensity
const byte ad_sunlight_IR          = 19; // sunlight IR intensity

// Digital Data assignment
const byte dd_warning_temperature_low  = 0;
const byte dd_warning_temperature_high = 1;
const byte dd_warning_humidity_low     = 2;
const byte dd_warning_humidity_high    = 3;
const byte dd_warning_UVIndex_high     = 4;
const byte dd_warning_moisture_low     = 5;
const byte dd_warning_noWater          = 6;
const byte dd_warning                  = 7;
const byte dd_resetWarning_noWater     = 9;  // virtual button
const byte dd_saveSettings             = 10; // virtual button
const byte dd_loadSettings             = 11; // virtual button
const byte dd_enableWatering           = 12;
const byte dd_water                    = 13; // virtual button
const byte dd_isWatering               = 14;


// temperature & humidity sensor DHT 11
//#define DHTTYPE DHT11
DHT dht(pin_tempHumid, DHT11);
float temperature     = 0;
float humidity = 0;

// DHT 11 functions are blocking functions which blocks program during 500ms (measured loop cycle time)
// => solution: prevent it from executing every cycle. Since it's a slow sensor (2s response time), it is 
//    enough execute it only every 2s
HC_Timer timer_dht;


// Soil moisture sensor
float moisture = 0;


// Encoder
Encoder encoder(pin_encoder_A, pin_encoder_B);
byte encoder_variation = 0;


// Sunlight sensor (SI1145)
SI114X sunlight;
float sunlight_UVIndex = 0;
float sunlight_Visible = 0;
float sunlight_IR = 0;

// Flow sensor
volatile int flow_count = 0;
float flow_lhour = 0;   // in l/hour
float flow_lmin = 0;    // in l/min
float flow_lsec = 0;    // in l/s
float dispensedWaterVolume = 0;
float dispensedWaterVolume_perDay = 0;
HC_Timer timer_flow;


// OLED display 0.96 inch
/*#ifdef U8X8_HAVE_HW_SPI
    #include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
    #include <Wire.h>
#endif*/
//U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // SSD1306 and SSD1308Z are compatible
//HC_Timer timer_oled;


// Process ***********************************************************

// Watering
enum WateringState
{
    state_init           = 0,
    state_start          = 1,
    state_watering       = 2,
    state_absorption     = 3,
    state_finish         = 4,
    state_retry          = 5,
    state_noWaterWarning = 6
};
WateringState state = state_init;
bool watering_request_flag = false;
bool watering_ongoing_flag = false;
HC_Timer timer_watering;
HC_Timer timer_absorption;

// Warning flags
bool warning_noWater_flag = false;
bool warning_moistureLow_flag = false;


// Eeprom ************************************************************

// Settings (saved in eeprom, Float Space)
// index
const byte FloatIndex_settings_temperature_min = 0;
const byte FloatIndex_settings_temperature_max = 1;
const byte FloatIndex_settings_humidity_min    = 2;
const byte FloatIndex_settings_humidity_max    = 3;
const byte FloatIndex_settings_UVIndex_max     = 4;
const byte FloatIndex_settings_moisture_min    = 5;
const byte FloatIndex_settings_wateringTime    = 6;
const byte FloatIndex_settings_absorptionTime  = 7;
// value
float settings_temperature_min = 0;    // in °C
float settings_temperature_max = 30;   // in °C
float settings_humidity_min    = 0;    // in %
float settings_humidity_max    = 60;   // in %
float settings_UVIndex_max     = 9;    
float settings_moisture_min    = 100;  // in %
float settings_wateringTime    = 2;    // in s
float settings_absorptionTime  = 5;    // in s

// Settings (save in eeprom, Boolean Space)
// index
const byte BooleanIndex_settingsDetected         = 0; // flag: true if settings already saved once 
const byte BooleanIndex_settings_wateringEnabled = 1;
// value
bool settings_wateringEnabled = true;



// -----------------------------------------------------------------------------
// Setup (called once at power on) ---------------------------------------------
// -----------------------------------------------------------------------------

void setup()
{
    // initialize library
    HC_begin();

    // initialize DHT 11 sensor
    Wire.begin();
    dht.begin();

    // initialize SI1145 sensor
    while(!sunlight.Begin())
        delay(1000);

    // initialize flow measurement: revolution counting
    attachInterrupt(digitalPinToInterrupt(2), flow_counter_interrupt, RISING);

    // pins mode
    pinMode(pin_button, INPUT);
    pinMode(pin_flow,   INPUT);
    pinMode(pin_relay,  OUTPUT);

    // settings reading from eeprom
    if(isSettingsDetected())
        loadSettings();
}



// -----------------------------------------------------------------------------
// Loop (called at each cycle) -------------------------------------------------
// -----------------------------------------------------------------------------

void loop()
{
    // communicate with HITIPanel
    HC_communicate();

    // measure all sensors
    measureAll();

    // read settings from HITIPanel
    readSettingsFromHITIPanel();

    // manage virtual switches
    manageVirtualButtons();

    // manage watering
    manageWatering();
    
    // display data in HITIPanel
    sendDataToHITIPanel();
}



// -----------------------------------------------------------------------------
// Methods ---------------------------------------------------------------------
// -----------------------------------------------------------------------------

// temperature & humidity sensor (DHT 11) *******************************

// measure
void measureTempAndHumidity()
{
    float temperature_hum_val[2] = {0};
    
    // blocking function: blocks program during 500ms
    if(!dht.readTempAndHumidity(temperature_hum_val))
    {
        humidity    = temperature_hum_val[0];  
        temperature = temperature_hum_val[1];  
    }
}


// encoder **************************************************************

// measure
void measureEncoder()
{
    encoder_variation = encoder.readAndReset();
}


// flow sensor (YF-S402) ************************************************

// count rotor revolution
void flow_counter_interrupt()
{
    ++flow_count;
}

// measure water flow and Volume
void measureWaterFlowAndDispensedVolume()
{
    static byte volume_perDay = 0;   // for counting daily volume
    static long seconds_counter = 0; // for counting seconds
    
    // every second
    if(timer_flow.delay(1000))
    {
        // flow calculation (flow sensor YF-S402) -------------------
        // pulse frequency (Hz) = 73*Q, with Q in L/min
        flow_lmin  = (float) flow_count / 73; // L/min
        flow_lsec  = flow_lmin / 60.0;         // L/sec
     
        // reset revolution counter
        flow_count = 0;

        // dispensed water volume calculation -----------------------
        dispensedWaterVolume += flow_lsec; // this variable is reset at each watering
        volume_perDay += flow_lsec;

        // every day
        if(seconds_counter++ > 86399)
        {
            dispensedWaterVolume_perDay = volume_perDay; // record volume per day
            volume_perDay = 0;   // reset variable
            seconds_counter = 0; // reset counter
        }
    }
}


// water pump (Desun Pump DS2500) ***************************************

void waterPumpOn()
{
    digitalWrite(pin_relay, HIGH);
}

void waterPumpOff()
{
    digitalWrite(pin_relay, LOW);
}


// measurement **********************************************************

void measureAll()
{
    // measure temperature & humidity every 2s (except when watering)
    if(timer_dht.delay(2000) && !watering_ongoing_flag)
        measureTempAndHumidity();

    // measure sunlight
    sunlight_UVIndex = (float)sunlight.ReadUV()/100.0;
    sunlight_Visible = sunlight.ReadVisible();
    sunlight_IR      = sunlight.ReadIR();

    // measure soil moisture
    moisture = (float)analogRead(pin_moisture)/7;
    if(moisture > 100)
        moisture = 100;

    // measure water flow and dispensed volume
    measureWaterFlowAndDispensedVolume();
    
    // measure encoder count
    measureEncoder();
}


// warnings *************************************************************

// temperature
bool isWarningDetected_temperature_low() { return temperature < settings_temperature_min; }
bool isWarningDetected_temperature_high(){ return temperature > settings_temperature_max; }

// humidity
bool isWarningDetected_humidity_low()    { return humidity < settings_humidity_min; }
bool isWarningDetected_humidity_high()   { return humidity > settings_humidity_max; }

// UV index
bool isWarningDetected_UVIndex_high()    { return sunlight_UVIndex > settings_UVIndex_max; }

// Soil moisture
bool isWarningDetected_moisture_low()    { return moisture < settings_moisture_min; }

// No water
bool isWarningDetected_noWater()         { return warning_noWater_flag; }


// read settings from HITIPanel ****************************************

void readSettingsFromHITIPanel()
{
    // temperature
    settings_temperature_min = HC_analogDataRead(ad_temperature_min);
    settings_temperature_max = HC_analogDataRead(ad_temperature_max);

    // humidity
    settings_humidity_min = HC_analogDataRead(ad_humidity_min);  
    settings_humidity_max = HC_analogDataRead(ad_humidity_max);  
     
    // sunlight
    settings_UVIndex_max = HC_analogDataRead(ad_sunlight_UVIndex_max);

    // soil moisture
    settings_moisture_min = HC_analogDataRead(ad_moisture_min);

    // watering
    settings_wateringTime   = HC_analogDataRead(ad_wateringTime);
    settings_absorptionTime = HC_analogDataRead(ad_absorptionTime);
    settings_wateringEnabled = HC_digitalDataRead(dd_enableWatering);
}


// send data to HITIPanel *********************************************

void sendDataToHITIPanel()
{
    // temperature
    HC_analogDataWrite(ad_temperature_min, settings_temperature_min);
    HC_analogDataWrite(ad_temperature_max, settings_temperature_max);
    HC_analogDataWrite(ad_temperature,     temperature);

    // humidity
    HC_analogDataWrite(ad_humidity_min,    settings_humidity_min);  
    HC_analogDataWrite(ad_humidity_max,    settings_humidity_max);  
    HC_analogDataWrite(ad_humidity,        humidity);  
     
    // sunlight
    HC_analogDataWrite(ad_sunlight_UVIndex_max, settings_UVIndex_max);
    HC_analogDataWrite(ad_sunlight_UVIndex,     sunlight_UVIndex);
    HC_analogDataWrite(ad_sunlight_Visible,     sunlight_Visible);
    HC_analogDataWrite(ad_sunlight_IR,          sunlight_IR);

    // soil moisture
    HC_analogDataWrite(ad_moisture_min, settings_moisture_min);
    HC_analogDataWrite(ad_moisture,     moisture);

    // watering, water flow and dispensed volume
    HC_analogDataWrite(ad_wateringTime,                settings_wateringTime);
    HC_analogDataWrite(ad_absorptionTime,              settings_absorptionTime);
    HC_analogDataWrite(ad_flow_lmin,                   flow_lmin);
    HC_analogDataWrite(ad_dispensedWaterVolume,        dispensedWaterVolume);
    HC_analogDataWrite(ad_dispensedWaterVolume_perDay, dispensedWaterVolume_perDay);
    //HC_analogDataWrite(ad_watering_state,              state);
    HC_digitalDataWrite(dd_enableWatering,             settings_wateringEnabled);
    HC_digitalDataWrite(dd_isWatering,                 watering_ongoing_flag);    

    // encoder
    //HC_analogDataWrite(ad_encoder, encoder_variation);

    // warnings
    bool warning = isWarningDetected_temperature_low() || isWarningDetected_temperature_high() ||
                   isWarningDetected_humidity_low()    || isWarningDetected_humidity_high()    ||
                   isWarningDetected_UVIndex_high()    || isWarningDetected_moisture_low()     ||
                   isWarningDetected_noWater();

    HC_digitalDataWrite(dd_warning_temperature_low,  isWarningDetected_temperature_low());
    HC_digitalDataWrite(dd_warning_temperature_high, isWarningDetected_temperature_high());
    HC_digitalDataWrite(dd_warning_humidity_low,     isWarningDetected_humidity_low());
    HC_digitalDataWrite(dd_warning_humidity_high,    isWarningDetected_humidity_high());
    HC_digitalDataWrite(dd_warning_UVIndex_high,     isWarningDetected_UVIndex_high());
    HC_digitalDataWrite(dd_warning_moisture_low,     isWarningDetected_moisture_low());
    HC_digitalDataWrite(dd_warning_noWater,          isWarningDetected_noWater());
    HC_digitalDataWrite(dd_warning,                  warning);
}


// Eeprom management ***************************************************

void saveSettings()
{
    HC_eeprom.writeFloat(FloatIndex_settings_temperature_min, settings_temperature_min);
    HC_eeprom.writeFloat(FloatIndex_settings_temperature_max, settings_temperature_max);
    HC_eeprom.writeFloat(FloatIndex_settings_humidity_min,    settings_humidity_min);
    HC_eeprom.writeFloat(FloatIndex_settings_humidity_max,    settings_humidity_max);
    HC_eeprom.writeFloat(FloatIndex_settings_UVIndex_max,     settings_UVIndex_max);
    HC_eeprom.writeFloat(FloatIndex_settings_moisture_min,    settings_moisture_min);
    HC_eeprom.writeFloat(FloatIndex_settings_wateringTime,    settings_wateringTime);
    HC_eeprom.writeFloat(FloatIndex_settings_absorptionTime,  settings_absorptionTime);
    
    HC_eeprom.writeBoolean(BooleanIndex_settings_wateringEnabled,  settings_wateringEnabled);

    // set flag => settings saved at least once
    HC_eeprom.writeBoolean(BooleanIndex_settingsDetected, true);
}

void loadSettings()
{
    settings_temperature_min = HC_eeprom.readFloat(FloatIndex_settings_temperature_min);
    settings_temperature_max = HC_eeprom.readFloat(FloatIndex_settings_temperature_max);
    settings_humidity_min    = HC_eeprom.readFloat(FloatIndex_settings_humidity_min);
    settings_humidity_max    = HC_eeprom.readFloat(FloatIndex_settings_humidity_max);
    settings_UVIndex_max     = HC_eeprom.readFloat(FloatIndex_settings_UVIndex_max);
    settings_moisture_min    = HC_eeprom.readFloat(FloatIndex_settings_moisture_min);
    settings_wateringTime    = HC_eeprom.readFloat(FloatIndex_settings_wateringTime);
    settings_absorptionTime  = HC_eeprom.readFloat(FloatIndex_settings_absorptionTime);
    
    settings_wateringEnabled = HC_eeprom.readBoolean(BooleanIndex_settings_wateringEnabled);

    // display loaded data in HITIPanel
    sendDataToHITIPanel();
}

bool isSettingsDetected()
{
    return HC_eeprom.readBoolean(BooleanIndex_settingsDetected);
}


// Virtual Buttons ****************************************************

void manageVirtualButtons()
{
    manageVirtualButton_saveSettings();
    manageVirtualButton_loadSettings();
    manageVirtualButton_watering();
    manageVirtualButton_resetWarning_noWater();
}

// Settings
void manageVirtualButton_saveSettings()
{
    if(HC_digitalDataRead(dd_saveSettings))
    {
        HC_digitalDataWrite(dd_saveSettings, false); // reset virtual switch
        saveSettings();
    }
}
void manageVirtualButton_loadSettings()
{
    if(HC_digitalDataRead(dd_loadSettings))
    {
        HC_digitalDataWrite(dd_loadSettings, false); // reset virtual switch
        loadSettings();
    }
}

// Watering
void manageVirtualButton_watering()
{
    if(HC_digitalDataRead(dd_water))
    {
        HC_digitalDataWrite(dd_water, false); // reset virtual switch
        watering_request_flag = true; // set request flag
    }
}

// Reset warning: No water
void manageVirtualButton_resetWarning_noWater()
{
    if(HC_digitalDataRead(dd_resetWarning_noWater))
    {
        HC_digitalDataWrite(dd_resetWarning_noWater, false); // reset virtual switch
        warning_noWater_flag = false; // reset warning flag
    }
}


// Watering ************************************************************

void manageWatering()
{
    static byte retry = 0;

    // if watering is disabled
    if(!settings_wateringEnabled)
    {
        // reset flags
        watering_ongoing_flag = 0;
        watering_request_flag = 0;

        // pump OFF
        waterPumpOff();

        state = state_init;
    }
    // if watering is enabled
    else
    {
        switch(state)
        {
            // 0: check if watering is required and possible
            case state_init: 
                // is there water?
                // is watering required? (request OR dry soil)
                if(!warning_noWater_flag && (watering_request_flag || isWarningDetected_moisture_low()))
                    state = state_start;
                break;
    
            // 1: start
            case state_start:
                // set flags
                watering_ongoing_flag = 1;
                retry = 0;
    
                // reset dispensed water volume    
                dispensedWaterVolume = 0;
                    
                state = state_watering;
                break;
    
            // 2: watering
            case state_watering:
                // pump ON
                waterPumpOn();
                
                // water during 2s
                if(timer_watering.delay(settings_wateringTime*1000))
                    state = (flow_lsec < 0.005) ? state_retry : state_absorption; 
                break;
    
            // 3: water absorption
            case state_absorption:
                // pump OFF
                waterPumpOff();
            
                // absorption during 5s
                if(timer_absorption.delay(settings_absorptionTime*1000))
                    state =  isWarningDetected_moisture_low() ? state_watering : state_finish;             
                break;
    
            // 4: finish
            case state_finish:
                // reset flags
                watering_ongoing_flag = 0;
                watering_request_flag = 0;
        
                state = state_init;
                break;
    
            // 5: no water => retry pumping x5
            case state_retry:
                state = (retry ++ < 5) ? state_watering : state_noWaterWarning;
                break;
    
            // 6: no water => warning
            case state_noWaterWarning:
                // pump OFF
                waterPumpOff();

                // set warning flag
                warning_noWater_flag = true; 
                
                state = state_finish;
                break;
        }
    }
}
