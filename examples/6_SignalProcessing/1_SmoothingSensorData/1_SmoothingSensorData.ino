/*
 HITIComm examples:  Basics / 6_AnalogData

 This sketch shows how to use a HITI Signal Filter to: 
   => average and smooth data from a temperature sensor
   => virtually increase the resolution
 
 - TMP36 temperature sensor       on pin A1
 - Temp value in  V               on HITI Data A0
 - Temp value in °C               on HITI Data A1
 - Trigger for signal filtering   on HITI Data 0 
   
 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_SignalFilter.h>

// pins assignment
const int pin_TemperatureSensor = A1;

// Analog Data assignment
const int ad_Temperature_voltage = 0; // Temperature reading (converted in V)
const int ad_Temperature_celsius = 1; // Temperature reading (converted in °C)

// Digital Data assignment:
const int dd_ActivateFilter = 0; // Virtual Switch

// HITI Signal Filter
HC_SignalFilter filter;


void setup()
{
    // initialize library
    HC_begin();

    // increase the buffer size to improve filtering (default: 10, max: 255)
    filter.setBufferSize(50);
}


void loop()
{
    // communicate with HITIPanel
    HC_communicate();
    
    // read sensor raw values
    int rawData = analogRead(pin_TemperatureSensor);
    
    // convert to voltage V    (using 5V power supply)
    float voltage = ((float)rawData / 1024.0) * 5.0;
    
    // convert to degrees °C   (TMP36 sensor => +10mV/°C, 750mV at 25°C)
    float celsius = (voltage - 0.5) * 100;
    
    // if the Virtual Switch is activated from HITIPanel
    if (HC_digitalDataRead(dd_ActivateFilter) == HIGH)
    {
        // filter value
        celsius = filter.average(celsius);
    }
    
    // display values in HITIPanel
    HC_analogDataWrite(ad_Temperature_voltage, voltage);
    HC_analogDataWrite(ad_Temperature_celsius, celsius);
}
