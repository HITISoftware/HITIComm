/*
 HITIComm examples:  Basics / 7_AnalogData

 This sketch shows how to use HITIPanel software to:
   => convert data from a temperature sensor using HITI Analog Data

 - TMP36 temperature sensor       on pin A1
 - Temp value in  V               on Analog Data 0
 - Temp value in °C               on Analog Data 1

 Copyright © 2021 Christophe LANDRET
 MIT License
 */

#include <HITIComm.h>

// Pins assignment
const int pin_TemperatureSensor = A1;

// Analog Data assignment:
const int ad_Temperature_voltage = 0; // sensor values in V
const int ad_Temperature_celsius = 1; // sensor values in °C

void setup()
{
    // initialize library
    HC_begin();
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

    // display converted values in HITIPanel (Command Panel 0 and 1)
    HC_analogDataWrite(ad_Temperature_voltage, voltage);
    HC_analogDataWrite(ad_Temperature_celsius, celsius);
}
