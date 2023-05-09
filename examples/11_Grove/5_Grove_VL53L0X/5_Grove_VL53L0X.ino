/*
 HITIComm examples:  Grove / 5_Grove_VL53L0X

 This sketch shows how to use HITIPanel software to:
   => monitor data from a Grove ToF distance sensor (VL53L0X)

 - VL53L0X sensor    on I2C bus
 - distance          on Analog Data 0

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include "Seeed_vl53l0x.h"

Seeed_vl53l0x VL53L0X;

// HITI Timer
HC_Timer timer;

void setup() 
{
    // initialize HITIComm library
    HC_begin();

    // initialize VL53L0X device
    VL53L0X_Error Status = VL53L0X.VL53L0X_common_init();

    // request continuous ranging
    VL53L0X.VL53L0X_continuous_ranging_init();
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();

    // create a data set and initialize data values to 0
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    memset(&RangingMeasurementData, 0, sizeof(VL53L0X_RangingMeasurementData_t));

    // perform continuous measurement
    VL53L0X_Error Status = VL53L0X.PerformContinuousRangingMeasurement(&RangingMeasurementData);

    // if no error during measurement
    if (Status == VL53L0X_ERROR_NONE)
    {
        // display value in HITIPanel
        if (RangingMeasurementData.RangeMilliMeter >= 2000)
            HC_analogDataWrite(0, -1); // out of range
        else
            HC_analogDataWrite(0, RangingMeasurementData.RangeMilliMeter);
    }
}
