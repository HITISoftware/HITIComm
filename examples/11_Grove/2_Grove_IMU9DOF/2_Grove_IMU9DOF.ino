/*
 HITIComm examples:  Grove / 2_Grove_IMU9DOF

 This sketch shows how to use HITIPanel software to:
   => monitor raw data from a Grove 9-axis tracking device (IMU 9DOF)

 - IMU 9DOF          on I2C bus
 - raw data          on Analog Data 0-8

 Copyright © 2021 Christophe LANDRET
 MIT License
*/


#include <HITIComm.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;
I2Cdev   I2C_M;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// HITI Timer
HC_Timer timer;


void setup() 
{
  // initialize HITIComm library
  HC_begin();
  
  // initialize I2C and Grove library
  Wire.begin();
  accelgyro.initialize();
}


void loop() 
{
    // communicate with HITIPanel
    HC_communicate();

    // every 30ms
    if(timer.delay(30))
    {
      // read sensor data (takes up to 25ms)
      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    }
    
    // display data in HITIPanel
    HC_analogDataWrite(0, (float) ax * 2    / 32768); // +/- 2g (16 bit)
    HC_analogDataWrite(1, (float) ay * 2    / 32768); 
    HC_analogDataWrite(2, (float) az * 2    / 32768); 
    HC_analogDataWrite(3, (float) gx * 250  / 32768); // +/- 250°/s (16 bit)
    HC_analogDataWrite(4, (float) gy * 250  / 32768); 
    HC_analogDataWrite(5, (float) gz * 250  / 32768); 
    HC_analogDataWrite(6, (float) mx * 4800 / 32768); // +/- 4800uT (16 bit)
    HC_analogDataWrite(7, (float) my * 4800 / 32768);
    HC_analogDataWrite(8, (float) mz * 4800 / 32768);
}
