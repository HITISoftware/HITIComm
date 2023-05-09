/*
 HITIComm examples:  Grove / 3_Grove_TFMiniLiDAR

 This sketch shows how to use HITIPanel software to:
   => monitor data from a Grove ToF LiDAR (TF Mini LiDAR)

 - TF Mini LiDAR              on pin 2,3
 - distance                   on Analog Data 0
 - signal strength            on Analog Data 1

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <TFLidar.h>

SoftwareSerial uart(2, 3);
TFMini SeeedTFMini;
TFLidar SeeedTFLidar(&SeeedTFMini);

// HITI Timer
HC_Timer timer;

void setup()
{
    // initialize HITIComm library
    HC_begin();

    // initialize Grove library
    SeeedTFLidar.begin(&uart, 115200);
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();

    // every 30ms, if data is ready
    if (timer.delay(30) && SeeedTFLidar.get_frame_data())
    {
        // 1) read sensor data (takes up to 25ms)
        // 2) display data in HITIPanel
        HC_analogDataWrite(0, SeeedTFLidar.get_distance());
        HC_analogDataWrite(1, SeeedTFLidar.get_strength());
    }
}
