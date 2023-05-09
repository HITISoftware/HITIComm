 /*
 HITIComm examples:  Grove / 4_Grove_QTouch

 This sketch shows how to use HITIPanel software to:
   => monitor touched keys from a Grove touch input device (QTouch)

 - QTouch device      on I2C bus
 - Key 0              on Digital Data 0
 - Key 1              on Digital Data 1
 - Key 2              on Digital Data 2

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>
#include <HC_MultiTimer.h>

#include <Wire.h>
#include <Seeed_QTouch.h>

// Digital Data assignment
const int key0 = 0;
const int key1 = 1;
const int key2 = 2;

// HITI MultiTimer with 3 timers
HC_MultiTimer timers(3);

void setup()
{
    // initialize HITIComm library
    HC_begin();
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();

    // sensors reading takes up to 10ms each
    // => read them one after the other, every 20ms
    // => display them in HITIPanel
    if(timers.delay(0, 20))
        HC_digitalDataWrite(key0, QTouch.isTouch(key0));

    if(timers.delay(1, 20))
        HC_digitalDataWrite(key1, QTouch.isTouch(key1));

    if(timers.delay(2, 20))
        HC_digitalDataWrite(key2, QTouch.isTouch(key2));
}
