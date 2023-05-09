/*
HITIComm examples:  Debugging / 3_LoopCycleTime

 This sketch shows how to measure the loop() cycle time with HITIPanel

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

void setup()
{
    // initialize library
    HC_begin();
}

void loop()
{
    // measure loop() cycle time
    HC_communicate();
}
