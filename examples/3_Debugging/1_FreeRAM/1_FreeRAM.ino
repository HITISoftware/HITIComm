/*
HITIComm examples:  Debugging / 2_FreeRAM

 This sketch shows how to measure your Free RAM with HITIPanel

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
    // measure SRAM on Probe 0
    HC_communicate();
}
