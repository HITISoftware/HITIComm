/*
 HITIComm examples:  Basics / 6_DigitalData

 This sketch shows how to use HITIPanel software to:
   => control and watch boolean values using HITI Digital Data

 - input 1   on Digital Data 0
 - input 2   on Digital Data 1
 - result    on Digital Data 2

 Copyright © 2021 Christophe LANDRET
 MIT License
 */

#include <HITIComm.h>

// Digital Data assignment
const int dd_input1 = 0;
const int dd_input2 = 1;
const int dd_result = 2;

void setup()
{
    // initialize library
    HC_begin();
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();

    // get inputs from HITIPanel (Command Panel 0 and 1)
    bool input1 = HC_digitalDataRead(dd_input1);
    bool input2 = HC_digitalDataRead(dd_input2);

    // OR operation
    bool result = input1 + input2;

    // display result in HITIPanel (Command Panel 2)
    HC_digitalDataWrite(dd_result, result);
}
