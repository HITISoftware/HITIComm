/*
 HITIComm examples:  Basics / 9_CodeID

 This sketch shows how to use HITIPanel software to:
   => identify a loaded code using a Code ID

 Important notes:
   1) A Code ID is made of 2 strings: a Name and a Version.
   2) Both strings must be placed in Program Memory by using "const ... PROGMEM" keywords

 Copyright © 2021 Christophe LANDRET
 MIT License
*/

#include <HITIComm.h>

// code ID
const char code_name[]    PROGMEM = "My running sketch";
const char code_version[] PROGMEM = "1.0.0";

void setup()
{
    // initialize library
    HC_begin();

    // set code ID
    HC_codeName(code_name);
    HC_codeVersion(code_version);
}

void loop()
{
    // communicate with HITIPanel
    HC_communicate();
}
