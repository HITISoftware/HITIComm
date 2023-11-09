#include <HITIComm.h>

void setup()
{
    // initialize HITIComm library
    HC_begin();
}

void loop()
{
    // communicate with HITIPanel software
    HC_communicate();

    // ...
}
