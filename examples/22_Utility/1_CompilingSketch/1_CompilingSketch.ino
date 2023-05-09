#include <HITIComm.h>
#include <HC_MultiTimer.h>
#include <HC_Servo.h>
#include <HC_MotorGroup.h>
#include <HC_SignalFilter.h>

void setup() 
{
  // put your setup code here, to run once:
  HC_begin();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  HC_communicate();
}
