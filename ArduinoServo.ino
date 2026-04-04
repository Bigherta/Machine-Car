#include "include.h"

int ps2_ok;
void setup() 
{
	pinMode(KEY,INPUT_PULLUP);
	InitPWM();
	InitTimer2();
	InitUart1();
  	ps2_ok = InitPS2();
	InitFlash();
	InitMemory();
}

void loop() 
{
	TaskRun(ps2_ok); 
}
