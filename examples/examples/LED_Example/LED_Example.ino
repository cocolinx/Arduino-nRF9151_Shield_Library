// Simple example of LED control.
// Turn on 5 seconds -> Turn off 5 seconds
// 
// by CocoLinx
// 2026-02-03
// --------------------------------------------------

#include <CocoLinx.h>

#define TEST_INTERVAL_SECONDS 1000 * 5	 // 5 seconds

CocoLinx coco;

void setup() 
{
  int32_t ret;
	bool success;

	// serial monitor
	Serial.begin(115200);
	while (!Serial) ;
	Serial.println();
	Serial.println("====== setup() start ======");

  Serial.print("cocolinx begin...");
  success = coco.begin(CocoLinx::SERIAL_HARDWARE);
	if(success == false) 
  {
		Serial.println("error");
		Serial.println("halt forever...");
		while (1);
	}
  else 
  {
    Serial.println("okay");
  }
}

void loop() 
{
	int32_t ret;	
	static uint32_t testCount = 0;

	Serial.print("***** LED_Example [");	
	Serial.print(testCount);
	Serial.println("] *****");

	testCount++;
  
	Serial.print("LED turn on...");
	ret = coco.sysSetLed(true);
	if(ret != 0)
	{	
		Serial.print("error: ");
		Serial.println(ret);
	}
	else
	{
		Serial.println("okay");
		delay(TEST_INTERVAL_SECONDS);

		Serial.print("LED turn off...");
		ret = coco.sysSetLed(false);
		if(ret != 0)
		{
			Serial.print("error: ");
			Serial.println(ret);
		}
		else
		{
			Serial.println("okay");
		}
	}
  delay(TEST_INTERVAL_SECONDS);
}
