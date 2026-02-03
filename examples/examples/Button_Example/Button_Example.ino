// Simple example of Button count check.
// 
// 
// by CocoLinx
// 2026-02-03
// --------------------------------------------------

#include <CocoLinx.h>

#define TEST_INTERVAL_MS 1000 * 5 // 5 seconds

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
	static uint32_t millisPrev = -(1000 * 60 * 5); // start first test on first loop
  static uint32_t clickCount;

	int32_t ret;	

	if((millis() - millisPrev) >= TEST_INTERVAL_MS)
	{
		Serial.print("button click count read...");	
		ret = coco.sysGetBtnCount(&clickCount);
		if(ret != 0)
		{
			Serial.print("error: ");
			Serial.println(ret);
		}
		else
		{
			Serial.println("okay");
			Serial.print(clickCount, DEC);
			Serial.println(" counts");
		}
		millisPrev = millis();
	}
}
