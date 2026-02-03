// Simple example application that GNSS measurement
// gnss start -> gnss read 3 minutes -> gnss stop -> sleep 5 minutes -> gnss start
// 
// by CocoLinx
// 2026-02-03
// --------------------------------------------------

#include <CocoLinx.h>

#define TEST_INTERVAL_MS 1000 * 60 * 5	 // 5 minutes

CocoLinx coco;
CocoLinx::GnssData gnssData;

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
  static uint32_t millisPrev = -(1000 * 60 * 5);
  static uint32_t testCount = 0;

	int32_t ret;	

  if((millis() - millisPrev) >= TEST_INTERVAL_MS)
  {
    Serial.print("***** GNSS_Example [");	
    Serial.print(testCount);
    Serial.println("] *****");

    testCount++;
    
    Serial.print("gnss connect...");
    ret = coco.gnssStart();
    if(ret != 0) 
    {
      Serial.print("error: ");
      Serial.println(ret);
      coco.gnssStop();
    }
    else 
    {
      Serial.println("okay");

      int pollCount = 60; // 3000ms * 60 = 180 seconds
      
      while(pollCount--) 
      {
        Serial.print("gnss read...");
        ret = coco.gnssRead(&gnssData);
        if(ret != 0) 
        {
          Serial.print("error: ");
          Serial.println(ret); // error code: -28 means not fixed.
        } 
        else 
        {
          Serial.println("okay");
          Serial.print("elapsed_ms: ");
          Serial.println(gnssData.elapsed_ms);
          Serial.print("satellites_visible: ");
          Serial.println(gnssData.satellites_visible);
          Serial.print("satellites_used: ");
          Serial.println(gnssData.satellites_used);
          Serial.print("latitude: ");
          Serial.println(gnssData.latitude);
          Serial.print("longitude: ");
          Serial.println(gnssData.longitude);		
        }
        if(pollCount > 0) delay(3000);
      }
    }
    
    Serial.print("gnss stop...");
    ret = coco.gnssStop();
    if(ret != 0)
    {
      Serial.print("error: ");
      Serial.println(ret);
    }
    else
    {
      Serial.println("okay");
    }
    millisPrev = millis();
  }
}
