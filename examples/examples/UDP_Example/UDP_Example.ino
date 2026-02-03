// Simple example of UDP application.
// LTE connect -> UDP socket open -> UDP connect -> Data send -> echo -> Data recv
//
// by CocoLinx
// 2026-02-03
// --------------------------------------------------

#include <CocoLinx.h>

#define TEST_LTE_PLMN_SELECT 	CocoLinx::PLMN_SKT
#define TEST_INTERVAL_MS 1000 * 60 * 5	 // 5 minutes

CocoLinx coco;

void setup() {
	int32_t ret;
	bool success;

	// serial monitor
	Serial.begin(115200);
	while (!Serial) ;
	Serial.println();
	Serial.println("====== setup() start ======");

	// cocolinx begin (hardware:serial1 or software)
	Serial.print("cocolinx begin...");
	success = coco.begin(CocoLinx::SERIAL_HARDWARE);
	if(success == false) {
		Serial.println("error");
		Serial.println("halt forever...");
		while (1);
	} else {
		Serial.println("okay");
	}
}

void loop() {
  static uint32_t millisPrev = -(1000 * 60 * 5);
	static uint32_t testCount = 0;

  int32_t ret;

  if((millis() - millisPrev) >= TEST_INTERVAL_MS)
  {
		Serial.print("***** UDP_Example [");	
		Serial.print(testCount);
		Serial.println("] *****");

    testCount++;
    
    const uint8_t ipv4[4] = {43, 200, 166, 133};
    const uint16_t port = 7777;

    // lte connection
    ret = coco.lteConnect(180000, TEST_LTE_PLMN_SELECT);

    if(ret != 0) 
    {
      Serial.print("LTE Connection error: ");
      Serial.println(ret);
    }
    else 
    {
      Serial.println("udp echo server: 43, 200, 166, 133:7777 by cocolinx");
      Serial.print("udp open...");

      ret = coco.udpOpen(ipv4, port);
      if(ret != 0) 
      {
        Serial.print("UDP open error: ");
        Serial.println(ret);
        coco.udpClose();
      } 
      else 
      {
        Serial.println("okay");

        // send...
        char udpTx[] = "hello udp world~~~";
        Serial.print("udp send...");
        ret = coco.udpSend(udpTx, strlen(udpTx));
        if(ret != 0) 
        {
          Serial.print("UDP send error: ");
          Serial.println(ret);
        } else 
        {
          Serial.println("okay");
          Serial.print("sent> ");
          Serial.println(udpTx);
        }

        // recv...for 15secs
        Serial.println("udp recv data(max 15secs)...start");
        char udpRx[64];
        int rxcnt = 30; // 500ms * 30 = 15secs
        while(rxcnt--) 
        {
          int rxsize = coco.udpRecv(udpRx, sizeof(udpRx) - 1);
          if(rxsize > 0) 
          {
            Serial.print("recv> ");
            udpRx[rxsize] = '\0';
            Serial.println(udpRx);
            break;
          } 
          else 
          {
            if(rxcnt > 0) delay(500); // delay
          }
        }
      }
    }
    millisPrev = millis();
  }
}