#include <CocoLinx.h>

#define TEST_LTE_PLMN_SELECT 	CocoLinx::PLMN_SKT
#define TEST_INTERVAL_SECONDS 1000 * 60 * 5	 // 3minutes

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
  int32_t ret;
  const uint8_t ipv4[4] = {43, 200, 166, 133};
	const uint16_t port = 7777;

  // lte connection
  ret = coco.lteConnect(180000, TEST_LTE_PLMN_SELECT);

  if(ret != 0) 
  {
    Serial.print("LTE Connection error: ");
    Serial.println(ret);
    delay(500);
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
      delay(TEST_INTERVAL_SECONDS);
    }
  }
}