#if not defined(ESP32)
#error Compile for ESP32!!
#endif

#include "Adafruit_GFX.h"
#include "TFT32_Adafruit_ILI9341.h"

#define _cs   15  // goes to TFT CS
#define _dc   2  // goes to TFT DC
#define _rst  27  // goes to TFT RESET
#define TFT_BKLT 4

#define RF69_IRQ              16 
#define RF69_SPI_CS           5  
#include "RFM69X.h"

#define IS_RFM69HW_HCW        true //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

#define NODEID      1
#define NETWORKID   100
#define FREQUENCY   RF69_433MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define KEY         "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define POWER_LEVEL 31 // power output ranges from 0 (5dBm) to 31 (20dBm)
#define ACK_TIME    30  // # of ms to wait for an ack

#define SERIAL_BAUD 115200

RFM69X radio = RFM69X(RF69_SPI_CS, RF69_IRQ, IS_RFM69HW_HCW, digitalPinToInterrupt(RF69_IRQ));

TFT32_Adafruit_ILI9341 tft = TFT32_Adafruit_ILI9341(_cs, _dc, _rst);

bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

#pragma pack(1) //https://www.arduinoforum.de/arduino-Thread-struct-wird-auf-einem-ESP8266-anders-behandelt-wie-auf-einem-Arduino
typedef struct {
  uint16_t nodeId; //store this nodeId
  uint32_t uptime; //uptime in ms
  uint32_t  temp;  //temperature maybe?
} Payload;
Payload theData;
#pragma pack(0) 
 
template <typename T> unsigned int serialPrintAnything (const T& value)
{
  const byte * p = (const byte*) &value;
  unsigned int i;
  Serial.print(" size: "); Serial.print(sizeof value); Serial.print("; byte-data = ");
  for (i = 0; i < sizeof value; i++) {
    Serial.print("0x");
    if (*p <= 15) Serial.print("0");
    Serial.print(*p++, HEX);
    Serial.print(" ");
  }
  Serial.println();
  return i;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  pinMode(TFT_BKLT, OUTPUT);
  digitalWrite(TFT_BKLT,LOW);
  
  tft.setTextSize(2);tft.println("Starting...");
  tft.setTextSize(1);
  yield();

  if(!radio.initialize(FREQUENCY,NODEID,NETWORKID)) {
    Serial.println("RFM69 could not be initialised!");
    
    while(1){
          delay(50);
    }
  }

  
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.setPowerLevel(POWER_LEVEL); 
  radio.encrypt(KEY);
  radio.promiscuous(promiscuousMode);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  
  tft.println(buff);
}

byte ackCount=0;
void loop() {

  if (radio.receiveDone())
  {       
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    Serial.print(" [RX_RSSI:");Serial.print(radio.readRSSI());Serial.print(" dB]");

    if (promiscuousMode)
    {
      Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }

    if (radio.DATALEN != sizeof(Payload)){
      Serial.print("Invalid payload received, not matching Payload struct!");
      Serial.print(" Expected: "); Serial.print(sizeof(Payload)); Serial.print("Received: ");  Serial.print(radio.DATALEN);
    }
    else
    {
      
      theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
       serialPrintAnything(theData);

      Serial.print(" nodeId=");
      Serial.print(theData.nodeId, HEX);
      Serial.print(" uptime=");
      Serial.print(theData.uptime, HEX);
      Serial.print(" temp=");
      Serial.print(theData.temp, HEX);
    }
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 5, ACK_TIME))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    Serial.println();

  tft.print("From Node: ");tft.print(radio.SENDERID);
  tft.print("; RSSI: ");tft.println(radio.readRSSI());
 
  }
}

