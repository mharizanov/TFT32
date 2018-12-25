#include <lvgl.h>
#include <Ticker.h>
#include <ssl_client.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include <WiFi.h>

const char* ssid     = "*****your ssid*****";
const char* password = "*****your ssid*****";

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_pass = "";
const char* mqtt_intopic = "in/#";
const char* mqtt_outtopic = "out";

#include "EasyBuzzer.h"
#define BUZZER_PIN 12

#include "TFT32_Adafruit_ILI9341.h"
#define cs   15  // goes to TFT CS
#define dc   2  // goes to TFT DC
#define rst  27  // goes to TFT RESET
#define TFT_BKLT 4
TFT32_Adafruit_ILI9341 tft = TFT32_Adafruit_ILI9341(cs, dc, rst);

#include "TFT32_TouchScreen.h"
#define XP 26
#define XM 33
#define YM 25
#define YP 32
TFT32_TouchScreen ts = TFT32_TouchScreen(XP, YP, XM, YM, 364);

void lvgl_init();
void create_control_gui();
void mqtt_setup();
void mqtt_loop();

void setup() {
  Serial.begin(115200);

  tft.begin(); // TFT init
  tft.setRotation(1); // Landscape

  pinMode(TFT_BKLT, OUTPUT);
  digitalWrite(TFT_BKLT, LOW); // Backlight ON

  EasyBuzzer.setPin(BUZZER_PIN); // Buzzer init

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt_setup();

  lvgl_init(); // LittleVGL init
  create_control_gui(); // Create GUI
}

void loop() {

  delay(10);
  EasyBuzzer.update();
}
