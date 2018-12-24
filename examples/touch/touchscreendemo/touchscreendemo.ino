// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// This demo code returns raw readings, public domain

#include "Adafruit_GFX.h"
#include "TFT32_Adafruit_ILI9341.h"
#include "TFT32_TouchScreen.h"
#include <Adafruit_NeoPixel.h>

#define BUTTON_PIN   0
#define PIXEL_PIN    17    // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT  1
Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define cs   15  // goes to TFT CS
#define dc   2  // goes to TFT DC
#define rst  27  // goes to TFT RESET
#define TFT_BKLT 4
TFT32_Adafruit_ILI9341 tft = TFT32_Adafruit_ILI9341(cs, dc, rst);

#define XP 26
#define XM 33
#define YM 25
#define YP 32

TFT32_TouchScreen ts = TFT32_TouchScreen(XP, YP, XM, YM, 364);

#define PENRADIUS  4
// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

int color = RED;
int showType = 0;
bool oldState = HIGH;

void setup(void) {
  Serial.begin(115200);
  
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(BLACK);

  pinMode(TFT_BKLT, OUTPUT);
  digitalWrite(TFT_BKLT, LOW);

  pinMode(BUTTON_PIN, INPUT);

  neopixel.begin();
  neopixel.setPixelColor(0, neopixel.Color(80, 0, 0)); // Red
  neopixel.show();

  tft.setCursor(28, 100);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("Touch to paint");

  tft.setCursor(70, 185);
  tft.setTextColor(ILI9341_RED);tft.setTextSize(1);
  tft.println("Press this button to change pen color -->");
}

void loop(void) {
  // a point object holds x y and z coordinates
  TSPoint p = ts.getPoint();

  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if (p.z > ts.pressureThreshhold) {
    tft.fillCircle(p.x, p.y, PENRADIUS, color);
  }

  // Get current button state.
  bool newState = digitalRead(BUTTON_PIN);

  // Check if state changed from high to low (button press).
  if (newState == LOW && oldState == HIGH) {
    // Short delay to debounce button.
    delay(20);
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if (newState == LOW) {
      Serial.println("Pressed!");
      showType++;
      if (showType > 2)
        showType = 0;
      switch (showType) {
        case 0: neopixel.setPixelColor(0, neopixel.Color(80, 0, 0)); // Red
          color = RED;
          break;
        case 1: neopixel.setPixelColor(0, neopixel.Color(0, 80, 0)); // Green
          color = GREEN;
          break;
        case 2: neopixel.setPixelColor(0, neopixel.Color(0, 0, 80)); // Blue
          color = BLUE;
          break;
      }
    }
  }
  neopixel.show();
  // Set the last button state to the old state.
  oldState = newState;
  delay(5);
}
