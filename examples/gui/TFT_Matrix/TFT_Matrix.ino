// A fun MATRIX-like screen demo of scrolling
// Screen will flicker initially until fully drawn
// then scroll smoothly

#define ILI9341_VSCRDEF 0x33
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <SPI.h>
SPIClass SPI2(HSPI);

#define _cs   15  // goes to TFT CS
#define _dc   2  // goes to TFT DC
#define _mosi 13  // goes to TFT MOSI
#define _sclk 14  // goes to TFT SCK/CLK
#define _rst  27  // goes to TFT RESET
#define _miso     // leave unconnected
#define TFT_BKLT 4

#define TEXT_HEIGHT 8 // Height of text to be printed and scrolled
#define BOT_FIXED_AREA 0  // Number of lines in bottom fixed area (lines counted from bottom of screen)
#define TOP_FIXED_AREA 0 // Number of lines in top fixed area (lines counted from top of screen)

Adafruit_ILI9341 tft = Adafruit_ILI9341(_cs, _dc, _rst);

uint16_t yStart = TOP_FIXED_AREA;
uint16_t yArea = 320 - TOP_FIXED_AREA - BOT_FIXED_AREA;
uint16_t yDraw = 320 - BOT_FIXED_AREA - TEXT_HEIGHT;
byte pos[42];
uint16_t xPos = 0;

void write_cmd(byte b) {
  digitalWrite(_dc,LOW);
  digitalWrite(_cs, LOW);
  SPI2.write(b);
  digitalWrite(_cs,HIGH);
}

void write_data(byte b) {
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
  SPI2.write(b);
  digitalWrite(_cs,HIGH);
}

void setup() {
  Serial.begin(115200);
  pinMode(TFT_BKLT, OUTPUT);
  digitalWrite(TFT_BKLT,LOW);

  SPI2.setHwCs(true);
  SPI2.begin();
  SPI2.setFrequency(80000000);
  tft.begin(80000000,SPI2);
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);
  setupScrollArea(TOP_FIXED_AREA, BOT_FIXED_AREA);
}

void loop(void) {
  // First fill the screen with randomt streaks of characters
  for (int j = 0; j < 400; j += TEXT_HEIGHT) {
    int color;
    for (int i = 0; i < 40; i++) {
      if (pos[i] > 20) pos[i] -= 3; // Rapid fade initially brightness values
      if (pos[i] > 0) pos[i] -= 1; // Slow fade later
      if (random(20) == 1) pos[i] = 63; // ~1 in 20 probability of a new character
      color = pos[i] << 5; // Set the character brightness
      if (pos[i] == 63) color = ILI9341_WHITE; // Draw white character
      tft.drawChar( xPos, yDraw, random(32, 128), color, ILI9341_BLACK,  1); // Draw the character
      xPos+=8;    
    }
    yDraw = scroll_slow(TEXT_HEIGHT, 8); // Scroll, 14ms per pixel line
    xPos = 0;
  }

  //tft.setRotation(2);
  //tft.setTextColor(63 << 5, ILI9341_BLACK);
  //tft.drawCentreString("MATRIX",120,60,4);
  //tft.setRotation(0);

  // Now scroll smoothly forever
  while (1) yDraw = scroll_slow(320,8); // Scroll 320 (arbitrary number!) lines, 10ms per line

}

void setupScrollArea(uint16_t TFA, uint16_t BFA) {
  write_cmd(ILI9341_VSCRDEF); // Vertical scroll definition
  write_data(TFA >> 8);
  write_data(TFA);
  write_data((320 - TFA - BFA) >> 8);
  write_data(320 - TFA - BFA);
  write_data(BFA >> 8);
  write_data(BFA);
}

int scroll_slow(int lines, int wait) {
  int yTemp = yStart;
  for (int i = 0; i < lines; i++) {
    yStart++;
    if (yStart == 320 - BOT_FIXED_AREA) yStart = TOP_FIXED_AREA;
    scrollAddress(yStart);
    delay(wait);
  }
  return  yTemp;
}

void scrollAddress(uint16_t VSP) {
  write_cmd(ILI9341_VSCRSADD); // Vertical scrolling start address
  write_data(VSP >> 8);
  write_data(VSP);
}



