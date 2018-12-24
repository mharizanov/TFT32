// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Code under MIT License

#ifndef _TFT32_ADAFRUIT_TOUCHSCREEN_H_
#define _TFT32_ADAFRUIT_TOUCHSCREEN_H_
#include <stdint.h>


#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(TEENSYDUINO) || defined(__AVR_ATmega2560__)
typedef volatile uint8_t RwReg;
#endif
#if defined(ARDUINO_STM32_FEATHER)
typedef volatile uint32 RwReg;
#endif
#if defined(ARDUINO_FEATHER52) || defined(ESP32)
typedef volatile uint32_t RwReg;
#endif

#if defined (__AVR__) || defined(TEENSYDUINO) || defined(ARDUINO_ARCH_SAMD)
  #define USE_FAST_PINIO
#endif

#define TFT_HRES 320
#define TFT_VRES 240

#ifndef		OK
	#define		OK		        0
	#define		NOT_OK		   -1
#endif

#define	INT32 double
typedef struct Point { INT32 x, y ; } POINT ;
typedef struct Matrix {INT32 An, Bn, Cn, Dn, En, Fn, Divider ; } MATRIX ;

class TSPoint {
 public:
  TSPoint(void);
  TSPoint(int16_t x, int16_t y, int16_t z);
  
  bool operator==(TSPoint);
  bool operator!=(TSPoint);

  int16_t x, y, z;
};

class TFT32_TouchScreen {
 public:
  TFT32_TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym, uint16_t rx);

  bool isTouching(void);
  uint16_t pressure(void);
  int readTouchY();
  int readTouchX();
  int readXM();
  int readYP();  
  TSPoint getPoint();
  int16_t pressureThreshhold;

private:
  uint8_t _yp, _ym, _xm, _xp;
  uint16_t _rxplate;
  int setCalibrationMatrix( POINT * displayPtr,
                            POINT * screenPtr,
                            MATRIX * matrixPtr);

   int getDisplayPoint( POINT * displayPtr,
						POINT * screenPtr,
						MATRIX * matrixPtr );
						
  volatile RwReg *xp_port, *yp_port, *xm_port, *ym_port;
  RwReg xp_pin, xm_pin, yp_pin, ym_pin;

};

#endif
