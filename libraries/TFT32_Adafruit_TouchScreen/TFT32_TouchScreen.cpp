// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Code under MIT License
#include "Arduino.h"
#include "pins_arduino.h"
#include <pgmspace.h>
#include "driver/adc.h"
#include "TFT32_TouchScreen.h"

int adc1_get_raw(adc1_channel_t channel);

// increase or decrease the touchscreen oversampling. This is a little different than you make think:
// 1 is no oversampling, whatever data we get is immediately returned
// 2 is double-sampling and we only return valid data if both points are the same
// 3+ uses insert sort to get the median value.
// We found 2 is precise yet not too slow so we suggest sticking with it!

#define NUMSAMPLES 8

static MATRIX calibFactors;

static const POINT lcdCalibPoints[ 3 ] =
{
  { TFT_HRES*0.1, TFT_VRES*0.1 },     /* 10%, 10% */
  { TFT_HRES*0.9, TFT_VRES*0.5 },     /* 90%, 50% */
  { TFT_HRES*0.5, TFT_VRES*0.9 },     /* 50%, 90% */
};

static const POINT touchCalibPoints[ 3 ] =
{
  { 101, 300 },       /* 10%, 10% */
  { 729, 585 },       /* 90%, 50% */
  { 409, 882 }, 	  /* 50%, 90% */
};


int TFT32_TouchScreen::setCalibrationMatrix( POINT * displayPtr, POINT * screenPtr, MATRIX * matrixPtr)
{
    int  retValue = OK ;
    
    matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                         ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

    if( matrixPtr->Divider == 0 )
    {
        retValue = NOT_OK ;
    }
    else
    {
        matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                        ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

        matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                        ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;

        matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                        (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                        (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;

        matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                        ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
    
        matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                        ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;

        matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                        (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                        (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;
    }
 
    return( retValue ) ;

} /* end of setCalibrationMatrix() */


int TFT32_TouchScreen::getDisplayPoint( POINT * displayPtr, POINT * screenPtr, MATRIX * matrixPtr )
{
    int  retValue = OK ;

    if( matrixPtr->Divider != 0 )
    {

            /* Operation order is important since we are doing integer */
            /*  math. Make sure you add all terms together before      */
            /*  dividing, so that the remainder is not rounded off     */
            /*  prematurely.                                           */

        displayPtr->x = ( (matrixPtr->An * screenPtr->x) + 
                          (matrixPtr->Bn * screenPtr->y) + 
                           matrixPtr->Cn 
                        ) / matrixPtr->Divider ;

        displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) + 
                          (matrixPtr->En * screenPtr->y) + 
                           matrixPtr->Fn 
                        ) / matrixPtr->Divider ;
    }
    else
    {
        retValue = NOT_OK ;
    }

    return( retValue ) ;
}

int TFT32_TouchScreen::readYP(){
  int raw=0;
  adc1_config_width(ADC_WIDTH_BIT_10);   //Range 0-1023 
  adc1_config_channel_atten(ADC1_GPIO32_CHANNEL, ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
  raw=adc1_get_raw( ADC1_GPIO32_CHANNEL); //Read analog
  return(raw);
}

int TFT32_TouchScreen::readXM(){
  int raw=0;
  adc1_config_width(ADC_WIDTH_BIT_10);   //Range 0-1023 
  adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
  raw=adc1_get_raw( ADC1_GPIO33_CHANNEL); //Read analog
  return(raw);
}

TSPoint::TSPoint(void) {
  x = y = 0;
}

TSPoint::TSPoint(int16_t x0, int16_t y0, int16_t z0) {
  x = x0;
  y = y0;
  z = z0;
}

bool TSPoint::operator==(TSPoint p1) {
  return  ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

bool TSPoint::operator!=(TSPoint p1) {
  return  ((p1.x != x) || (p1.y != y) || (p1.z != z));
}

#if (NUMSAMPLES > 2)
static void insert_sort(int array[], uint8_t size) {
  uint8_t j;
  int save;
  
  for (int i = 1; i < size; i++) {
    save = array[i];
    for (j = i; j >= 1 && save < array[j - 1]; j--)
      array[j] = array[j - 1];
    array[j] = save; 
  }
}
#endif

TSPoint TFT32_TouchScreen::getPoint(void) {
  int x, y, z;
  int samples[NUMSAMPLES];
  uint8_t i, valid;

  valid = 1;

  pinMode(_yp, INPUT);
  pinMode(_ym, INPUT);
  pinMode(_xp, OUTPUT);
  pinMode(_xm, OUTPUT);

  digitalWrite(_xp, HIGH);
  digitalWrite(_xm, LOW);

   for (i=0; i<NUMSAMPLES; i++) {
     samples[i] = readYP();
   }

#if NUMSAMPLES > 2
   insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
   // Allow small amount of measurement noise, because capacitive
   // coupling to a TFT display's signals can induce some noise.
   if (samples[0] - samples[1] < -4 || samples[0] - samples[1] > 4) {
     valid = 0;
   } else {
     samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
   }
#endif

   x = (samples[NUMSAMPLES/2]);

   pinMode(_xp, INPUT);
   pinMode(_xm, INPUT);
   pinMode(_yp, OUTPUT);
   pinMode(_ym, OUTPUT);

   digitalWrite(_ym, LOW);
   digitalWrite(_yp, HIGH);

   for (i=0; i<NUMSAMPLES; i++) {
     samples[i] = readXM();
   }

#if NUMSAMPLES > 2
   insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
   // Allow small amount of measurement noise, because capacitive
   // coupling to a TFT display's signals can induce some noise.
   if (samples[0] - samples[1] < -4 || samples[0] - samples[1] > 4) {
     valid = 0;
   } else {
     samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
   }
#endif

   y = (1023-samples[NUMSAMPLES/2]);

   // Set X+ to ground
   // Set Y- to VCC
   // Hi-Z X- and Y+
   pinMode(_xp, OUTPUT);
   pinMode(_yp, INPUT);

   digitalWrite(_xp, LOW);
   digitalWrite(_ym, HIGH); 
  
   int z1 = readXM(); 
   int z2 = readYP();

   if (_rxplate != 0) {
     // now read the x 
     float rtouch;
     rtouch = z2;
     rtouch /= z1;
     rtouch -= 1;
     rtouch *= x;
     rtouch *= _rxplate;
     rtouch /= 1024;
     
     z = rtouch;
   } else {
     z = (1023-(z2-z1));
   }

   if (! valid) {
     z = 0;
   }

   POINT sample, normalizedSample;
   sample.x = x;
   sample.y = y;
   getDisplayPoint( &normalizedSample, &sample, &calibFactors );

   return TSPoint(normalizedSample.x, normalizedSample.y, z);
}

TFT32_TouchScreen::TFT32_TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym,
			 uint16_t rxplate=0) {
  _yp = yp;
  _xm = xm;
  _ym = ym;
  _xp = xp;
  _rxplate = rxplate;

  pressureThreshhold = 10;
  
/* Initialize touch screen calibration factor matrix  */
setCalibrationMatrix( (POINT*)&lcdCalibPoints,   /* Display coordinates   */
		(POINT*)&touchCalibPoints, /* Touch coordinates     */
		&calibFactors );      /* Calibration factor matrix  */
		
}

int TFT32_TouchScreen::readTouchX(void) {
   pinMode(_yp, INPUT);
   pinMode(_ym, INPUT);
  // digitalWrite(_yp, LOW);
   //digitalWrite(_ym, LOW);
   
   pinMode(_xp, OUTPUT);
   digitalWrite(_xp, HIGH);
   pinMode(_xm, OUTPUT);
   digitalWrite(_xm, LOW);
   
   return (1023-readYP());
}


int TFT32_TouchScreen::readTouchY(void) {
   pinMode(_xp, INPUT);
   pinMode(_xm, INPUT);
  // digitalWrite(_xp, LOW);
  // digitalWrite(_xm, LOW);
   
   pinMode(_yp, OUTPUT);
   digitalWrite(_yp, HIGH);
   pinMode(_ym, OUTPUT);
   digitalWrite(_ym, LOW);
   
   return (1023-readXM());
}


uint16_t TFT32_TouchScreen::pressure(void) {
  // Set X+ to ground
  pinMode(_xp, OUTPUT);
  digitalWrite(_xp, LOW);
  
  // Set Y- to VCC
  pinMode(_ym, OUTPUT);
  digitalWrite(_ym, HIGH); 
  
  // Hi-Z X- and Y+
  //digitalWrite(_xm, LOW);
  pinMode(_xm, INPUT);
 // digitalWrite(_yp, LOW);
  pinMode(_yp, INPUT);
  
  int z1 = readXM(); 
  int z2 = readYP();

  if (_rxplate != 0) {
    // now read the x 
    float rtouch;
    rtouch = z2;
    rtouch /= z1;
    rtouch -= 1;
    rtouch *= readTouchX();
    rtouch *= _rxplate;
    rtouch /= 1024;
    
    return rtouch;
  } else {
    return (1023-(z2-z1));
  }
}
