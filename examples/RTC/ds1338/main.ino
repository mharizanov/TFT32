
#include "ds1338.h"
#define  CalendarYrToTm(Y)   ((Y) - 1970)
#define  tmYearToY2k(Y)      ((Y) - 30)    // offset is from 2000

ds1338_time_t rtc_time;

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  rtc_time.hours = Hour;
  rtc_time.minutes = Min;
  rtc_time.seconds = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  rtc_time.date = Day;
  rtc_time.month = monthIndex + 1;
  rtc_time.year = CalendarYrToTm(Year);
  return true;
}

void setup() {
  Serial.begin(115200);
  ds1338_init();

  Serial.println((__DATE__, __TIME__));
    
  // get the date and time the compiler was run
  getDate(__DATE__);
  getTime(__TIME__);
  // and configure the RTC with this info
  ds1338_set_time(&rtc_time);

}
void loop() {
  ds1338_get_time(&rtc_time);
  Serial.print(tmYearToY2k(rtc_time.year)); Serial.print("-"); Serial.print(rtc_time.month); Serial.print("-"); Serial.print(rtc_time.date); Serial.print(" ");
  Serial.print(rtc_time.hours); Serial.print(":"); Serial.print(rtc_time.minutes); Serial.print(":"); Serial.println(rtc_time.seconds);
  delay(1000);
}
