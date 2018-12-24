#include "ds1338.h"
#include "Wire.h"

void ds1338_write(const uint8_t reg, const uint8_t data)
{
  Wire.beginTransmission(DS1338_TWI_ADR );
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t ds1338_read(const uint8_t reg)
{
  Wire.beginTransmission(DS1338_TWI_ADR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(DS1338_TWI_ADR,1);
  uint8_t data = Wire.read();
  return data;
}

void ds1338_init(void)
{
  Wire.begin();
  ds1338_write(DS1338_CONTROL, DS1338_CONTROL_SETTING);
  // Set the CH bit to zero to enable clock
  uint8_t data = ds1338_read(DS1338_SECONDS);
  data &= ~(1 << DS1338_CH);
  ds1338_write(DS1338_SECONDS, data);
}

void ds1338_get_time(ds1338_time_t * const time)
{
  uint8_t data = ds1338_read(DS1338_SECONDS);
  time->seconds = 10 * ((data & 0b01110000) >> 4) + (data & 0x0F);

  data = ds1338_read(DS1338_MINUTES);
  time->minutes = UNPACK_BCD(data);

  data = ds1338_read(DS1338_HOURS);
  if (data & (1 << DS1338_12_24_HR))
  {
    // 12 hour mode
    time->hours = 10 * ((data & 0b00010000) >> 4) + (data & 0x0F);
    time->is_pm = data & (1 << DS1338_AM_PM);
  } else {
    // 24 hour mode
    time->hours = 10 * ((data & 0b00110000) >> 4) + (data & 0x0F);
  }

  data = ds1338_read(DS1338_DAY);
  // Shift day so that it's 0-6, not 1-7
  time->day = data - 1;

  data = ds1338_read(DS1338_DATE);
  time->date = UNPACK_BCD(data);

  data = ds1338_read(DS1338_MONTH);
  time->month = UNPACK_BCD(data);

  data = ds1338_read(DS1338_YEAR);
  time->year = UNPACK_BCD(data);
}

void ds1338_set_time(const ds1338_time_t * const time)
{
  // Should always write zero to the CS bit to enable the clock
  ds1338_write(DS1338_SECONDS, TO_BCD(time->seconds));
  ds1338_write(DS1338_MINUTES, TO_BCD(time->minutes));

  uint8_t hour_reg = ds1338_read(DS1338_HOURS);
  // Wipe everything apart from the 12/24 hour bit
  hour_reg &= ~(0b00111111);
  if ((hour_reg & (1 << DS1338_12_24_HR)) && (time->is_pm)) {
    // 12 hour mode and it's PM
    hour_reg |= (1 << DS1338_AM_PM);
  }
  hour_reg |= TO_BCD(time->hours);
  ds1338_write(DS1338_HOURS, hour_reg);

  // Shift back day from 0-6 to 1-7.
  ds1338_write(DS1338_DAY, time->day + 1);
  ds1338_write(DS1338_DATE, TO_BCD(time->date));
  ds1338_write(DS1338_MONTH, TO_BCD(time->month));
  ds1338_write(DS1338_YEAR, TO_BCD(time->year));
}
