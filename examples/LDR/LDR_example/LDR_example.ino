#include <driver/adc.h>
int readLightSensor(){
  adc1_config_width(ADC_WIDTH_BIT_12);   //Range 0-1023 
  adc1_config_channel_atten(ADC1_GPIO34_CHANNEL, ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
  return adc1_get_raw( ADC1_GPIO34_CHANNEL); //Read analog
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  }

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(readLightSensor());
  delay(500);
}
