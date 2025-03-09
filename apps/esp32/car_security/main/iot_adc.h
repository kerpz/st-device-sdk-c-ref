#ifndef _IOT_ESP_ADC_H_
#define _IOT_ESP_ADC_H_

#define GPIO_ADC_VOLTAGE ADC1_CHANNEL_0 /* GPIO36 */

void adc_setup(void);
int get_adc_readings(float *voltage);

#endif /* _IOT_ESP_ADC_H_ */