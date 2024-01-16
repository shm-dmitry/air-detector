#ifndef MAIN_ADC_ADC_H_
#define MAIN_ADC_ADC_H_

#include "esp_adc/adc_oneshot.h"

void adc_init();
adc_oneshot_unit_handle_t adc_get_channel();

#endif /* MAIN_ADC_ADC_H_ */
