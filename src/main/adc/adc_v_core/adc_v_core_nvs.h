#ifndef MAIN_ADC_ADC_V_CORE_ADC_V_CORE_NVS_H_
#define MAIN_ADC_ADC_V_CORE_ADC_V_CORE_NVS_H_

#include "stdint.h"

void adc_v_core_nws_read(const char * name,  uint16_t * to);
void adc_v_core_nws_write(const char * name, uint16_t value);

void adc_v_core_nws_read_postfix(const char * name,  char postfix, uint16_t * to);
void adc_v_core_nws_write_postfix(const char * name, char postfix, uint16_t value);

#endif /* MAIN_ADC_ADC_V_CORE_ADC_V_CORE_NVS_H_ */
