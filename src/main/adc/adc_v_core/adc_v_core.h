#ifndef MAIN_ADC_ADC_V_CORE_ADC_V_CORE_H_
#define MAIN_ADC_ADC_V_CORE_ADC_V_CORE_H_

#include "stdint.h"
#include "stdbool.h"

typedef double (*adc_v_core__adc2rsro_t)(uint16_t adc, uint16_t calibration_value);
typedef double (*adc_v_core__apply_compensation_t)(double rsro, int8_t compensation_t, uint8_t compensation_h, bool * success);
typedef double (*adc_v_core__rsro2value_t)(double rsro);

typedef struct {
	adc_v_core__adc2rsro_t           adc2rsro;
	adc_v_core__apply_compensation_t apply_compensation;
	adc_v_core__rsro2value_t         rsro2value;
} adc_v_core__functions_t;

typedef struct {
	int8_t min_t;
	int8_t max_t;

	bool temperature;
	bool humidity;
} adc_v_core__compensation_t;

typedef struct {
	uint8_t adc_channel;
	const char * topic_data;
	const char * topic_command;
	const char * name;
	const char * tag;
} adc_v_core__buildconfig_t;

typedef struct {
	adc_v_core__functions_t          functions;
	adc_v_core__compensation_t       compensation;
	adc_v_core__buildconfig_t        buildconfig;
	uint16_t                         calibrate_find_value_x10;
} adc_v_core_setup_t;

void adc_v_core_init(const adc_v_core_setup_t * settings);

#endif /* MAIN_ADC_ADC_V_CORE_ADC_V_CORE_H_ */
