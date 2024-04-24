#include "o2a2.h"

#include "sdkconfig.h"

#include "../adc_v_core/adc_v_core.h"
#include "../../log/log.h"

#define O2A2_STANDART_O2_VALUE_X10   209

double o2a2_adc2rsro(uint16_t adc, uint16_t calibration_value) {
	// calibration_value - is an O2 value at O2A2_STANDART_O2_VALUE_X10 %.
	return (((double) adc) * (double)O2A2_STANDART_O2_VALUE_X10) / ((double) calibration_value);
}

/*
Approximate using polynom f(x) = ax^3 + bx^2 + cx + d
Points to resolve [A-D] taken from datasheet graph.
Result:
A = -139/4620000
B = 1/61600
C = 23057/92400
D = 29335/308
 */
double o2a2_apply_compensation(double rsro, int8_t compensation_t, uint8_t compensation_h, bool * success) {
	double _t = (double)compensation_t;

	double pc = -139.0 * (_t * _t * _t) / 4620000.0 +
			    (_t * _t) / 61600.0 +
				23057.0 * _t / 92400.0 +
				29335.0 / 308.0;

	if (pc > 90.0 && pc < 105.0) {
		rsro = (rsro / pc) * 100.0;
		*success = true;
	} else {
		*success = false;
		ESP_LOGE(LOG_O2A2, "Bad compensations: T = %d; o2_x10 = %f, ; temp_delta = %f",
				compensation_t, rsro, pc);
	}

	return rsro;
}

double o2a2_rsro2value(double rsro) {
	return rsro / 10.0;
}

void o2a2_init() {
	adc_v_core_setup_t setup = {
		.calibrate_find_value_x10 = O2A2_STANDART_O2_VALUE_X10,

		.compensation = {
			.min_t = -20,
			.max_t = 50,
			.temperature = true,
			.humidity = false
		},

		.buildconfig = {
			.adc_channel = CONFIG_O2A2_ADC_CHANNEL,
			.topic_data = CONFIG_O2A2_TOPIC_DATA,
			.topic_command = CONFIG_O2A2_TOPIC_COMMAND,
			.name = "o2",
			.tag = LOG_O2A2
		},

		.functions = {
			.adc2rsro = &o2a2_adc2rsro,
			.apply_compensation = &o2a2_apply_compensation,
			.rsro2value = &o2a2_rsro2value
		}
	};

	adc_v_core_init(&setup);
}

