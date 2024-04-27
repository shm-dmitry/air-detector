#include "mq136.h"

#include "math.h"

#include "sdkconfig.h"

#include "../adc_v_core/adc_v_core.h"
#include "../../log/log.h"

#define MQ136_DEBUG_COMPENSATIONS 		false

#define MQ136_AM		   				((double)4095.0)
#define MQ136_COMPENSATION_DEFAULT_H	33.0

// Math:
// As  = ADC value for current measurement (adc variable)
// Ao  = ADC value for zero-measurement    (calibrated value)
// Am  = 4095 (max ADC value)

// Rs/Ro = (Ao / As) * (Am - As) / (Am - Ao) = (Ao * (Am - As)) / (As * (Am - Ao))
double mq136_adc2rsro(uint16_t adc, uint16_t calibration_value) {
	double temp = (double)adc * (MQ136_AM - (double)calibration_value);
	if (temp > -0.001 && temp < 0.001) { // check for a division-by-zero
		ESP_LOGW(LOG_MQ136, "div by zero. ADC = %d", adc);
		return 0;
	}

	return ((double)calibration_value * (MQ136_AM - (double)adc)) / temp;
}

// Temperature compensation: T = T+20; and used next formula
// y = A*e^(-Bx) + C*e^(-Dx) + F
// Where:
// A = 9.4232
// B = 0.0001651
// C = 1.3875
// D = 0.063899
// F = -8.4365
// Humidity compensation:
// Linear dependency: 0.2 on T=-10 & 0.1 on T=50; so:
// dh:85->33=-1/600x+11/60 = (110-x)/600
// and dh:any->33= dh85->33/52*(humidity-33)
double mq136_apply_compensation(double value, int8_t compensation_t, uint8_t compensation_h, bool * success) {
	double t = compensation_t;
	double h = compensation_h;

	// 1. find humidity delta from current to 33% at current temperature
	double delta_h_85_33 = (110.0-h)/600.0;
	double delta_h = ((h-33.0)*delta_h_85_33)/52.0;

	// 2. find T delta for this temperature and 33% humidity (it's compensation allready applied)
	double delta_t = 9.4232*exp(-0.0001651*(t+20.0)) + 1.3875*exp(-0.063899*(t+20.0)) - 8.4365;

	// 3. total compensation: humidity moves graph down
	double compensation = delta_t - delta_h;

	if (compensation < 0.5 || compensation > 2) {
		ESP_LOGE(LOG_MQ136, "Bad compensations: H = %d, T = %d; rs/ro = %f, delta_h = %f; delta_t = %f; total compensation = %f",
				compensation_h, compensation_t, value, delta_h, delta_t, compensation);
		*success = false;
		return value;
	} else {
#if MQ136_DEBUG_COMPENSATIONS
		ESP_LOGI(LOG_MQ136, "Apply compensations: H = %d, T = %d; rs/ro = %f, delta_h = %f; delta_t = %f; total compensation = %f, result = %f",
				_h, _t, value, delta_h, delta_t, compensation, (value / compensation));
#endif
	}

	*success = true;
	// apply compensations
	return value / compensation;
}


// Used approximation formula:
// y = A*e^(-Bx) + C*e^(-Dx) + F
// Where:
// A = 4551.6
// B = 5.4194
// C = 1357.4
// D = 0.013213
// F = -1322.4
double mq136_rsro2value(double rs_ro) {
	return 4551.6*exp(-5.4194*rs_ro) + 1357.4*exp(-0.013213*rs_ro) - 1322.4;
}

void mq136_init() {
	adc_v_core_setup_t setup = {
		.calibrate_find_value_x10 = 0,

		.compensation = {
			.min_t = -10,
			.max_t = 70,
			.temperature = true,
			.humidity = true
		},

		.buildconfig = {
			.adc_channel = CONFIG_MQ136_ADC_CHANNEL,
			.topic_data = CONFIG_MQ136_TOPIC_DATA,
			.topic_command = CONFIG_MQ136_TOPIC_COMMAND,
			.name = "h2s",
			.tag = LOG_MQ136
		},

		.functions = {
			.is_startup_allowed = &adc_v_core_startup_allowed,
			.adc2rsro = &mq136_adc2rsro,
			.apply_compensation = &mq136_apply_compensation,
			.rsro2value = &mq136_rsro2value
		}
	};

	adc_v_core_init(&setup);
}
