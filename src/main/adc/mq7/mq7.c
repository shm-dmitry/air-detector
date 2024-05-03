#include "mq7.h"

#include "sdkconfig.h"

#include "math.h"

#include "../adc_v_core/adc_v_core.h"
#include "../../log/log.h"

#define MQ7_DEBUG_COMPENSATIONS 	false

#define MQ7_AM		   				((double)4095.0)


// Math:
// As  = ADC value for current measurement (adc variable)
// Ao  = ADC value for zero-measurement    (calibrated value)
// Am  = 4095 (max ADC value)

// Rs/Ro = (Ao / As) * (Am - As) / (Am - Ao) = (Ao * (Am - As)) / (As * (Am - Ao))
double mq7_adc2rsro(uint16_t adc, uint16_t calibration_value) {
	double temp = (double)adc * (MQ7_AM - (double)calibration_value);
	if (temp > -0.001 && temp < 0.001) { // check for a division-by-zero
		LOGW(LOG_MQ136, "div by zero. ADC = %d", adc);
		return 0;
	}

	return ((double)calibration_value * (MQ7_AM - (double)adc)) / temp;
}

// Temperature compensation: T = T+20; and used next formula
// y = A*e^(-Bx) + C*e^(-Dx) + F
// Where:
// A = 0.746382
// B = 0.031302
// C = 1.689218
// D = 0.266425
// F = 0.786563
// Humidity compensation:
// Linear dependency: 0.28 on T=-10 -> 0.14 on T=50; so:
// dh:85->33=7*(110-x)/3000
// and dh:any->33= [dh85->33]/52*(humidity-33)
double mq7_apply_compensation(double value, int8_t compensation_t, uint8_t compensation_h, bool * success) {
	double t = compensation_t;
	double h = compensation_h;

	// 1. find humidity delta from current to 33% at current temperature
	double delta_h_85_33 = ((110.0-t)*7.0)/3000.0;
	double delta_h = ((h-33.0)*delta_h_85_33)/52.0;

	// 2. find T delta for this temperature and 33% humidity (it's compensation allready applied)
	double delta_t = 0.746382*exp(-0.031302*(t+20.0)) + 1.689218*exp(-0.266425*(t+20.0)) + 0.786563;

	// 3. total compensation: humidity moves graph down
	double compensation = delta_t - delta_h;

	if (compensation < 0.5 || compensation > 2) {
		LOGE(LOG_MQ7, "Bad compensations: H = %d, T = %d; rs/ro = %f, delta_h = %f; delta_t = %f; total compensation = %f",
				compensation_h, compensation_t, value, delta_h, delta_t, compensation);
		*success = false;
		return value;
	} else {
#if MQ7_DEBUG_COMPENSATIONS
		LOGI(LOG_MQ7, "Apply compensations: H = %d, T = %d; rs/ro = %f, delta_h = %f; delta_t = %f; total compensation = %f, result = %f",
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
// A = 10865
// B = 13.158
// C = 816.79
// D = 2.7196
// F = 36.189
double mq7_rsro2value(double rs_ro) {
	return 10865*exp(-13.158*rs_ro) + 816.79*exp(-2.7196*rs_ro) + 36.189;
}

void mq7_init() {
	adc_v_core_setup_t setup = {
		.calibrate_find_value_x10 = 0,

		.compensation = {
			.min_t = -10,
			.max_t = 50,
			.temperature = true,
			.humidity = true
		},

		.buildconfig = {
			.adc_channel = CONFIG_MQ7_ADC_CHANNEL,
			.topic_data = CONFIG_MQ7_TOPIC_DATA,
			.topic_command = CONFIG_MQ7_TOPIC_COMMAND,
			.name = "co",
			.tag = LOG_MQ7
		},

		.functions = {
			.is_startup_allowed = &adc_v_core_startup_allowed,
			.adc2rsro = &mq7_adc2rsro,
			.apply_compensation = &mq7_apply_compensation,
			.rsro2value = &mq7_rsro2value
		}
	};

	adc_v_core_init(&setup);
}
