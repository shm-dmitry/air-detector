#include "mq7.h"

#include "sdkconfig.h"

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
		ESP_LOGW(LOG_MQ136, "div by zero. ADC = %d", adc);
		return 0;
	}

	return ((double)calibration_value * (MQ7_AM - (double)adc)) / temp;
}

// Used a polynomial of the third degree to approxymate graph from datasheet
// Y = A*x^3 + B*x^2 + C*x + D.
// I took few points from datasheet and solves a system of linear equations
// so, I fount next values for 33%RH and different temperatures:
// A = -1/405000
// B = 13/27000
// C = -37/1350
// D = 557/405
// Different values for RH approxymated via linear delta by RH. But this delta
// depeneded same from T, so this delta approxymated by T using same polynom with
// values:
// A = 1/8100000
// B = -13/540000
// C = 101/54000
// D = -1043/8100
double mq7_apply_compensation(double value, int8_t compensation_t, uint8_t compensation_h, bool * success) {
	double t = compensation_t;
	double h = compensation_h;

	// 1st step: found hum-delta step for this temperature.
	double hum_delta_step_85_to_33 = t*t*t*1.0/8100000.0 - t*t*13.0/540000.0 + t*101.0/54000.0 + (-1043.0/8100.0);
	// found hum compensation using y = D * (H-33) / (85-33) to transfer H from current to 65%
	double hum_delta = hum_delta_step_85_to_33 * ((65.0 - 33.0) - (h - 33.0)) / (85.0 - 33.0);

	// 2nd step. ok, at now we can calculate temperature delta (for hum = 33%, hum compensation calculated on step-1)
	double temp_delta_tcur = -t*t*t*1.0/405000.0 + t*t*13.0/27000.0 - t*37.0/1350.0 + 557.0/405.0;

	// total compensation
	double compensation = hum_delta + temp_delta_tcur;

	if (compensation < 0.5 || compensation > 2) {
		ESP_LOGE(LOG_MQ7, "Bad compensations: H = %d, T = %d; rs/ro = %f, hum_delta = %f; temp_delta = %f; total compensation = %f",
				compensation_h, compensation_t, value, hum_delta, temp_delta_tcur, compensation);
		*success = false;
		return value;
	} else {
#if MQ7_DEBUG_COMPENSATIONS
		ESP_LOGI(LOG_MQ7, "Apply compensations: H = %d, T = %d; rs/ro = %f, hum_delta = %f; temp_delta = %f; total compensation = %f, result = %f",
				_h, _t, value, hum_delta, temp_delta_tcur, compensation, (value / compensation));
#endif
	}

	*success = true;
	// apply compensations
	return value / compensation;
}


// Used a polynomial of the third degree to approxymate graph from datasheet
// Y = A*x^3 + B*x^2 + C*x + D.
// I took few points from datasheet and solves a system of linear equations
// so, I fount next values:
// A = -383449750000/14583127
// B = 2845831817500/43749381
// C = -1804037315650/43749381
// D = 105049416430/14583127
double mq7_rsro2value(double rs_ro) {
	return -383449750000.0 * (rs_ro)*(rs_ro)*(rs_ro) / 14583127.0 +
			2845831817500.0 * (rs_ro)*(rs_ro) / 43749381.0 +
			-1804037315650.0 * (rs_ro) / 43749381.0 +
			105049416430.0 / 14583127.0;
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
