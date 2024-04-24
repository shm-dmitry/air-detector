#include "mq136.h"

#include "sdkconfig.h"

#include "../adc_v_core/adc_v_core.h"
#include "../../log/log.h"

#define MQ136_DEBUG_COMPENSATIONS false

#define MQ136_AM		   				((double)4095.0)

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
double mq136_apply_compensation(double value, int8_t compensation_t, uint8_t compensation_h, bool * success) {
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
		ESP_LOGE(LOG_MQ136, "Bad compensations: H = %d, T = %d; rs/ro = %f, hum_delta = %f; temp_delta = %f; total compensation = %f",
				compensation_h, compensation_t, value, hum_delta, temp_delta_tcur, compensation);
		*success = false;
		return value;
	} else {
#if MQ136_DEBUG_COMPENSATIONS
		ESP_LOGI(LOG_MQ136, "Apply compensations: H = %d, T = %d; rs/ro = %f, hum_delta = %f; temp_delta = %f; total compensation = %f, result = %f",
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
// A = -8000/7
// B = 26200/7
// C = -29040/7
// D = 11120/7
double mq136_rsro2value(double rs_ro) {
	return -8000.0 * (rs_ro)*(rs_ro)*(rs_ro) / 7.0 +
	26200.0 * (rs_ro)*(rs_ro) / 7.0 +
	-29040.0 * (rs_ro) / 7.0 +
	11120.0 / 7.0;
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
			.adc2rsro = &mq136_adc2rsro,
			.apply_compensation = &mq136_apply_compensation,
			.rsro2value = &mq136_rsro2value
		}
	};

	adc_v_core_init(&setup);
}
