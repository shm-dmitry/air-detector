#include "mq136.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"

#include "string.h"

#include "sdkconfig.h"
#include "cJSON.h"
#include "../../cjson/cjson_helper.h"
#include "../../common/mqtt.h"
#include "mq136_nvs.h"
#include "../../i2c/bme280/bme280_api.h"
#include "../../log/log.h"
#include "../adc.h"

#define MQ136_APPLY_COMPENSATION_PERIOD 60000000
#define MQ136_EXEC_PERIOD  				30000000
#define MQ136_NOVALUE      				0xFFFF
#define MQ136_COMPENSATION_NOVALUE      126
#define MQ136_CALIBRATION_NOVALUE		0xFFFF
#define MQ136_AM		   				((double)4095.0)
#define MQ136_3_3V		   				((double)3.3)

#define MQ136_DEBUG_COMPENSATIONS			true
#define MQ136_DEBUG_CALCULATION				true

// Used a polynomial of the third degree to approxymate graph from datasheet
// Y = A*x^3 + B*x^2 + C*x + D.
// I took few points from datasheet and solves a system of linear equations
// so, I fount next values:
// A = -8000/7
// B = 26200/7
// C = -29040/7
// D = 11120/7
#define MQ136_SOLVE_RSRO_TO_PPM(rs_ro) \
	(uint16_t)( \
		-8000.0 * (rs_ro)*(rs_ro)*(rs_ro) / 7.0 + \
		26200.0 * (rs_ro)*(rs_ro) / 7.0 + \
		-29040.0 * (rs_ro) / 7.0 + \
		11120.0 / 7.0 \
	)

#define MQ136_TEMPERATURE_COMPENSATION(t) \
	-(t)*(t)*(t)*1.0/405000.0 + (t)*(t)*13.0/27000.0 - (t)*37.0/1350.0 + 557.0/405.0

mq136_nvs_data_t mq136_calibration_value = {
	.a0     = MQ136_CALIBRATION_NOVALUE,
	.v5x100 = MQ136_CALIBRATION_NOVALUE
};
int8_t mq136_compensation_temperature = MQ136_COMPENSATION_NOVALUE;
uint8_t mq136_compensation_humidity   = MQ136_COMPENSATION_NOVALUE;

void mq136_init_auto_compensation();

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
double mq136_adjust_temperature_humidity(double value) {
	int8_t _t = mq136_compensation_temperature;
	uint8_t _h = mq136_compensation_humidity;
	if (_t == MQ136_COMPENSATION_NOVALUE || _h == MQ136_COMPENSATION_NOVALUE) {
		return value;
	}

	double t = _t;
	double h = _h;

	// 1st step: found hum-delta step for this temperature.
	double hum_delta_step_85_to_33 = t*t*t*1.0/8100000.0 - t*t*13.0/540000.0 + t*101.0/54000.0 + (-1043.0/8100.0);
	// found hum compensation using y = D * (H-33) / (85-33) to gransfer H from current to 65%
	double hum_delta = hum_delta_step_85_to_33 * ((65.0 - 33.0) - (h - 33.0)) / (85.0 - 33.0);

	// 2nd step. ok, at this step we have changes rs/ro from hum=33% to hum=85%.
	// at now we can calculate temperature delta (for hum = 33%)
	double temp_delta_h33_tcur = MQ136_TEMPERATURE_COMPENSATION(t);
	double temp_delta_h33_t20 = MQ136_TEMPERATURE_COMPENSATION(20);

	// corrent temperature to 20grad
	double temp_delta = temp_delta_h33_t20 - temp_delta_h33_tcur;

#if MQ136_DEBUG_COMPENSATIONS
	ESP_LOGI(LOG_MQ136, "Apply compensations: H = %d, T = %d; rs/ro = %f, hum_delta = %f; temp_delta = %f; result = %f",
			_h, _t, value, hum_delta, temp_delta, (value + hum_delta + temp_delta));
#endif

	// apply compensations
	return value + hum_delta + temp_delta;
}

void mq136_set_temp_humidity(int8_t temperature, uint8_t humidity) {
	if (temperature > -10 && temperature < 70) {
		mq136_compensation_temperature = temperature;
	} else {
		mq136_compensation_temperature = MQ136_COMPENSATION_NOVALUE;
	}

	if (humidity <= 100) {
		mq136_compensation_humidity = humidity;
	} else {
		mq136_compensation_humidity = MQ136_COMPENSATION_NOVALUE;
	}
}

// Math:
// As  = ADC value for current measurement (adc variable)
// Ao  = ADC value for zero-measurement    (calibrated value)
// V   = voltage on 5V-line
// Am  = 4095 (max ADC value)
// 3.3 = voltage on 3.3V-line

// Rs/Ro = (Ao / As) * (V*Am - As*3.3) / (V*Am - Ao*3.3)

uint16_t mq136_adc_to_ppm(int adc) {
	if (mq136_calibration_value.a0 == MQ136_CALIBRATION_NOVALUE     || mq136_calibration_value.a0 == 0 ||
		mq136_calibration_value.v5x100 == MQ136_CALIBRATION_NOVALUE || mq136_calibration_value.v5x100 == 0) {
		ESP_LOGW(LOG_MQ136, "No calibration. ADC = %d", adc);
		return MQ136_NOVALUE;
	}

	double temp = ((double)mq136_calibration_value.v5x100/100.0 * MQ136_AM - (double)adc*MQ136_3_3V);
	if (temp < 0.001 && temp > -0.001) { // check for a division-by-zero
		ESP_LOGW(LOG_MQ136, "div by zero. Calibration v5x100 = %d, ADC = %d", mq136_calibration_value.v5x100, adc);
		return MQ136_NOVALUE;
	}

	double rs_ro = ((double)adc / (double)mq136_calibration_value.a0) *
			((double)mq136_calibration_value.v5x100/100.0 * MQ136_AM - (double)adc*MQ136_3_3V) /
			temp;

#if MQ136_DEBUG_CALCULATION
	ESP_LOGI(LOG_MQ136, "Before compensations: ADC = %d -> rs/ro = %f", adc, rs_ro);
#endif

	rs_ro = mq136_adjust_temperature_humidity(rs_ro);

#if MQ136_DEBUG_CALCULATION
	ESP_LOGI(LOG_MQ136, "After compensations: ADC = %d -> rs/ro = %f", adc, rs_ro);
#endif

	uint16_t result = MQ136_SOLVE_RSRO_TO_PPM(rs_ro);

#if MQ136_DEBUG_CALCULATION
	ESP_LOGI(LOG_MQ136, "Result: %d ppm for ADC = %d and rs/ro = %f", result, adc, rs_ro);
#endif

	return result;
}

uint16_t mq136_read_value() {
	int value = 0;
	esp_err_t res = adc_oneshot_read(adc_get_channel(), (adc_channel_t) CONFIG_MQ136_ADC_CHANNEL, &value);
	if (res != ESP_OK) {
		ESP_LOGE(LOG_MQ136, "Cant read ADC value. Error %04X", res);
		return MQ136_NOVALUE;
	}

	return mq136_adc_to_ppm(value);
}

void mq136_calibrate(uint16_t v5x100) {
	int value = 0;
	esp_err_t res = adc_oneshot_read(adc_get_channel(), (adc_channel_t) CONFIG_MQ136_ADC_CHANNEL, &value);
	if (res != ESP_OK) {
		return;
	}

	mq136_calibration_value.a0 = value;
	mq136_calibration_value.v5x100 = v5x100;
	mq136_nws_write(&mq136_calibration_value);
}

void mq136_timer_exec_function(void* arg) {
	uint16_t value = mq136_read_value();
	if (value == MQ136_NOVALUE) {
		return;
	}

	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "h2s", value);

	char * json = cJSON_Print(root);
	mqtt_publish(CONFIG_MQ136_TOPIC_DATA, json);
	cJSON_free(json);

	cJSON_Delete(root);
}

void mq136_timer_apply_correction_function(void* arg) {
#if CONFIG_BME280_ENABLED
	bme280_data_t data = {0};
	if (bme280_read(&data) == ESP_OK) {
		mq136_set_temp_humidity(data.temperature, data.humidity);
	}
#endif
}

void mq136_commands(const char * topic, const char * data) {
	cJSON *root = cJSON_Parse(data);
	if (root == NULL) {
		return;
	}

	char * type = cJSON_GetStringValue(cJSON_GetObjectItem(root, "type"));
	if (strcmp(type, "calibrate") == 0) {
		uint16_t v5x100 = get_number16_from_json(cJSON_GetObjectItem(root, "v5_x100"), 0xFFFF);
		if (v5x100 == 0xFFFF) {
			v5x100 = 500;
		}

		mq136_calibrate(v5x100);
	}

	cJSON_Delete(root);
}

void mq136_init() {
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_get_channel(), (adc_channel_t) CONFIG_MQ136_ADC_CHANNEL, &config));

    ESP_LOGI(LOG_MQ136, "ADC initialized");

    mq136_nws_read(&mq136_calibration_value);

    if (mq136_calibration_value.a0 != MQ136_CALIBRATION_NOVALUE && mq136_calibration_value.v5x100 != MQ136_CALIBRATION_NOVALUE) {
    	ESP_LOGI(LOG_MQ136, "Calibration value: A0 = %d; v5x100 = %d", mq136_calibration_value.a0, mq136_calibration_value.v5x100);
    } else {
    	ESP_LOGW(LOG_MQ136, "No calibration value. Use type='calibrate' and v5_x100=... request");
    }

	esp_timer_create_args_t periodic_timer_args = {
		.callback = &mq136_timer_exec_function,
		/* name is optional, but may help identify the timer when debugging */
		.name = "mq136 publish value"
	};

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, MQ136_EXEC_PERIOD));

	mqtt_subscribe(CONFIG_MQ136_TOPIC_COMMAND, mq136_commands);

#if CONFIG_BME280_ENABLED
	mq136_init_auto_compensation();
#endif

    ESP_LOGI(LOG_MQ136, "Driver initialized");
}

void mq136_init_auto_compensation() {
	esp_timer_create_args_t periodic_timer_args = {
			.callback = &mq136_timer_apply_correction_function,
			/* name is optional, but may help identify the timer when debugging */
			.name = "Humidity auto compensation from BME280 for MQ136"
	};

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, MQ136_APPLY_COMPENSATION_PERIOD));
}
