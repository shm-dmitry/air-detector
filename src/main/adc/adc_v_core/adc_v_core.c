#include "adc_v_core.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "string.h"

#include "sdkconfig.h"
#include "cJSON.h"
#include "../../cjson/cjson_helper.h"
#include "../../common/mqtt.h"
#include "../../i2c/bme280/bme280_api.h"
#include "../../log/log.h"
#include "../adc.h"
#include "adc_v_core_nvs.h"

#define ADC_V_CORE_APPLY_COMPENSATION_PERIOD	60000000
#define ADC_V_CORE_EXEC_PERIOD  				30000000
#define ADC_V_CORE_COMPENSATION_NOVALUE      	126
#define ADC_V_CORE_COMPENSATION_IGNORED      	125
#define ADC_V_CORE_CALIBRATION_NOVALUE			0xFFFF

#define ADC_V_CORE_DEBUG_COMPENSATIONS			true
#define ADC_V_CORE_DEBUG_CALCULATION			true

#define ADC_V_CORE_CALIBRATE_DELTA_A0 200
#define ADC_V_CORE_CALIBRATE_DELTA_AUTORECALIBRATE_A0 50
#define ADC_V_CORE_AUTORECALIBRATE_COUNTER_MAX 5

typedef struct {
	char * name;
	char * topic_data;
	char * tag;

	int8_t autorecalibrate_counter;

	uint8_t adc_channel;

	uint16_t calibration_value;
	uint16_t calibrate_find_value_x10;

	adc_v_core__compensation_t compensation_settings;
	int8_t compensation_t;
	uint8_t compensation_h;

	adc_v_core__functions_t  functions;
} adc_v_core_context_t;

void adc_v_core_timer_apply_correction_function(void* arg);
void adc_v_core_init_auto_compensation(adc_v_core_context_t * context);
void adc_v_core_calibrate_execute(adc_v_core_context_t * context, uint16_t adc, bool full);

bool adc_v_core_adc2result(adc_v_core_context_t * context, uint16_t adc, bool autocompensation, double * result) {
	if (context->calibration_value == ADC_V_CORE_CALIBRATION_NOVALUE || context->calibration_value == 0) {
		LOGW(context->tag, "No calibration. ADC = %d", adc);
		return false;
	}

	double rs_ro = context->functions.adc2rsro(adc, context->calibration_value);

#if ADC_V_CORE_DEBUG_CALCULATION
	LOGI(context->tag, "Before compensations: ADC = %d -> rs/ro = %f", adc, rs_ro);
#endif

	int8_t _t = context->compensation_t;
	uint8_t _h = context->compensation_h;

	bool adjust_success = false;
	if (_t != ADC_V_CORE_COMPENSATION_NOVALUE &&
		_h != ADC_V_CORE_COMPENSATION_NOVALUE) {
		rs_ro = context->functions.apply_compensation(rs_ro, _t, _h, &adjust_success);
	}

#if ADC_V_CORE_DEBUG_CALCULATION
	LOGI(context->tag, "After compensations: ADC = %d -> rs/ro = %f", adc, rs_ro);
#endif

	double _result = context->functions.rsro2value(rs_ro);
	if (adjust_success && autocompensation) {
		if (_result < 0) {
			if (context->autorecalibrate_counter < ADC_V_CORE_AUTORECALIBRATE_COUNTER_MAX) {
				context->autorecalibrate_counter++;
			} else {
				ESP_LOGW(context->tag, "rsro->ppm : result=%f. Start fast auto-recalibration to find zero.", _result);
				adc_v_core_calibrate_execute(context, adc, false);
				context->autorecalibrate_counter = 0;
			}
		} else {
			context->autorecalibrate_counter = 0;
		}
	}

#if ADC_V_CORE_DEBUG_CALCULATION
	LOGI(context->tag, "Result: %f ppm for ADC = %d and rs/ro = %f", _result, adc, rs_ro);
#endif

	*result = _result;
	return true;
}

void adc_v_core_calibrate(adc_v_core_context_t * context) {
	if (!context->functions.is_startup_allowed()) {
		LOGE(context->tag, "Calibration not allowed");
		return;
	}

	int value = 0;
	esp_err_t res = adc_oneshot_read(adc_get_channel(), (adc_channel_t) (context->adc_channel), &value);
	if (res != ESP_OK) {
		LOGE(context->tag, "Cant read ADC value, err=%04X", res);
		return;
	}

	context->calibration_value = (uint16_t)value;

	// check - Have I data for a humidity and temperatore calibration?
	int8_t _t = context->compensation_t;
	uint8_t _h = context->compensation_h;
	if (_t == ADC_V_CORE_COMPENSATION_NOVALUE || _h == ADC_V_CORE_COMPENSATION_NOVALUE) {
		adc_v_core_nws_write(context->tag, context->calibration_value);
		LOGW(context->tag, "No data for compensaction.");
		return;
	}

	adc_v_core_calibrate_execute(context, value, true);
	adc_v_core_nws_write(context->tag, context->calibration_value);
}

bool adc_v_core_read_value(adc_v_core_context_t * context, double * result) {
	int value = 0;
	esp_err_t res = adc_oneshot_read(adc_get_channel(), (adc_channel_t) (context->adc_channel), &value);
	if (res != ESP_OK) {
		LOGE(context->tag, "Cant read ADC value. Error %04X", res);
		return false;
	}

	return adc_v_core_adc2result(context, value, true, result);
}

void adc_v_core_calibrate_execute(adc_v_core_context_t * context, uint16_t adc, bool full) {
	uint16_t delta = full ? ADC_V_CORE_CALIBRATE_DELTA_A0 : ADC_V_CORE_CALIBRATE_DELTA_AUTORECALIBRATE_A0;

	uint16_t min_a0 = (adc < delta ? 0 : adc - delta);
	uint16_t max_a0 = (((uint32_t) adc + (uint32_t)delta) > (uint32_t)0xFFFF ? 0xFFFF : adc + delta);

	bool findmin = context->calibrate_find_value_x10 == 0;
	double found_value = findmin ? 1000000 : 0;
	uint16_t found_value_a0 = adc;

	double result = 0;
	for (uint16_t i = min_a0; i<max_a0; i++) {
		if (i % 50 == 0) {
			vTaskDelay(1);
		}

		context->calibration_value = i;

		if (!adc_v_core_adc2result(context, adc, false, &result)) {
			context->calibration_value = adc;
			LOGE(context->tag, "Calibration - error in adc_v_core_adc2result");
			return;
		}

		if (
			(findmin && result < 0.5) ||
			(!findmin && result > (((double)context->calibrate_find_value_x10) / 10.0 - 0.5))
				) {
			LOGI(context->tag, "Calibration - compensation applied. Result: %f; A0: %d -> %d", result, adc, i);
			return;
		}

		if (
				(findmin && result < found_value) ||
				(!findmin && result > found_value)
			) {
			found_value = result;
			found_value_a0 = i;
		}
	}

	LOGW(context->tag, "Calibration - compensation applied partically. value = %f; A0: %d -> %d", found_value, adc, found_value_a0);
	context->calibration_value = found_value_a0;
}

void adc_v_core_commands(const char * data, void * arg) {
	adc_v_core_context_t * context = (adc_v_core_context_t *)arg;

	cJSON *root = cJSON_Parse(data);
	if (root == NULL) {
		return;
	}

	char * type = cJSON_GetStringValue(cJSON_GetObjectItem(root, "type"));
	if (strcmp(type, "calibrate") == 0) {
		adc_v_core_calibrate(context);
	}

	cJSON_Delete(root);
}

void adc_v_core_timer_exec_function(void* arg) {
	adc_v_core_context_t * context = (adc_v_core_context_t *) arg;

	if (!context->functions.is_startup_allowed()) {
		return;
	}

	double result = 0;
	if (!adc_v_core_read_value(context, &result)) {
		return;
	}

	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, context->name, (result < 0 ? (uint16_t)0 : (uint16_t)result));
	char * tmp = malloc(strlen(context->name) + 4 + 1);
	if (tmp) {
		strcpy(tmp, context->name);
		strcpy(tmp + strlen(tmp), "_raw");
		cJSON_AddNumberToObject(root, tmp, result);
		free(tmp);
	}

	char * json = cJSON_Print(root);
	mqtt_publish(context->topic_data, json);
	cJSON_free(json);

	cJSON_Delete(root);
}

void adc_v_core_init(const adc_v_core_setup_t * settings) {
	adc_v_core__buildconfig_t buildconfig = settings->buildconfig;

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_get_channel(), (adc_channel_t) buildconfig.adc_channel, &config));

    LOGI(buildconfig.tag, "ADC initialized");

    adc_v_core_context_t * context = malloc(sizeof(adc_v_core_context_t));
    if (context == NULL) {
        LOGE(buildconfig.tag, "OOM: context");
    	return;
    }

    memset(context, 0, sizeof(adc_v_core_context_t));

    context->name = (char *)malloc(strlen(buildconfig.name) + 1);
    if (context->name == NULL) {
        LOGE(buildconfig.tag, "OOM: name");
    	return;
    }
    strcpy(context->name, buildconfig.name);

    context->topic_data = (char *)malloc(strlen(buildconfig.topic_data) + 1);
    if (context->topic_data == NULL) {
        ESP_LOGE(buildconfig.tag, "OOM: topic_data");
    	return;
    }
    strcpy(context->topic_data, buildconfig.topic_data);

    context->tag = (char *)malloc(strlen(buildconfig.tag) + 1);
    if (context->tag == NULL) {
        ESP_LOGE(buildconfig.tag, "OOM: topic_data");
    	return;
    }
    strcpy(context->tag, buildconfig.tag);

    context->adc_channel = buildconfig.adc_channel;
    context->functions = settings->functions;

    context->calibration_value = ADC_V_CORE_CALIBRATION_NOVALUE;
    adc_v_core_nws_read(buildconfig.tag, &(context->calibration_value));

    context->compensation_h = ADC_V_CORE_COMPENSATION_NOVALUE;
    context->compensation_t = ADC_V_CORE_COMPENSATION_NOVALUE;
    context->compensation_settings = settings->compensation;
    context->autorecalibrate_counter = 0;
    context->calibrate_find_value_x10 = settings->calibrate_find_value_x10;

    if (context->calibration_value != ADC_V_CORE_CALIBRATION_NOVALUE) {
    	LOGI(buildconfig.tag, "Calibration value: A0 = %d", context->calibration_value);
    } else {
    	LOGW(buildconfig.tag, "No calibration value. Use type='calibrate' request");
    }

	esp_timer_create_args_t periodic_timer_args = {
		.callback = &adc_v_core_timer_exec_function,
		.arg = context,
	};

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ADC_V_CORE_EXEC_PERIOD));

	mqtt_subscribe(buildconfig.topic_command, adc_v_core_commands, context);

#if CONFIG_BME280_ENABLED
	adc_v_core_init_auto_compensation(context);
#endif

    LOGI(buildconfig.tag, "Driver initialized");
}

void adc_v_core_init_auto_compensation(adc_v_core_context_t * context) {
	esp_timer_create_args_t periodic_timer_args = {
		.callback = &adc_v_core_timer_apply_correction_function,
		.arg = context
	};

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ADC_V_CORE_APPLY_COMPENSATION_PERIOD));

	adc_v_core_timer_apply_correction_function(context);
}

void adc_v_core_timer_apply_correction_function(void* arg) {
#if CONFIG_BME280_ENABLED
	bme280_data_t data = {0};
	if (bme280_read(&data) == ESP_OK) {
		adc_v_core_context_t * context = (adc_v_core_context_t *) arg;

		if (context->compensation_settings.temperature &&
				data.temperature >= context->compensation_settings.min_t &&
				data.temperature <= context->compensation_settings.max_t) {
			context->compensation_t = data.temperature;
		} else {
			context->compensation_t = context->compensation_settings.temperature ?
					ADC_V_CORE_COMPENSATION_IGNORED : ADC_V_CORE_COMPENSATION_NOVALUE;
		}

		if (context->compensation_settings.humidity && data.humidity <= 100) {
			context->compensation_h = data.humidity;
		} else {
			context->compensation_h = context->compensation_settings.humidity ?
					ADC_V_CORE_COMPENSATION_IGNORED : ADC_V_CORE_COMPENSATION_NOVALUE;
		}
	}
#endif
}

bool adc_v_core_startup_allowed() {
	return true;
}

