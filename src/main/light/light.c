#include "light.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"

#include "sdkconfig.h"
#include "cJSON.h"
#include "../cjson/cjson_helper.h"
#include "../common/mqtt.h"

#define LIGHT_EXEC_PERIOD	30000000
#define LIGHT_NOVALUE       0xFF

#define LIGHT_ADC_MAX		(4095)
#define LIGHT_ADC_ZERO		(2000)

// ADC_MAX / 2 == 0%, ADC_MAX == 100%
// y = 100 * (v - Az) / (Am - Az)
#define LIGHT_ADC_TO_RESULT(value) \
	(100 * (value - LIGHT_ADC_ZERO) / (LIGHT_ADC_MAX - LIGHT_ADC_ZERO))


adc_oneshot_unit_handle_t light_adc_channel;

uint8_t light_read_value() {
	int value = 0;
	esp_err_t res = adc_oneshot_read(light_adc_channel, (adc_channel_t) CONFIG_LIGHT_ADC_CHANNEL, &value);
	if (res != ESP_OK) {
		return LIGHT_NOVALUE;
	}

	return (uint8_t) LIGHT_ADC_TO_RESULT(value);
}

void light_timer_exec_function(void* arg) {
	uint8_t value = light_read_value();
	if (value == LIGHT_NOVALUE) {
		return;
	}

	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "light", value);

	char * json = cJSON_Print(root);
	mqtt_publish(CONFIG_LIGHT_TOPIC_DATA, json);
	cJSON_free(json);

	cJSON_Delete(root);
}

void light_init() {
	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_1,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &light_adc_channel));

	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = ADC_ATTEN_DB_12,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(light_adc_channel, (adc_channel_t) CONFIG_LIGHT_ADC_CHANNEL, &config));

	esp_timer_create_args_t periodic_timer_args = {
		.callback = &light_timer_exec_function,
		/* name is optional, but may help identify the timer when debugging */
		.name = "light publish value"
	};

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LIGHT_EXEC_PERIOD));

}
