#include "../sgp41/sgp41.h"

#include "string.h"

#include "sdkconfig.h"
#include "esp_timer.h"

#include "../bme280/bme280.h"
#include "cJSON.h"
#include "../../common/mqtt.h"
#include "../sgp41/sgp41_api.h"
#include "../../log/log.h"

#include "../bme280/bme280_api.h"

#define SGP41_EXEC_PERIOD 30000000
#define SGP41_APPLY_COMPENSATION_PERIOD 60000000

void sgp41_timer_exec_function(void* arg) {
	sgp41_data_t data = { 0 };
	if (sgp41_read(&data)) {
		return;
	}

	cJSON *root = cJSON_CreateObject();
	if (data.tvoc != SGP41_VALUE_NODATA) {
		cJSON_AddNumberToObject(root, "tvoc", data.tvoc);
	}
	if (data.nox != SGP41_VALUE_NODATA) {
		cJSON_AddNumberToObject(root, "nox", data.nox);
	}
	if (data.tvoc_raw != SGP41_VALUE_NODATA) {
		cJSON_AddNumberToObject(root, "tvoc_raw", data.tvoc_raw);
	}
	if (data.nox_raw != SGP41_VALUE_NODATA) {
		cJSON_AddNumberToObject(root, "nox_raw", data.nox_raw);
	}

	char * json = cJSON_Print(root);
	mqtt_publish(CONFIG_SGP41_TOPIC_DATA, json);
	cJSON_free(json);

	cJSON_Delete(root);
}

void sgp41_timer_apply_correction_function(void* arg) {
#if CONFIG_BME280_ENABLED
	bme280_data_t data = {0};
	if (bme280_read(&data) == ESP_OK) {
		sgp41_set_temp_humidity(data.temperature, data.humidity);
	}
#endif
}

void sgp41_init() {
	i2c_handler_t * sgp41_i2c = i2c_get_handlers(I2C_NUM_0);
	if (sgp41_i2c != NULL) {
		esp_err_t err = sgp41_api_init(sgp41_i2c);
		if (err != 0) {
			ESP_LOGE(LOG_SGP41, "Cant initialize SGP41: %02X", err);
		} else {
			ESP_LOGI(LOG_SGP41, "SGP41 initialized");
		}

		esp_timer_create_args_t periodic_timer_args = {
				.callback = &sgp41_timer_exec_function,
				/* name is optional, but may help identify the timer when debugging */
				.name = "sgp41 publish value"
		};

		esp_timer_handle_t periodic_timer;
		ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
		ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SGP41_EXEC_PERIOD));
	}
}

void sgp41_init_auto_compensation() {
	esp_timer_create_args_t periodic_timer_args = {
			.callback = &sgp41_timer_apply_correction_function,
			/* name is optional, but may help identify the timer when debugging */
			.name = "Humidity auto compensation from BME280"
	};

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SGP41_APPLY_COMPENSATION_PERIOD));
}
