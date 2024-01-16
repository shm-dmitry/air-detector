#include "bme280.h"

#include "string.h"
#include "sdkconfig.h"

#include "esp_timer.h"

#include "bme280_api.h"
#include "cJSON.h"
#include "../../common/mqtt.h"
#include "../../log/log.h"

#define BME280_EXEC_PERIOD 30000000

void bme280_timer_exec_function(void* arg) {
	bme280_data_t data = { 0 };
	if (bme280_read(&data)) {
		return;
	}

	ESP_LOGI(LOG_BME280, "Temperature: %f; Humidity: %f%%", data.temperature, data.humidity);

	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "temperature", data.temperature);
	cJSON_AddNumberToObject(root, "humidity", data.humidity);
	cJSON_AddNumberToObject(root, "pressure", data.pressure);
	cJSON_AddNumberToObject(root, "heatindex", data.heatindex);
	cJSON_AddNumberToObject(root, "absolute_humidity", data.absolute_humidity);

	char * json = cJSON_Print(root);
	mqtt_publish(CONFIG_BME280_TOPIC_DATA, json);
	cJSON_free(json);

	cJSON_Delete(root);
}

void bme280_init() {
	i2c_handler_t * bme280_i2c = i2c_get_handlers(I2C_NUM_0);
	if (bme280_i2c != NULL) {
		esp_err_t res = bme280_init_driver(bme280_i2c);
		if (res != ESP_OK) {
			ESP_LOGE(LOG_BME280, "BME280 - cant initialize driver. Error %04X", res);
		} else {
			ESP_LOGI(LOG_BME280, "BME280 driver initialized");
		}

		esp_timer_create_args_t periodic_timer_args = {
				.callback = &bme280_timer_exec_function,
				/* name is optional, but may help identify the timer when debugging */
				.name = "bme280 publish value"
		};

		esp_timer_handle_t periodic_timer;
		ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
		ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, BME280_EXEC_PERIOD));
	}
}
