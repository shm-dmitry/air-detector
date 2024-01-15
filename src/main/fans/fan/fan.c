#include "../../fans/fan/fan.h"

#include "../../cjson/cjson_helper.h"
#include "../../common/mqtt.h"
#include "../../log/log.h"

#include "string.h"

#include "driver/gpio.h"

#define FAN_CHANGE_STATUS_ENABLED  1
#define FAN_CHANGE_STATUS_DISABLED 0
#define FAN_CHANGE_STATUS_NOT_SET  2

esp_err_t fan_start();
esp_err_t fan_stop();

void fan_commands(const char * topic, const char * data) {
	cJSON *root = cJSON_Parse(data);
	if (root == NULL) {
		return;
	}

	uint8_t state = get_boolean_from_json(cJSON_GetObjectItem(root, "state"), FAN_CHANGE_STATUS_ENABLED, FAN_CHANGE_STATUS_DISABLED, FAN_CHANGE_STATUS_NOT_SET);

	if (state == FAN_CHANGE_STATUS_ENABLED) {
		fan_start();
	} else if (state == FAN_CHANGE_STATUS_DISABLED) {
		fan_stop();
	}

	cJSON_Delete(root);
}

void fan_init() {
	gpio_config_t config = {
			.intr_type = GPIO_INTR_DISABLE,
		    .mode = GPIO_MODE_INPUT_OUTPUT,
			.pin_bit_mask = 1ULL << CONFIG_FAN_GPIO,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.pull_up_en = GPIO_PULLUP_DISABLE,
		};

	esp_err_t res = gpio_config(&config);
	if (res) {
		ESP_LOGI(LOG_FAN, "Cant init GPIO. error %d", res);
		return;
	}

	fan_stop();

	mqtt_subscribe(CONFIG_FAN_TOPIC_DATA, fan_commands);
}


esp_err_t fan_start() {
	esp_err_t res = gpio_set_level(CONFIG_FAN_GPIO, 1);
	if (res) {
		ESP_LOGE(LOG_FAN, "Cant set HIGH level on pin %d: %d", CONFIG_FAN_GPIO, res);
	} else {
		ESP_LOGI(LOG_FAN, "HIGH level on pin %d activated.", CONFIG_FAN_GPIO);
	}

	return res;
}

esp_err_t fan_stop() {
	esp_err_t res = gpio_set_level(CONFIG_FAN_GPIO, 0);
	if (res) {
		ESP_LOGE(LOG_FAN, "Cant set LOW level on pin %d: %d", CONFIG_FAN_GPIO, res);
	} else {
		ESP_LOGI(LOG_FAN, "LOW level on pin %d activated.", CONFIG_FAN_GPIO);
	}

	return res;
}

