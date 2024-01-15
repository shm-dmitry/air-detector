#include "../../fans/fan_pwm/fan_pwm.h"

#include "fan_pwm_api.h"
#include "../../cjson/cjson_helper.h"
#include "../../fans/fan_pwm/fan_pwm_api.h"
#include "../../common/mqtt.h"
#include "../../log/log.h"

#include "sdkconfig.h"

#define FAN_PWM_NOCHANGE 250

void fan_pwm_commands(const char * topic, const char * data) {
	cJSON *root = cJSON_Parse(data);
	if (root == NULL) {
		return;
	}

	uint8_t percent = get_number8_from_json(cJSON_GetObjectItem(root, "percent"), FAN_PWM_NOCHANGE);
	if (percent != FAN_PWM_NOCHANGE) {
		fan_pwm_set_percent(percent);
	}

	cJSON_Delete(root);
}

void fanpwm_init() {
	esp_err_t res = fan_pwm_port_init();
	if (res == ESP_OK) {
		ESP_LOGI(LOG_FANPWM, "FAN PWM driver initialized");
	} else {
		ESP_LOGI(LOG_FANPWM, "Cant initlize FAN PWM driver: %04X", res);
	}

	mqtt_subscribe(CONFIG_FANPWM_TOPIC_COMMAND, fan_pwm_commands);
}
