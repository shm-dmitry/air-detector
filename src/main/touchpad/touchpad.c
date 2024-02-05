#include "touchpad.h"

#include "../log/log.h"
#include "../cjson/cjson_helper.h"
#include "../common/mqtt.h"
#include "../led/led.h"
#include "string.h"
#include "touchpad_api.h"

#define TOUCHPAD_ON_KEY_DOWN_COLOR 0xFF000000

void touchpad_callback_func(uint8_t state, uint8_t click_index) {
	if (state == TOUCHPAD_ON_KEY_DOWN) {
		led_set_override_color(TOUCHPAD_ON_KEY_DOWN_COLOR);
	} else {
		led_reset_override_color();
	}


	cJSON *root = cJSON_CreateObject();

	if (state == TOUCHPAD_ON_KEY_DOWN) {
		cJSON_AddStringToObject(root, "value", "on_key_down");
		cJSON_AddNumberToObject(root, "click", click_index);
	} else if (state == TOUCHPAD_ON_KEY_UP) {
		cJSON_AddStringToObject(root, "value", "on_key_up");
		cJSON_AddNumberToObject(root, "click", click_index);
	} else if (state == TOUCHPAD_ON_CLICK) {
		cJSON_AddStringToObject(root, "value", "on_click");
		cJSON_AddNumberToObject(root, "click", click_index);
	} else if (state == TOUCHPAD_ON_ERROR) {
		cJSON_AddStringToObject(root, "value", "on_error");
	} else {
		cJSON_AddStringToObject(root, "value", "idle");
	}

	char * json = cJSON_Print(root);
	mqtt_publish_sync(CONFIG_TOUCHPAD_TOPIC_DATA, json);
	cJSON_free(json);

	cJSON_Delete(root);
}

void touchpad_init() {
	esp_err_t res = touchpad_setup(touchpad_callback_func);
	if (res == ESP_OK) {
		ESP_LOGI(LOG_TOUCHPAD, "Touchpad driver initialized");
	} else {
		ESP_LOGI(LOG_TOUCHPAD, "Cant initialize touchpad driver: %04X", res);
	}
}
