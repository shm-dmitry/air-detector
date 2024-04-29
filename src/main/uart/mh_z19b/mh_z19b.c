#include "mh_z19b.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "../uart_core.h"

#include "cJSON.h"
#include "../../cjson/cjson_helper.h"
#include "../../common/mqtt.h"
#include "../../log/log.h"
#include "string.h"

static uint8_t COMMAND_MHZ19_RANGE_5000[]  = { 0x99, 0x13, 0x88, 0x00, 0x00, 0x00 };
static uint8_t COMMAND_MHZ19_CALIBRATE_ENABLE[]  = { 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00 };
static uint8_t COMMAND_MHZ19_CALIBRATE_DISABLE[] = { 0x79, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t COMMAND_MHZ19_READ_VALUE[]  = { 0x86, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t COMMAND_MHZ19_CALIBRATE_ZERO[] = { 0x87, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define MHZ19B_BUF_SIZE 	   9
#define MHZ19B_DRIVER_BUF_SIZE 1024
#define MHZ19B_QUEUE_SIZE      10
#define MHZ19B_UART_PORT       2
#define MHZ19B_AWAIT_RESPONSE  1000
#define MHZ19B_EXEC_PERIOD 	   10000000

uint8_t mhz19b_crc(const uint8_t * buffer);
esp_err_t mhz19b_send_buffer(const uint8_t * buffer, uint8_t * reply);
void mhz19b_commands(const char * data, void *);
void mhz19b_timer_exec_function(void*);
esp_err_t mhz19b_validate(const uint8_t * send, const uint8_t * reply);

esp_err_t mhz19b_autocalibrate(bool value) {
	return mhz19b_send_buffer(value ? COMMAND_MHZ19_CALIBRATE_ENABLE : COMMAND_MHZ19_CALIBRATE_DISABLE, NULL);
}

esp_err_t mhz19b_calibrate() {
	return mhz19b_send_buffer(COMMAND_MHZ19_CALIBRATE_ZERO, NULL);
}

void mhz19b_init() {
	esp_err_t res = uart_core_init(LOG_PMS7003, MHZ19B_UART_PORT, CONFIG_MHZ19B_TX, CONFIG_MHZ19B_RX);
	if (res) {
		return;
	}

	res = mhz19b_send_buffer(COMMAND_MHZ19_RANGE_5000, NULL);

	if (res) {
		ESP_LOGE(LOG_MHZ19B, "mh_z19b_send_buffer(range) error: %04X", res);
		return;
	} else {
		ESP_LOGI(LOG_MHZ19B, "mh_z19b_send_buffer(range) OK");
	}

	vTaskDelay(100 / portTICK_PERIOD_MS);

	res = mhz19b_autocalibrate(false);
	if (res) {
		ESP_LOGE(LOG_MHZ19B, "mh_z19b_autocalibrate(false) error: %04X", res);
	}

	mqtt_subscribe(CONFIG_MHZ19B_TOPIC_COMMAND, mhz19b_commands, NULL);

	esp_timer_create_args_t periodic_timer_args = {
			.callback = &mhz19b_timer_exec_function,
	};

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, MHZ19B_EXEC_PERIOD));

}

esp_err_t mhz19b_read(uint16_t * co2) {
	uint8_t buffer[9] = { 0 };

	esp_err_t res = mhz19b_send_buffer(COMMAND_MHZ19_READ_VALUE, buffer);
	if (res) {
		ESP_LOGE(LOG_MHZ19B, "mh_z19b_read error: %02X", res);
		return res;
	}

	*co2 = buffer[2] * 256 + buffer[3];

	return ESP_OK;
}

esp_err_t mhz19b_send_buffer(const uint8_t * buffer, uint8_t * reply) {
	uint8_t command[] = { 0xFF, 0x01, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], 0x00 };
	command[8] = mhz19b_crc(command);

	return uart_core_send_buffer(LOG_MHZ19B, MHZ19B_UART_PORT, command, MHZ19B_BUF_SIZE, reply, MHZ19B_BUF_SIZE, mhz19b_validate, MHZ19B_AWAIT_RESPONSE);
}

uint8_t mhz19b_crc(const uint8_t * buffer) {
	uint8_t crc = 0;
	for (uint8_t i = 1; i < 8; i++)
	{
		crc += buffer[i];
	}
	crc = 255 - crc;
	crc++;

	return crc;
}

esp_err_t mhz19b_validate(const uint8_t * send, const uint8_t * reply) {
	if (reply[0] != 0xFF) {
		ESP_LOGE(LOG_MHZ19B, "Invalid response from device (bad magic byte) %02X != 0xFF", reply[0]);
		return ESP_ERR_INVALID_RESPONSE;
	}
	if (reply[1] != send[2]) {
		ESP_LOGE(LOG_MHZ19B, "Invalid response from device (bad command) %02X != %02X", reply[1], send[0]);
		return ESP_ERR_INVALID_RESPONSE;
	}

	uint8_t crc = mhz19b_crc(reply);
	if (reply[8] != crc) {
		ESP_LOGE(LOG_MHZ19B, "Invalid response from device (bad crc : %02X != %02X)", reply[8], crc);
		return ESP_ERR_INVALID_CRC;
	}

	return ESP_OK;
}

void mhz19b_commands(const char * data, void *) {
	cJSON *root = cJSON_Parse(data);
	if (root == NULL) {
		return;
	}

	char * type = cJSON_GetStringValue(cJSON_GetObjectItem(root, "type"));
	if (strcmp(type, "calibrate") == 0) {
		mhz19b_calibrate();
	}

	cJSON_Delete(root);
}

void mhz19b_timer_exec_function(void*) {
	uint16_t co2 = 0;
	if (mhz19b_read(&co2)) {
		return;
	}

	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "co2", co2);

	char * json = cJSON_Print(root);
	mqtt_publish(CONFIG_MHZ19B_TOPIC_DATA, json);
	cJSON_free(json);

	cJSON_Delete(root);
}
