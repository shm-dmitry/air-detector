#include "pms7003.h"

#include "pms7003_def.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "../uart_core.h"

#include "cJSON.h"
#include "../../cjson/cjson_helper.h"
#include "../../common/mqtt.h"
#include "../../log/log.h"
#include "string.h"

#define PMS7003_UART_PORT 1

#define PMS7003_BUF_SIZE 32
#define PMS7003_AWAIT_RESPONSE 1000
#define PMS7003_EXEC_PERIOD 30000000

#define PMS7003_COMMAND_SIZE        7
#define PMS7003_COMMAND_WAKEUP      { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 }
#define PMS7003_COMMAND_SET_ACTIVE  { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 }
#define PMS7003_COMMAND_READ_DATA   { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 }

esp_err_t pms7003_set_active();
esp_err_t pms7003_wakeup();
void pms7003_timer_exec_function(void* arg);
esp_err_t pms7003_validate(const uint8_t *, const uint8_t *);

void pms7003_init() {
	esp_err_t res = uart_core_init(LOG_PMS7003, PMS7003_UART_PORT, CONFIG_PMS7003_TX, CONFIG_PMS7003_RX);
	if (res) {
		return;
	}

	gpio_config_t config = {
				.intr_type = GPIO_INTR_DISABLE,
			    .mode = GPIO_MODE_OUTPUT,
				.pin_bit_mask = 1ULL << CONFIG_PMS7003_RESET,
				.pull_down_en = GPIO_PULLDOWN_DISABLE,
				.pull_up_en = GPIO_PULLUP_DISABLE,
			};

	res = gpio_config(&config);
	if (res) {
		ESP_LOGI(LOG_FAN, "Cant init GPIO. error %d", res);
		return;
	}

	gpio_set_level(CONFIG_PMS7003_RESET, 1);

	res = pms7003_wakeup();
	if (res) {
		ESP_LOGE(LOG_PMS7003, "Cant wakeup sensor: %d", res);
		return;
	}

	vTaskDelay(100 / portTICK_PERIOD_MS);

	res = pms7003_set_active(true);
	if (res) {
		ESP_LOGE(LOG_PMS7003, "Cant set sensor state == active: %d", res);
		return;
	}

	esp_timer_create_args_t periodic_timer_args = {
			.callback = &pms7003_timer_exec_function,
	};

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, PMS7003_EXEC_PERIOD));
}

esp_err_t pms7003_set_active() {
	uint8_t command[] = PMS7003_COMMAND_SET_ACTIVE;
	return uart_core_send_buffer(LOG_PMS7003, PMS7003_UART_PORT, command, PMS7003_COMMAND_SIZE, NULL, 0, pms7003_validate, PMS7003_AWAIT_RESPONSE);
}


esp_err_t pms7003_wakeup() {
	uint8_t command[] = PMS7003_COMMAND_WAKEUP;
	return uart_core_send_buffer(LOG_PMS7003, PMS7003_UART_PORT, command, PMS7003_COMMAND_SIZE, NULL, 0, pms7003_validate, PMS7003_AWAIT_RESPONSE);
}

esp_err_t pms7003_validate(const uint8_t * send, const uint8_t * reply) {
	if (reply[0] != 0x42) {
		ESP_LOGE(LOG_PMS7003, "pms7003_send_buffer  Invalid response: bad byte#0 = %02X", reply[0]);
		return ESP_ERR_INVALID_RESPONSE;
	}
	if (reply[1] != 0x4d) {
		ESP_LOGE(LOG_PMS7003, "Invalid response: bad byte#1 = %02X", reply[1]);
		return ESP_ERR_INVALID_RESPONSE;
	}

	uint16_t crc = 0;

	for(int i=0; i<PMS7003_BUF_SIZE - 2; i++){
		crc += reply[i];
	}

	if ((reply[PMS7003_BUF_SIZE - 2] << 8 | reply[PMS7003_BUF_SIZE - 1]) != crc) {
		ESP_LOGE(LOG_PMS7003, "Invalid response: bad CRC : %02X%02X != %04X", reply[PMS7003_BUF_SIZE - 2], reply[PMS7003_BUF_SIZE - 1], crc);
		return ESP_ERR_INVALID_CRC;
	}

	return ESP_OK;
}


esp_err_t pms7003_read(pms7003_data_t * data) {
	uint8_t command[] = PMS7003_COMMAND_READ_DATA;
	uint8_t reply[PMS7003_BUF_SIZE] = { 0 };

	esp_err_t res = uart_core_send_buffer(LOG_PMS7003, PMS7003_UART_PORT, command, PMS7003_COMMAND_SIZE, reply, PMS7003_BUF_SIZE, pms7003_validate, PMS7003_AWAIT_RESPONSE);
	if (res) {
		return res;
	}

	data->atmospheric_pm_1_0 =  (reply[10] << 8) + reply[11];
	data->atmospheric_pm_2_5 =  (reply[12] << 8) + reply[13];
	data->atmospheric_pm_10_0 = (reply[14] << 8) + reply[15];

	return ESP_OK;
}

void pms7003_timer_exec_function(void* arg) {
	pms7003_data_t data = {
		.atmospheric_pm_1_0  = 0,
		.atmospheric_pm_2_5  = 0,
		.atmospheric_pm_10_0 = 0
	};

	if (pms7003_read(&data)) {
		return;
	}

	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "atmospheric_pm_1_0",  data.atmospheric_pm_1_0);
	cJSON_AddNumberToObject(root, "atmospheric_pm_2_5",  data.atmospheric_pm_2_5);
	cJSON_AddNumberToObject(root, "atmospheric_pm_10_0", data.atmospheric_pm_10_0);

	char * json = cJSON_Print(root);
	mqtt_publish(CONFIG_PMS7003_TOPIC_DATA, json);
	cJSON_free(json);

	cJSON_Delete(root);
}
