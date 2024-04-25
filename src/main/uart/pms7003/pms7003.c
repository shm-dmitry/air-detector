#include "pms7003.h"

#include "pms7003_def.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"

#include "cJSON.h"
#include "../../cjson/cjson_helper.h"
#include "../../common/mqtt.h"
#include "../../log/log.h"
#include "string.h"

#define PMS7003_UART_PORT 1

#define PMS7003_BUF_SIZE 32
#define PMS7003_DRIVER_BUF_SIZE 1024
#define PMS7003_AWAIT_RESPONSE 1000
#define PMS7003_QUEUE_SIZE 10
#define PMS7003_EXEC_PERIOD 30000000

#define PMS7003_COMMAND_SIZE        7
#define PMS7003_COMMAND_WAKEUP      { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 }
#define PMS7003_COMMAND_SET_ACTIVE  { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 }
#define PMS7003_COMMAND_READ_DATA   { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 }

esp_err_t pms7003_set_active();
esp_err_t pms7003_wakeup();
uint16_t pms7003_crc(uint8_t * reply);
esp_err_t pms7003_send_buffer(const uint8_t * command, uint8_t * reply);
void pms7003_timer_exec_function(void* arg);

void pms7003_init() {
	uart_config_t uart_config = {
			.baud_rate = 9600,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB, };

	int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

	esp_err_t res = uart_driver_install(PMS7003_UART_PORT,
			PMS7003_DRIVER_BUF_SIZE, PMS7003_DRIVER_BUF_SIZE,
			PMS7003_QUEUE_SIZE, NULL, intr_alloc_flags);
	if (res) {
		ESP_LOGE(LOG_PMS7003, "uart_driver_install error: %d", res);
		return;
	}

	ESP_LOGI(LOG_PMS7003, "uart_set_pin OK");

	res = uart_param_config(PMS7003_UART_PORT, &uart_config);
	if (res) {
		ESP_LOGE(LOG_PMS7003, "uart_param_config error: %d", res);
		return;
	}

	ESP_LOGI(LOG_PMS7003, "uart_param_config OK");

	res = uart_set_pin(PMS7003_UART_PORT, CONFIG_PMS7003_TX, CONFIG_PMS7003_RX,
			UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (res) {
		ESP_LOGE(LOG_PMS7003, "uart_set_pin error: %d", res);
		return;
	}

	ESP_LOGI(LOG_PMS7003, "uart_set_pin OK");

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
	return pms7003_send_buffer(command, NULL);
}


esp_err_t pms7003_wakeup() {
	uint8_t command[] = PMS7003_COMMAND_WAKEUP;
	return pms7003_send_buffer(command, NULL);
}

uint16_t pms7003_crc(uint8_t * reply) {
	uint16_t sum = 0;

	for(int i=0; i<PMS7003_BUF_SIZE - 2; i++){
		sum += reply[i];
	}

	return sum;
}

esp_err_t pms7003_send_buffer(const uint8_t * command, uint8_t * reply) {
	while (true) {
		uint8_t buf;

		// cleanup buffer
		if (uart_read_bytes(PMS7003_UART_PORT, &buf, 1, 10 / portTICK_PERIOD_MS) <= 0) {
			break;
		}
	}

	esp_err_t res = uart_write_bytes(PMS7003_UART_PORT, command, PMS7003_COMMAND_SIZE);
	if (res <= 0) {
		ESP_LOGE(LOG_PMS7003, "pms7003_send_buffer  Cant send data to device");
		return ESP_FAIL;
	}

	vTaskDelay(20 / portTICK_PERIOD_MS);

	if (reply == NULL) {
		return ESP_OK;
	}

	memset(reply, 0xbb, PMS7003_BUF_SIZE);

	uint8_t await = PMS7003_AWAIT_RESPONSE / 20;
	uint8_t reply_index = 0;
	for (uint8_t i = 0; i<=await; i++) {
		if (i == await) {
			ESP_LOGE(LOG_PMS7003, "pms7003_send_buffer  Timeout awaiting for a data from device.");
			return ESP_ERR_TIMEOUT;
		}

		vTaskDelay(20 / portTICK_PERIOD_MS);

		int readed = uart_read_bytes(PMS7003_UART_PORT, reply + reply_index, PMS7003_BUF_SIZE - reply_index, 1 / portTICK_PERIOD_MS);
		if (readed > 0) {
			reply_index += readed;
			if (reply_index >= PMS7003_BUF_SIZE) {
				break;
			}
		}
	}

	uint16_t crc = pms7003_crc(reply);

	if (reply[0] != 0x42) {
		ESP_LOGE(LOG_PMS7003, "pms7003_send_buffer  Invalid response: bad byte#0 = %02X", reply[0]);
		return ESP_ERR_INVALID_RESPONSE;
	}
	if (reply[1] != 0x4d) {
		ESP_LOGE(LOG_PMS7003, "pms7003_send_buffer  Invalid response: bad byte#1 = %02X", reply[1]);
		return ESP_ERR_INVALID_RESPONSE;
	}
	if ((reply[PMS7003_BUF_SIZE - 2] << 8 | reply[PMS7003_BUF_SIZE - 1]) != crc) {
		ESP_LOGE(LOG_PMS7003, "pms7003_send_buffer  Invalid response: bad CRC : %02X%02X != %04X", reply[PMS7003_BUF_SIZE - 2], reply[PMS7003_BUF_SIZE - 1], crc);
		return ESP_ERR_INVALID_CRC;
	}

	return ESP_OK;
}


esp_err_t pms7003_read(pms7003_data_t * data) {
	uint8_t command[] = PMS7003_COMMAND_READ_DATA;
	uint8_t reply[PMS7003_BUF_SIZE] = { 0 };

	esp_err_t res = pms7003_send_buffer(command, reply);
	if (res) {
		return res;
	}

	data->atmospheric_pm_1_0 =  (reply[10] << 8) + reply[11];
	data->atmospheric_pm_2_5 =  (reply[12] << 8) + reply[13];
	data->atmospheric_pm_10_0 = (reply[14] << 8) + reply[15];

	return ESP_OK;
}

void pms7003_timer_exec_function(void* arg) {
	pms7003_data_t data = { 0 };
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
