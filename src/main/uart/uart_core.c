#include "uart_core.h"

#include "driver/uart.h"
#include "../log/log.h"
#include "string.h"

#define PMS7003_DRIVER_BUF_SIZE 1024
#define PMS7003_QUEUE_SIZE 10

esp_err_t uart_core_init(const char * tag, uint8_t port, uint8_t tx, uint8_t rx) {
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

	esp_err_t res = uart_driver_install(port,
			PMS7003_DRIVER_BUF_SIZE, PMS7003_DRIVER_BUF_SIZE,
			PMS7003_QUEUE_SIZE, NULL, intr_alloc_flags);
	if (res) {
		LOGE(tag, "uart_driver_install error: %04X", res);
		return res;
	}

	LOGI(tag, "uart_set_pin OK");

	res = uart_param_config(port, &uart_config);
	if (res) {
		LOGE(tag, "uart_param_config error: %04X", res);
		return res;
	}

	LOGI(tag, "uart_param_config OK");

	res = uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (res) {
		LOGE(tag, "uart_set_pin error: %04X", res);
		return res;
	}

	return ESP_OK;
}

esp_err_t uart_core_send_buffer(const char * tag, uint8_t port, const uint8_t * send, uint8_t send_size, uint8_t * reply, uint8_t reply_size, uart_core_validate_buffer validator, uint16_t timeout) {
	while (true) {
		uint8_t buf;

		// cleanup buffer
		if (uart_read_bytes(port, &buf, 1, 10 / portTICK_PERIOD_MS) <= 0) {
			break;
		}
	}

	int res = uart_write_bytes(port, send, send_size);
	if (res <= 0) {
		LOGE(tag, "Cant send data to device");
		return ESP_FAIL;
	}

	vTaskDelay(20 / portTICK_PERIOD_MS);

	if (reply == NULL) {
		return ESP_OK;
	}

	memset(reply, 0xbb, reply_size);

	uint8_t await = timeout / 20;
	uint8_t reply_index = 0;
	for (uint8_t i = 0; i<=await; i++) {
		if (i == await) {
			LOGE(tag, "Timeout awaiting for a data from device.");
			return ESP_ERR_TIMEOUT;
		}

		vTaskDelay(20 / portTICK_PERIOD_MS);

		res = uart_read_bytes(port, reply + reply_index, reply_size - reply_index, 1 / portTICK_PERIOD_MS);
		if (res > 0) {
			reply_index += res;
			if (reply_index >= reply_size) {
				break;
			}
		}
	}

	if (validator != NULL) {
		return validator(send, reply);
	}

	return ESP_OK;
}
