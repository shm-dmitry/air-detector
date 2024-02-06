#include "mq136_nvs.h"

#include "../../common/nvs_rw.h"
#include "../../log/log.h"
#include "string.h"

#define MQ136_NVS_NAME "mq136"

void mq136_nws_read(mq136_nvs_data_t * to) {
	if (to == NULL) {
		return;
	}

	size_t buffer_size = 0;
	uint8_t * buffer = NULL;

	esp_err_t res = nvs_read_buffer(MQ136_NVS_NAME, &buffer, &buffer_size);
	if (res == ESP_OK) {
		if (buffer_size == 4 || buffer_size == 2) {
			to->a0 = (buffer[0] << 8) + buffer[1];
		} else {
			ESP_LOGE(LOG_MQ136, "Bad NVS buffer size: %d", buffer_size);
		}

		free(buffer);
		buffer = NULL;
	}
}

void mq136_nws_write(const mq136_nvs_data_t * value) {
	if (value == NULL) {
		return;
	}

	uint8_t buffer[2] = {(value->a0 >> 8),     (value->a0 % 0x100)};
	esp_err_t res = nvs_write_buffer(MQ136_NVS_NAME, buffer, 2);
	if (res != ESP_OK) {
		ESP_LOGE(LOG_MQ136, "Cant write NVS settings. Res = %04X", res);
	}
}
