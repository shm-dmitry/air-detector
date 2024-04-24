#include "adc_v_core_nvs.h"

#include "../../common/nvs_rw.h"
#include "../../log/log.h"
#include "string.h"

void adc_v_core_nws_read(const char * name, uint16_t * to) {
	if (to == NULL) {
		return;
	}

	size_t buffer_size = 0;
	uint8_t * buffer = NULL;

	esp_err_t res = nvs_read_buffer(name, &buffer, &buffer_size);
	if (res == ESP_OK) {
		if (buffer_size == 4 || buffer_size == 2) {
			*to = (buffer[0] << 8) + buffer[1];
		} else {
			ESP_LOGE(name, "Bad NVS buffer size: %d", buffer_size);
		}

		free(buffer);
		buffer = NULL;
	}
}

void adc_v_core_nws_write(const char * name, uint16_t value) {
	uint8_t buffer[2] = {(value >> 8), (value % 0x100)};
	esp_err_t res = nvs_write_buffer(name, buffer, 2);
	if (res != ESP_OK) {
		ESP_LOGE(name, "Cant write NVS settings. Res = %04X", res);
	}
}
