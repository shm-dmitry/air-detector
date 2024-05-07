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
			LOGE(name, "Bad NVS buffer size: %d", buffer_size);
		}

		free(buffer);
		buffer = NULL;
	}
}

void adc_v_core_nws_write(const char * name, uint16_t value) {
	uint8_t buffer[2] = {(value >> 8), (value % 0x100)};
	esp_err_t res = nvs_write_buffer(name, buffer, 2);
	if (res != ESP_OK) {
		LOGE(name, "Cant write NVS settings. Res = %04X", res);
	}
}

void adc_v_core_nws_read_zero_offset(const char * name,  uint16_t * to) {
	uint8_t len = strlen(name);
	char * tmp = malloc(len + 2 + 1);
	if (tmp) {
		memset(tmp, 0, len + 2 + 1);
		strcpy(tmp, name);
		tmp[len] = '_';
		tmp[len + 1] = 'z';
		tmp[len + 2] = 0;

		adc_v_core_nws_read(tmp, to);

		free(tmp);
	}
}

void adc_v_core_nws_write_zero_offset(const char * name, uint16_t value) {
	uint8_t len = strlen(name);
	char * tmp = malloc(len + 2 + 1);
	if (tmp) {
		memset(tmp, 0, len + 2 + 1);
		strcpy(tmp, name);
		tmp[len] = '_';
		tmp[len + 1] = 'z';
		tmp[len + 2] = 0;

		adc_v_core_nws_write(tmp, value);

		free(tmp);
	}
}
