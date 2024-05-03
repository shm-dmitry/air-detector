#include "sgp41_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "raw2index/sensirion_gas_index_algorithm.h"

#include "../../log/log.h"
#include "string.h"
#include "../i2c_impl.h"

#define SGP41_INIT_TASK_STACK_SIZE 4096

#define SGP41_I2C_ADDRESS 0x59
#define SGP41_I2C_TIMEOUT 50

#define SGP41_INIT_STATUS_NOT_INITIALIZED  0x00
#define SGP41_INIT_STATUS_INITIALIZING     0x01
#define SGP41_INIT_STATUS_INITIALIZED      0x02
#define SGP41_INIT_STATUS_ERROR            0x03

#define SGP41_REF_UNKNOWN				   127
#define SGP41_REF_DEFAULT_TEMPERATURE	   25
#define SGP41_REF_DEFAULT_HUMIDITY		   50

const uint8_t SGP41_EXECUTE_CONDITIONING[] = { 0x26, 0x12 };
const uint8_t SGP41_MEASURE_RAW_SIGNALS[]  = { 0x26, 0x19 };
const uint8_t SGP41_EXECUTE_SELF_TEST[]    = { 0x28, 0x0E };
const uint8_t SGP41_TURN_HEATER_OFF[]      = { 0x36, 0x15 };
const uint8_t SGP41_GET_SERIAL_NUMBER[]    = { 0x36, 0x82 };

const uint8_t SGP41_EXECUTE_CONDITIONING_ARGS[] = { 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93 };
#define SGP41_EXECUTE_CONDITIONING_ARGS_SIZE 6

static uint8_t sgp41_init_status = SGP41_INIT_STATUS_NOT_INITIALIZED;

static int8_t sgp41_ref_temperature = SGP41_REF_UNKNOWN;
static uint8_t sgp41_ref_humidity   = SGP41_REF_UNKNOWN;
static i2c_handler_t * sgp41_i2c = NULL;

static GasIndexAlgorithmParams sgp41_tvoc;
static GasIndexAlgorithmParams sgp41_nox;

esp_err_t sgp41_write_read_buffer(
		uint16_t timeout_ms,
		const uint8_t * command,
		const uint8_t * args_buffer,
		const uint8_t args_buffer_size,
		uint16_t * buffer,
		uint8_t buffer_size);

static uint8_t sgp41_crc(uint8_t byte1, uint8_t byte2) {
	uint8_t crc = 0xFF;

	crc ^= byte1;

	for (uint8_t i = 0; i < 8; i++) {
		if ((crc & 0x80) != 0)
			crc = (uint8_t) ((crc << 1) ^ 0x31);
		else
			crc <<= 1;
	}
	crc ^= byte2;

	for (uint8_t i = 0; i < 8; i++) {
		if ((crc & 0x80) != 0)
			crc = (uint8_t) ((crc << 1) ^ 0x31);
		else
			crc <<= 1;
	}

	return crc;
}

void sgp41_initializing_task(void *) {
	uint16_t value = 0;
	esp_err_t res = sgp41_write_read_buffer(350, SGP41_EXECUTE_SELF_TEST, NULL, 0, &value, 1);
	if (res != ESP_OK) {
		LOGE(LOG_SGP41, "Self test failed: %02X", res);
		sgp41_init_status = SGP41_INIT_STATUS_ERROR;
		vTaskDelete(NULL);
		return;
	}

	if ((value & 0b00000011) == 0) {
		LOGI(LOG_SGP41, "Self test OK");
	} else {
		LOGE(LOG_SGP41, "Self test ERROR %04X", value);
		sgp41_init_status = SGP41_INIT_STATUS_ERROR;
		vTaskDelete(NULL);
		return;
	}

	// start heating..
	res = sgp41_write_read_buffer(
			350,
			SGP41_EXECUTE_CONDITIONING,
			SGP41_EXECUTE_CONDITIONING_ARGS,
			SGP41_EXECUTE_CONDITIONING_ARGS_SIZE,
			&value,
			1);
	if (res != ESP_OK) {
		LOGE(LOG_SGP41, "Self test failed: %02X", res);
		sgp41_init_status = SGP41_INIT_STATUS_ERROR;
		vTaskDelete(NULL);
		return;
	}

	// skip result, await for a sensor heated - 10 seconds
	vTaskDelay(10000 / portTICK_PERIOD_MS);

	// all done, setup finished status
	sgp41_init_status = SGP41_INIT_STATUS_INITIALIZED;

	vTaskDelete(NULL);
}

void sgp41_set_temp_humidity(int8_t temperature, uint8_t humidity) {
	if (temperature >= -45 && temperature < 100) {
		sgp41_ref_temperature = temperature;
	} else {
		sgp41_ref_temperature = SGP41_REF_UNKNOWN;
	}

	if (humidity < 100) {
		sgp41_ref_humidity = humidity;
	} else {
		sgp41_ref_humidity = SGP41_REF_UNKNOWN;
	}
}

esp_err_t sgp41_api_init() {
	sgp41_i2c = i2c_get_handlers(SGP41_I2C_ADDRESS, SGP41_I2C_TIMEOUT);
	if (sgp41_i2c == NULL) {
		LOGE(LOG_SGP41, "Cant init I2C for address %d", SGP41_I2C_ADDRESS);
		return ESP_ERR_INVALID_STATE;
	}

	uint16_t buffer[3];
	memset(buffer, 0, sizeof(uint16_t) * 3);
	esp_err_t res = sgp41_write_read_buffer(1, SGP41_GET_SERIAL_NUMBER, NULL, 0, buffer, 3);

	if (res) {
		LOGE(LOG_SGP41, "Cant read serial number: %02X", res);
		sgp41_init_status = SGP41_INIT_STATUS_ERROR;
		return res;
	} else {
		LOGI(LOG_SGP41, "Sensor serial number: %04X.%04X.%04X", buffer[0], buffer[1], buffer[2]);
	}

	sgp41_init_status = SGP41_INIT_STATUS_INITIALIZING;

	memset(&sgp41_tvoc, 0, sizeof(GasIndexAlgorithmParams));
	memset(&sgp41_nox,  0, sizeof(GasIndexAlgorithmParams));

	GasIndexAlgorithm_init_with_sampling_interval(&sgp41_tvoc, GasIndexAlgorithm_ALGORITHM_TYPE_VOC, SGP41_SAMPLING_INTERVAL);
	GasIndexAlgorithm_init_with_sampling_interval(&sgp41_nox,  GasIndexAlgorithm_ALGORITHM_TYPE_NOX, SGP41_SAMPLING_INTERVAL);

	xTaskCreate(sgp41_initializing_task, "sgp41 init task", SGP41_INIT_TASK_STACK_SIZE, NULL, 10, NULL);

	return ESP_OK;
}

void sgp41_ref_to_ticks(uint8_t value, uint8_t * result_byte_1, uint8_t * result_byte_2, uint8_t * result_byte_crc) {
	uint32_t temp = (value * 0xFFFF) / 100;
	if (temp > 0xFFFF) {
		temp = 0xFFFF;
	}

	*result_byte_1 = (temp >> 8);
	*result_byte_2 = (temp % 0xFF);
	*result_byte_crc = sgp41_crc(*result_byte_1, *result_byte_2);
}

esp_err_t sgp41_read(sgp41_data_t * result) {
	if (sgp41_init_status != SGP41_INIT_STATUS_INITIALIZED) {
		LOGE(LOG_SGP41, "Driver not initialized. Init status == %d", sgp41_init_status);
		return ESP_FAIL;
	}

	uint8_t args[6] = {0, 0, 0, 0, 0, 0};

	int8_t temperature = sgp41_ref_temperature;
	uint8_t humidity = sgp41_ref_humidity;
	if (temperature != SGP41_REF_UNKNOWN && humidity != SGP41_REF_UNKNOWN) {
		sgp41_ref_to_ticks(humidity, args, args + 1, args + 2);
		sgp41_ref_to_ticks((uint8_t)(temperature + 45), args + 3, args + 4, args + 5);
	} else {
		sgp41_ref_to_ticks(SGP41_REF_DEFAULT_HUMIDITY, args, args + 1, args + 2);
		sgp41_ref_to_ticks((uint8_t)(SGP41_REF_DEFAULT_TEMPERATURE + 45), args + 3, args + 4, args + 5);
	}

	uint16_t buffer[2];
	memset(buffer, 0, sizeof(uint16_t) * 2);

	esp_err_t res = sgp41_write_read_buffer(60, SGP41_MEASURE_RAW_SIGNALS, args, 6, buffer, 2);
	if (res != ESP_OK) {
		return res;
	}

	result->tvoc_raw = buffer[0];
	result->nox_raw = buffer[1];

	int32_t resultvalue = SGP41_VALUE_NODATA;
	GasIndexAlgorithm_process(&sgp41_tvoc, result->tvoc_raw, &resultvalue);
	if (resultvalue >= SGP41_VALUE_NODATA || resultvalue < 0) {
		LOGW(LOG_SGP41, "Bad gas-index for tvoc: %li", resultvalue);
		resultvalue = SGP41_VALUE_NODATA;
	}

	result->tvoc = resultvalue;

	resultvalue = SGP41_VALUE_NODATA;
	GasIndexAlgorithm_process(&sgp41_nox, result->nox_raw, &resultvalue);
	if (resultvalue >= SGP41_VALUE_NODATA || resultvalue < 0) {
		LOGW(LOG_SGP41, "Bad gas-index for nox: %li", resultvalue);
		resultvalue = SGP41_VALUE_NODATA;
	}

	result->nox = resultvalue;

	return ESP_OK;
}

esp_err_t sgp41_write_read_buffer(
		uint16_t timeout_ms,
		const uint8_t * command,
		const uint8_t * args_buffer,
		const uint8_t args_buffer_size,
		uint16_t * buffer,
		uint8_t buffer_size) {
	esp_err_t res;

	if (args_buffer && args_buffer_size > 0) {
		uint8_t * data = malloc(2 + args_buffer_size);
		if (data == NULL) {
			return ESP_ERR_NO_MEM;
		}

		memcpy(data, command, 2);
		memcpy(data + 2, args_buffer, args_buffer_size);
		res = sgp41_i2c->write(sgp41_i2c->context, data, 2 + args_buffer_size);
		free(data);
	} else {
		res = sgp41_i2c->write(sgp41_i2c->context, command, 2);
	}

	if (res) {
		LOGE(LOG_SGP41, "Write command %02x%02x [args size: %d] failed: %02X", command[0], command[1], args_buffer_size, res);
		return res;
	}

	if (timeout_ms < portTICK_PERIOD_MS) {
		timeout_ms = portTICK_PERIOD_MS;
	}
	vTaskDelay(timeout_ms / portTICK_PERIOD_MS);

	uint8_t * temp = malloc(buffer_size * 3);
	if (temp == NULL) {
		return ESP_ERR_NO_MEM;
	}

	res = sgp41_i2c->read(sgp41_i2c->context, temp, buffer_size * 3);
	if (res) {
		LOGE(LOG_SGP41, "Read command %02x%02x failed: %d", command[0], command[1], res);
		return res;
	}

	for (int i = 0; i<buffer_size; i++) {
		if (sgp41_crc(temp[i * 3], temp[i * 3 + 1]) != temp[i * 3 + 2] &&
				(temp[i * 3] != 0xFF && temp[i * 3 + 1] != 0xFF && temp[i * 3 + 2] != 0xFF)) {
			LOGE(LOG_SGP41, "Read word#%d failed. Invalid crc for [%02X %02X]. %02X != %02X",
					i,
					temp[i * 3],
					temp[i * 3 + 1],
					sgp41_crc(temp[i * 3], temp[i * 3 + 1]),
					temp[i * 3 + 2]);
			return ESP_ERR_INVALID_CRC;
		}

		buffer[i] = (temp[i * 3] << 8) + temp[i * 3 + 1];
	}

	return ESP_OK;
}
