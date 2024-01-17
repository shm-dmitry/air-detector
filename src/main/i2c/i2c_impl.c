#include "i2c_impl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"

#include "../log/log.h"
#include "string.h"

#define I2C_DEFAULT_SPEED 100000
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define I2C_MUTEX_AWAIT ((TickType_t) 500)
#define I2C_COMMAND_AWAIT (1000 / portTICK_PERIOD_MS)

#define I2C_DEBUG_OUTPUT true

typedef struct i2c_context_t {
	uint8_t addr;
	i2c_master_dev_handle_t dev_handle;
	uint16_t transfer_timeout_ms;
} i2c_context_t;

SemaphoreHandle_t i2c_mutex = NULL;
i2c_master_bus_handle_t i2c_bus_handle = NULL;

esp_err_t i2c_read(void * i2c_handler_context, uint8_t* buffer, uint8_t buffer_size);
esp_err_t i2c_write(void * i2c_handler_context, const uint8_t* buffer, uint8_t buffer_size);
esp_err_t i2c_write_read(void * i2c_handler_context, const uint8_t* write_buffer, uint8_t write_buffer_size, uint8_t* read_buffer, uint8_t read_buffer_size);

void i2c_init_driver(int gpio_sda, int gpio_scl){
	i2c_master_bus_config_t conf = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_NUM_0,
        .scl_io_num = gpio_scl,
        .sda_io_num = gpio_sda
	};

	ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &i2c_bus_handle));

	vSemaphoreCreateBinary(i2c_mutex);
	if (i2c_mutex == NULL) {
	    ESP_LOGE(LOG_I2C, "Cant init semaphore");
		return;
	}

    ESP_LOGI(LOG_I2C, "I2C port with pins sda %d / scl %d initialized", gpio_sda, gpio_scl);
}

i2c_handler_t * i2c_get_handlers(uint8_t addr, uint16_t transfer_timeout_ms){
	if (i2c_bus_handle == NULL) {
		ESP_LOGE(LOG_I2C, "I2C port not initialized yet");
		return NULL;
	}

	i2c_device_config_t dev_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = addr,
	    .scl_speed_hz = I2C_DEFAULT_SPEED,
	};
	i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle));

    i2c_handler_t * result = (i2c_handler_t*) malloc(sizeof(i2c_handler_t));
    memset(result, 0, sizeof(i2c_handler_t));

    result->read = i2c_read;
    result->write = i2c_write;
    result->write_read = i2c_write_read;
    result->context = malloc(sizeof(i2c_context_t));
    if (result->context == NULL) {
    	free(result);
    	result = NULL;
    	return NULL;
    }

    ((i2c_context_t*)result->context)->addr                = addr;
    ((i2c_context_t*)result->context)->dev_handle          = dev_handle;
    ((i2c_context_t*)result->context)->transfer_timeout_ms = transfer_timeout_ms;

    return result;
}

esp_err_t i2c_read(void * i2c_handler_context, uint8_t* buffer, uint8_t buffer_size) {
	i2c_context_t * context = (i2c_context_t *) i2c_handler_context;

#if I2C_DEBUG_OUTPUT
	ESP_LOGI(LOG_I2C, "i2c_read send request to port addr %02x", context->addr);
#endif

	if (xSemaphoreTake(i2c_mutex, I2C_MUTEX_AWAIT) != pdTRUE) {
		ESP_LOGE(LOG_I2C, "i2c_read take mutex timeout for address %02X", context->addr);
		return ESP_ERR_TIMEOUT;
	}

	esp_err_t res = i2c_master_receive(context->dev_handle, buffer, buffer_size, context->transfer_timeout_ms);

    xSemaphoreGive(i2c_mutex);

#if I2C_DEBUG_OUTPUT
    if (res == ESP_OK) {
    	ESP_LOGI(LOG_I2C, "i2c_read received buffer from addr %02x", context->addr);
		ESP_LOG_BUFFER_HEXDUMP(LOG_I2C, buffer, buffer_size, ESP_LOG_INFO);
    }
#endif

    return res;
}

esp_err_t i2c_write(void * i2c_handler_context, const uint8_t* buffer, uint8_t buffer_size) {
	i2c_context_t * context = (i2c_context_t *) i2c_handler_context;

#if I2C_DEBUG_OUTPUT
	ESP_LOGI(LOG_I2C, "i2c_write sending buffer to addr %02x", context->addr);
	ESP_LOG_BUFFER_HEXDUMP(LOG_I2C, buffer, buffer_size, ESP_LOG_INFO);
#endif

	if (xSemaphoreTake(i2c_mutex, I2C_MUTEX_AWAIT) != pdTRUE) {
		ESP_LOGE(LOG_I2C, "i2c_write addr_id %d take mutex timeout", context->addr);
		return ESP_ERR_TIMEOUT;
	}

	esp_err_t res = i2c_master_transmit(context->dev_handle, buffer, buffer_size, context->transfer_timeout_ms);

    xSemaphoreGive(i2c_mutex);

    return res;
}

esp_err_t i2c_write_read(void * i2c_handler_context, const uint8_t* write_buffer, uint8_t write_buffer_size, uint8_t* read_buffer, uint8_t read_buffer_size) {
	i2c_context_t * context = (i2c_context_t *) i2c_handler_context;

#if I2C_DEBUG_OUTPUT
	ESP_LOGI(LOG_I2C, "i2c_write_read send buffer to addr %02x", context->addr);
	ESP_LOG_BUFFER_HEXDUMP(LOG_I2C, write_buffer, write_buffer_size, ESP_LOG_INFO);
#endif

	memset(read_buffer, 0xAB, read_buffer_size);

	if (xSemaphoreTake(i2c_mutex, I2C_MUTEX_AWAIT) != pdTRUE) {
		ESP_LOGE(LOG_I2C, "i2c_write_read take mutex timeout for address %d", context->addr);
		return ESP_ERR_TIMEOUT;
	}

	esp_err_t res = i2c_master_transmit_receive(context->dev_handle, write_buffer, write_buffer_size, read_buffer, read_buffer_size, context->transfer_timeout_ms);

	xSemaphoreGive(i2c_mutex);

#if I2C_DEBUG_OUTPUT
	if (res == ESP_OK) {
		ESP_LOGI(LOG_I2C, "i2c_write_read received buffer from addr %02x", context->addr);
		ESP_LOG_BUFFER_HEXDUMP(LOG_I2C, read_buffer, read_buffer_size, ESP_LOG_INFO);
	}
#endif

	return res;
}
