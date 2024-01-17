#ifndef MAIN_I2C_I2C_IMPL_H_
#define MAIN_I2C_I2C_IMPL_H_

#include "stdint.h"
#include "esp_err.h"

typedef esp_err_t (* i2c_read_function)(void * i2c_handler_context, uint8_t* buffer, uint8_t buffer_size);
typedef esp_err_t (* i2c_write_function)(void * i2c_handler_context, const uint8_t* buffer, uint8_t buffer_size);
typedef esp_err_t (* i2c_write_read_function)(void * i2c_handler_context, const uint8_t* write_buffer, uint8_t write_buffer_size, uint8_t* read_buffer, uint8_t read_buffer_size);

typedef struct i2c_handler_t {
	void * context;
	i2c_read_function read;
	i2c_write_function write;
	i2c_write_read_function write_read;
} i2c_handler_t;

void i2c_init_driver(int gpio_sda, int gpio_scl);

i2c_handler_t * i2c_get_handlers(uint8_t addr, uint16_t transfer_timeout_ms);

#endif /* MAIN_I2C_I2C_IMPL_H_ */
