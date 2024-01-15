#ifndef MAIN_I2C_SGP41_SGP41_API_H_
#define MAIN_I2C_SGP41_SGP41_API_H_

#include "../i2c_impl.h"
#include "../sgp41/sgp41_def.h"

#define SGP41_VALUE_NODATA 0xFFFF

void sgp41_set_temp_humidity(int8_t temperature, uint8_t humidity);

esp_err_t sgp41_api_init(i2c_handler_t * i2c);

esp_err_t sgp41_read(sgp41_data_t * result);

#endif /* MAIN_I2C_SGP41_SGP41_API_H_ */
