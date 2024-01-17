#ifndef MAIN_I2C_SGP41_SGP41_API_H_
#define MAIN_I2C_SGP41_SGP41_API_H_

#include "../sgp41/sgp41_def.h"

#include "stdint.h"
#include "esp_err.h"

#define SGP41_VALUE_NODATA 0xFFFF
#define SGP41_SAMPLING_INTERVAL 10

void sgp41_set_temp_humidity(int8_t temperature, uint8_t humidity);

esp_err_t sgp41_api_init();

esp_err_t sgp41_read(sgp41_data_t * result);

#endif /* MAIN_I2C_SGP41_SGP41_API_H_ */
