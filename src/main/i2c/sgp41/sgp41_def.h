#ifndef MAIN_I2C_SGP41_SGP41_DEF_H_
#define MAIN_I2C_SGP41_SGP41_DEF_H_

#include "stdint.h"

typedef struct sgp41_data_t {
	uint16_t tvoc;
	uint16_t nox;
	uint16_t tvoc_raw;
	uint16_t nox_raw;
} sgp41_data_t;

#endif /* MAIN_I2C_SGP41_SGP41_DEF_H_ */
