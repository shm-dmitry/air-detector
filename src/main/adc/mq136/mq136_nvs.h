#ifndef MAIN_MQ136_MQ136_NVS_H_
#define MAIN_MQ136_MQ136_NVS_H_

#include "stdint.h"

typedef struct {
	uint16_t a0;
} mq136_nvs_data_t;

void mq136_nws_read(mq136_nvs_data_t * to);
void mq136_nws_write(const mq136_nvs_data_t * value);

#endif /* MAIN_MQ136_MQ136_NVS_H_ */
