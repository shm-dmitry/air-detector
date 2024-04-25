#ifndef MAIN_UART_PMS7003_PMS7003_DEF_H_
#define MAIN_UART_PMS7003_PMS7003_DEF_H_

#include "stdint.h"

typedef struct {
	uint16_t atmospheric_pm_1_0;
	uint16_t atmospheric_pm_2_5;
	uint16_t atmospheric_pm_10_0;
} pms7003_data_t;

#endif /* MAIN_UART_PMS7003_PMS7003_DEF_H_ */
