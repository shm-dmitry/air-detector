#ifndef MAIN_UART_UART_CORE_H_
#define MAIN_UART_UART_CORE_H_

#include "stdint.h"
#include "stdbool.h"
#include "esp_err.h"

typedef esp_err_t (*uart_core_validate_buffer)(const uint8_t * send, const uint8_t * reply);

esp_err_t uart_core_init(const char * tag, uint8_t port, uint8_t tx, uint8_t rx);
esp_err_t uart_core_send_buffer(const char * tag, uint8_t port, const uint8_t * send, uint8_t send_size, uint8_t * reply, uint8_t reply_size, uart_core_validate_buffer validator, uint16_t timeout);

#endif /* MAIN_UART_UART_CORE_H_ */
