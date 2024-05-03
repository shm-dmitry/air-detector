#ifndef MAIN_LOG_LOG_H_
#define MAIN_LOG_LOG_H_

#include "esp_log.h"

#include "sdkconfig.h"

#define LOG_ALWAYS_ENABLED 0

// PMS7003 shares UART port with USB.
// I disable logging to ensure some log messages
// will not decoded by sensor as 'command'
#if CONFIG_PMS7003_ENABLED && !LOG_ALWAYS_ENABLED
#undef LOG_ALLOWED
#define LOG_ALLOWED 0
#else
#define LOG_ALLOWED 1
#endif

#if LOG_ALLOWED
#define LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define LOG_BUFFER_HEXDUMP(tag, buffer, buff_len, level) ESP_LOG_BUFFER_HEXDUMP(tag, buffer, buff_len, level)
#else
#define LOGE(tag, format, ...)
#define LOGI(tag, format, ...)
#define LOGW(tag, format, ...)
#define LOG_BUFFER_HEXDUMP(tag, buffer, buff_len, level)
#endif

#define LOG_NWS_RW       "nws_rw"
#define LOG_WIFI         "wifi"
#define LOG_MQTT		 "mqtt"
#define LOG_LED			 "led"
#define LOG_SGP41		 "sgp41"
#define LOG_I2C			 "i2c"
#define LOG_BME280       "bme280"
#define LOG_TOUCHPAD     "touchpad"
#define LOG_FAN          "fan"
#define LOG_FANPWM		 "fanpwm"
#define LOG_MQ136		 "mq136"
#define LOG_MQ7			 "mq7"
#define LOG_LIGHT		 "light"
#define LOG_O2A2		 "o2a2"
#define LOG_OTA			 "ota"
#define LOG_MHZ19B		 "mhz19b"
#define LOG_PMS7003		 "pms7003"
#define LOG_MAIN		 "main"

#endif /* MAIN_LOG_LOG_H_ */
