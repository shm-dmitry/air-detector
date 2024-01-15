#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "sdkconfig.h"

#include "log/log.h"
#include "led/led.h"
#include "i2c/sgp41/sgp41.c"
#include "i2c/i2c_impl.h"
#include "touchpad/touchpad.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fans/fan/fan.h"
#include "fans/fan_pwm/fan_pwm.h"

void app_main(void)
{
#if CONFIG_LED_ENABLED
	led_init();
#endif

#if CONFIG_I2C_ENABLED
	i2c_register_port(I2C_NUM_0, CONFIG_I2C_GPIO_SDA, CONFIG_I2C_GPIO_SCL);
#endif

#if CONFIG_SGP41_ENABLED
	sgp41_init();
#endif

#if CONFIG_BME280_ENABLED
	bme280_init();
#endif

#if CONFIG_TOUCHPAD_ENABLED
	touchpad_init();
#endif

#if CONFIG_FAN_ENABLED
	fan_init();
#endif

#if CONFIG_FANPWM_ENABLED
	fanpwm_init();
#endif

	while(true) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	sgp41_data_t result;
	while(true) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		esp_err_t res = sgp41_read(&result);
		if (res == ESP_OK) {
			ESP_LOGI("main", "sgp41 result: NOXraw = %04X, TVOCraw = %04X; NOX = %d; TVOC = %d", result.nox_raw, result.tvoc_raw, result.nox, result.tvoc);
		}
	}
}
