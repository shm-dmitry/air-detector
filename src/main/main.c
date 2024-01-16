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
#include "mq136/mq136.h"
#include "light/light.h"

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

#if CONFIG_MQ136_ENABLED
	mq136_init();
#endif

#if CONFIG_LIGHT_ENABLED
	light_init();
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
}
