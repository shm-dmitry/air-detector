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
#include "adc/mq136/mq136.h"
#include "adc/light/light.h"
#include "adc/o2a2/o2a2.h"
#include "adc/mq7/mq7.h"
#include "adc/adc.h"
#include "common/wifi.h"
#include "common/nvs_rw.h"
#include "common/mqtt.h"
#include "uart/mh_z19b/mh_z19b.h"
#include "uart/pms7003/pms7003.h"

void app_main(void)
{
	nvs_init();
	wifi_init();

#if CONFIG_LED_ENABLED
	led_init();
#endif

#if CONFIG_I2C_ENABLED
	i2c_init_driver(CONFIG_I2C_GPIO_SDA, CONFIG_I2C_GPIO_SCL);
#endif

#if CONFIG_BME280_ENABLED
	bme280_init();
#endif

#if CONFIG_SGP41_ENABLED
	sgp41_init();
#endif

#if CONFIG_MQ136_ENABLED || CONFIG_LIGHT_ENABLED
	adc_init();
#endif

#if CONFIG_MQ136_ENABLED
	mq136_init();
#endif

#if CONFIG_MQ7_ENABLED
	mq7_init();
#endif

#if CONFIG_LIGHT_ENABLED
	light_init();
#endif

#if CONFIG_O2A2_ENABLED
	o2a2_init();
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

#if CONFIG_MHZ19B_ENABLED
	mhz19b_init();
#endif

#if CONFIG_PMS7003_ENABLED
	pms7003_init();
#endif

	mqtt_start();

	LOGI(LOG_MAIN, "Application started");

	while(true) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
