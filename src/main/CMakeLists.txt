# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS "main.c"
    	 "cjson/cjson_helper.c"
    	 "common/mqtt.c"
    	 "common/mqtt_healthcheck.c"
    	 "common/mqtt_ota.c"
    	 "common/nvs_rw.c"
    	 "common/wifi_nvs.c"
    	 "common/wifi.c"
    	 "led/led.c"
    	 "led/led_encoder.c"
    	 "i2c/i2c_impl.c"
    	 "i2c/sgp41/sgp41_api.c"
    	 "i2c/sgp41/sgp41.c"
    	 "i2c/sgp41/raw2index/sensirion_gas_index_algorithm.c"
    	 "i2c/bme280/bme280.c"
    	 "i2c/bme280/bme280_api.c"
    	 "i2c/bme280/bme280_math.c"
    	 "touchpad/touchpad.c"
    	 "touchpad/touchpad_api.c"
    	 "fans/fan/fan.c"
    	 "fans/fan_pwm/fan_pwm.c"
    	 "fans/fan_pwm/fan_pwm_api.c"
    	 "fans/fan_pwm/fan_pwm_nvs.c"
    	 "adc/adc_v_core/adc_v_core.c"
    	 "adc/adc_v_core/adc_v_core_nvs.c"
    	 "adc/mq136/mq136.c"
    	 "adc/o2a2/o2a2.c"
    	 "adc/mq7/mq7.c"
    	 "adc/light/light.c"
    	 "adc/adc.c"
    	 "uart/mh_z19b/mh_z19b.c"
    	 "uart/pms7003/pms7003.c"
    INCLUDE_DIRS        # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES            # optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
)
