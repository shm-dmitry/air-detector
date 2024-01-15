# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS "main.c"
    	 "cjson/cjson_helper.c"
    	 "common/mqtt.c"
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
    INCLUDE_DIRS        # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES            # optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
)
