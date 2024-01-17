#include "led.h"

#include "led_encoder.h"
#include "../log/log.h"
#include "../common/mqtt.h"

#include "cJSON.h"
#include "string.h"

#include "driver/rmt_tx.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define LED_SENDER_TASK_STACK_SIZE	2048
#define LED_OVERRIDE_COLOR_NODATA	0x00000000

rmt_channel_handle_t tx_channel = NULL;
rmt_encoder_handle_t led_send_encoder = NULL;
QueueHandle_t led_send_queue = NULL;
uint32_t led_override_color_rgbw = LED_OVERRIDE_COLOR_NODATA;
uint32_t led_color_rgbw          = LED_OVERRIDE_COLOR_NODATA;

void led_update_color();

void led_commands(const char * topic, const char * data) {
	cJSON *root = cJSON_Parse(data);
	if (root == NULL) {
		return;
	}

	char * type = cJSON_GetStringValue(cJSON_GetObjectItem(root, "type"));
	if (strcmp(type, "set_color") == 0) {
		char * rgbs = cJSON_GetStringValue(cJSON_GetObjectItem(root, "rgb"));
		if (strlen(rgbs) == 0) {
			return;
		}

		char* invptr = NULL;
		uint32_t rgb = strtoul(rgbs, &invptr, 16);
		if (invptr == NULL || invptr == rgbs + strlen(rgbs)) {
			led_set_color(rgb);
		} else {
			ESP_LOGE(LOG_LED, "Cant parse RGB color %s. Bad char at position %d", rgbs, (int)(invptr - rgbs));
		}
	}

	cJSON_Delete(root);
}

static void led_sender_task(void* arg) {
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
		.flags.eot_level = 0 // TODO: think about set 1 here - it will reduce 10mA current consumption
    };

    while(true) {
		uint32_t value;
		xQueueReceive(led_send_queue, &value, portMAX_DELAY);

		uint8_t * buffer = (uint8_t *)&value;
/*
		ESP_LOGI(LOG_LED, "Sending [ %02X %02X %02X %02X ]",
				buffer[0],
				buffer[1],
				buffer[2],
				buffer[3]
				);
*/

		ESP_ERROR_CHECK(rmt_transmit(tx_channel,
									 led_send_encoder,
									 buffer,
									 4,
									 &transmit_config));

		rmt_tx_wait_all_done(tx_channel, 100);
	}
}

void led_set_color(uint32_t rgbw) {
	led_color_rgbw = rgbw;
	led_update_color();
}

void led_set_override_color(uint32_t rgbw) {
	led_override_color_rgbw = rgbw;
	led_update_color();
}

void led_reset_override_color() {
	led_override_color_rgbw = LED_OVERRIDE_COLOR_NODATA;
	led_update_color();
}

void led_init() {
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = CONFIG_LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = 10 * 1000 * 1000,
        .trans_queue_depth = 1,
		.flags = {
			.invert_out = 1
		}
    };


    esp_err_t res = rmt_new_tx_channel(&tx_chan_config, &tx_channel);
    if (res) {
		ESP_LOGE(LOG_LED, "rmt_new_tx_channel error: %d", res);
		return;
    }

    res = rmt_enable(tx_channel);
    if (res) {
		ESP_LOGE(LOG_LED, "rmt_enable error: %d", res);
		return;
    }

    rmt_new_ir_nec_encoder(&led_send_encoder);

    led_send_queue = xQueueCreate(10, sizeof(uint32_t));

	xTaskCreate(led_sender_task, "LED sender", LED_SENDER_TASK_STACK_SIZE, NULL, 10, NULL);

	mqtt_subscribe(CONFIG_LED_TOPIC_COMMANDS, led_commands);

	led_set_color(0);

    ESP_LOGI(LOG_LED, "LED initialized");
}

void led_update_color() {
	uint32_t rgbw = (led_override_color_rgbw == LED_OVERRIDE_COLOR_NODATA) ? led_color_rgbw : led_override_color_rgbw;

	uint8_t * temp = (uint8_t *)&rgbw;

	uint8_t w = temp[0];
	uint8_t b = temp[1];
	uint8_t g = temp[2];
	uint8_t r = temp[3];

	ESP_LOGI(LOG_LED, "LED: R = %02X; G = %02X; B = %02X; W = %02X", r, g, b, w);

	rgbw = 0;

	if (w == 0 && r == g && r == b) {
		temp[3] = r;
	} else {
		temp[0] = g;
		temp[1] = r;
		temp[2] = b;
		temp[3] = w;
	}

    xQueueSend(led_send_queue, &rgbw, ( TickType_t ) 10);
}

