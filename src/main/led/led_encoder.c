#include "led_encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_encoder.h"

#include "../log/log.h"

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *copy_encoder;
    rmt_encoder_t *bytes_encoder;
    rmt_symbol_word_t reset_chanel_symbol;
    uint8_t bytes_sent;
} rmt_fm_encoder_t;

static size_t IRAM_ATTR led_isr_rmt_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
	rmt_fm_encoder_t *encoder_data = __containerof(encoder, rmt_fm_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    rmt_encoder_handle_t bytes_encoder = encoder_data->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = encoder_data->copy_encoder;

    if (encoder_data->bytes_sent == 0) {
		encoded_symbols += copy_encoder->encode(copy_encoder, channel, &encoder_data->reset_chanel_symbol,
												sizeof(rmt_symbol_word_t), &session_state);
		if (session_state & RMT_ENCODING_COMPLETE) {
			encoder_data->bytes_sent = 1;
		}
		if (session_state & RMT_ENCODING_MEM_FULL) {
			state |= RMT_ENCODING_MEM_FULL;
			goto out;
		}
    }

    if (encoder_data->bytes_sent < data_size + 1) {
		encoded_symbols += bytes_encoder->encode(bytes_encoder,
												 channel,
												 primary_data,
												 data_size,
												 &session_state);
		if (session_state & RMT_ENCODING_COMPLETE) {
			encoder_data->bytes_sent += data_size;
		}
		if (session_state & RMT_ENCODING_MEM_FULL) {
			state |= RMT_ENCODING_MEM_FULL;
			goto out;
		}
    }

    encoded_symbols += copy_encoder->encode(copy_encoder, channel, &encoder_data->reset_chanel_symbol,
                                            sizeof(rmt_symbol_word_t), &session_state);
    if (session_state & RMT_ENCODING_COMPLETE) {
        encoder_data->bytes_sent = 0;
        state |= RMT_ENCODING_COMPLETE;
    }
    if (session_state & RMT_ENCODING_MEM_FULL) {
        state |= RMT_ENCODING_MEM_FULL;
        goto out;
    }

out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t IRAM_ATTR led_isr_rmt_del_encoder(rmt_encoder_t *encoder)
{
	rmt_fm_encoder_t *encoder_data = __containerof(encoder, rmt_fm_encoder_t, base);
    rmt_del_encoder(encoder_data->bytes_encoder);
    rmt_del_encoder(encoder_data->copy_encoder);
    free(encoder_data);
    return ESP_OK;
}

static esp_err_t IRAM_ATTR led_isr_rmt_reset_encoder(rmt_encoder_t *encoder)
{
	rmt_fm_encoder_t *encoder_data = __containerof(encoder, rmt_fm_encoder_t, base);
    rmt_encoder_reset(encoder_data->bytes_encoder);
    rmt_encoder_reset(encoder_data->copy_encoder);
    encoder_data->bytes_sent = 0;
    return ESP_OK;
}

void rmt_new_ir_nec_encoder(rmt_encoder_handle_t *ret_encoder) {
    rmt_fm_encoder_t *fm_encoder_iface = NULL;
    fm_encoder_iface = calloc(1, sizeof(rmt_fm_encoder_t));
    fm_encoder_iface->base.encode = led_isr_rmt_encode;
    fm_encoder_iface->base.del = led_isr_rmt_del_encoder;
    fm_encoder_iface->base.reset = led_isr_rmt_reset_encoder;

    // "reset" == LOW>80uS -> I use 90uS of LOW level to ensure command correctly read
    fm_encoder_iface->reset_chanel_symbol = (rmt_symbol_word_t) {
		.level0 = 0,
		.duration0 = 450,
		.level1 = 0,
		.duration1 = 450
    };

    // I have tolerance ±0.15uS, but I have min signal time 1.25±0.6uS.
    // So I increase time of each symbol part in tolerance limit to ensure
    // I use valid min signal time
    rmt_bytes_encoder_config_t bytes_encoder_config = {
    	// bit0: [LOW: 0.3uS; HIGH 0.9uS].
        .bit0 = {
            .level0 = 1,
            .duration0 = 4,
            .level1 = 0,
            .duration1 = 9,
        },
    	// bit1: [LOW: 0.6uS; HIGH 0.6uS].
        .bit1 = {
            .level0 = 1,
            .duration0 = 7,
            .level1 = 0,
            .duration1 = 6,
        },
		.flags = {
			.msb_first = true
		}
    };
    rmt_new_bytes_encoder(&bytes_encoder_config, &fm_encoder_iface->bytes_encoder);

    rmt_copy_encoder_config_t copy_encoder_config = {};
    rmt_new_copy_encoder(&copy_encoder_config, &fm_encoder_iface->copy_encoder);

    *ret_encoder = &fm_encoder_iface->base;
}
