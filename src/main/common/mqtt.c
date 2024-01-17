#include "mqtt.h"

#include "mqtt_client.h"
#include "nvs_rw.h"
#include "sdkconfig.h"

#include "../log/log.h"

#define MAX_MQTT_URI_LEN 100
#define MAX_MQTT_USER_PASS 20

typedef struct mqtt_callback_mapping_t {
	char * topic;
	mqtt_topic_callback_t function;
} mqtt_callback_mapping_t;

mqtt_callback_mapping_t* callbacks = NULL;
uint8_t callbacks_count = 0;
esp_mqtt_client_handle_t client;

static void mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
	if (callbacks_count == 0 || event == NULL) {
		return;
	}

	switch (event->event_id) {
	case MQTT_EVENT_CONNECTED:
		for (int i = 0; i<callbacks_count; i++) {
			if (callbacks[i].topic) {
				esp_mqtt_client_subscribe_single(event->client, callbacks[i].topic, 0);
			}
		}
		break;
	case MQTT_EVENT_DATA:
		if (event->data && event->topic && event->data_len && event->topic_len && event->data_len < 1024 && event->topic_len < 1024) {
			char * topic = malloc(event->topic_len + 1);
			memset(topic, 0, event->topic_len + 1);
			memcpy(topic, event->topic, event->topic_len);

			char * data = malloc(event->data_len + 1);
			memset(data, 0, event->data_len + 1);
			memcpy(data, event->data, event->data_len);

			ESP_LOGI(LOG_MQTT, "Received message in topic %s : %s", topic, data);

			for (int i = 0; i<callbacks_count; i++) {
				if (callbacks[i].topic && strcmp(topic, callbacks[i].topic) == 0) {
					callbacks[i].function(topic, data);
				}
			}

			free(topic);
			free(data);
		}
		break;
	default:
		break;
	}
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
	mqtt_event_handler_cb(event_data);
}

char * mqtt_prepend_prefix(const char * topic) {
	if (topic == NULL) {
		return NULL;
	}

	char * str = malloc(strlen(CONFIG_MQTT_TOPICS_PREFIX) + strlen(topic) + 1);
	if (str == NULL) {
		return NULL;
	}

	strcpy(str, CONFIG_MQTT_TOPICS_PREFIX);
	strcpy(str + strlen(CONFIG_MQTT_TOPICS_PREFIX), topic);

	return str;
}

void mqtt_publish_sync(const char * topic, const char * message) {
	if (client) {
		char * temp = mqtt_prepend_prefix(topic);
		if (temp == NULL) {
			return;
		}

		if (esp_mqtt_client_publish(client, temp, message, 0, 0, 1) >= 0) {
	    	ESP_LOGI(LOG_MQTT, "MQTT publish OK topic = %s, message = %s", temp, message);
		} else {
			mqtt_publish(topic, message);
		}

		free(temp);
		temp = NULL;
	}
}

void mqtt_publish(const char * topic, const char * message) {
	if (client) {
		char * temp = mqtt_prepend_prefix(topic);
		if (temp == NULL) {
			return;
		}

		if (esp_mqtt_client_enqueue(client, temp, message, 0, 0, 1, 1) >= 0) {
	    	ESP_LOGI(LOG_MQTT, "MQTT enqueue OK topic = %s, message = %s", temp, message);
		} else {
	    	ESP_LOGE(LOG_MQTT, "MQTT enqueue error: topic = %s, message = %s", temp, message);
		}

		free(temp);
		temp = NULL;
	}
}

void mqtt_subscribe(const char * topic, mqtt_topic_callback_t callback) {
	if (callback == NULL) {
		ESP_LOGW(LOG_MQTT, "Cant subscribe for a topic %s. Callback is NULL.", topic);
		return;
	}

	mqtt_callback_mapping_t* newtable = (mqtt_callback_mapping_t*) malloc(sizeof(mqtt_callback_mapping_t) * (callbacks_count + 1));
	memset(newtable, 0, sizeof(mqtt_callback_mapping_t) * (callbacks_count + 1));

	if (callbacks_count > 0) {
		memcpy(newtable, callbacks, sizeof(mqtt_callback_mapping_t) * callbacks_count);
		free(callbacks);
		callbacks = NULL;
	}

	callbacks = newtable;

	callbacks[callbacks_count].topic = mqtt_prepend_prefix(topic);
	callbacks[callbacks_count].function = callback;

	if (callbacks[callbacks_count].topic) {
		ESP_LOGI(LOG_MQTT, "Client subscribed on topic %s", callbacks[callbacks_count].topic);
	} else {
		ESP_LOGE(LOG_MQTT, "Cant allocate memory to subscribe on topic %s%s", CONFIG_MQTT_TOPICS_PREFIX, topic);
	}

	callbacks_count++;
}

void mqtt_start() {
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = CONFIG_MQTT_BROKER_URI,
		.credentials = {
			.username = CONFIG_MQTT_BROKER_USERNAME,
			.authentication.password = CONFIG_MQTT_BROKER_PASSWORD,
		},
	};

	client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (esp_mqtt_client_start(client)) {
    	client = NULL;
    	ESP_LOGE(LOG_MQTT, "Cant start MQTT client!");
    } else {
    	ESP_LOGI(LOG_MQTT, "MQTT started");
    }
}
