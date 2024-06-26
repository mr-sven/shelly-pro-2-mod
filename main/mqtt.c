/**
 * @file mqtt.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief functions around mqtt client
 * @version 0.1
 * @date 2023-11-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "freertos/FreeRTOS.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "mqtt_client.h"

#include "mqtt.h"

static const char *TAG = "MQTT";

esp_mqtt_client_handle_t client = NULL;

extern void mqtt_connected(void);
extern void mqtt_data(char *topic, char *data);

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

int mqtt_publish(const char *topic, const char *data, int len, int qos, int retain)
{
    return esp_mqtt_client_publish(client, topic, data, len, qos, retain);
}

int mqtt_subscribe(const char *topic, int qos)
{
    ESP_LOGI(TAG, "Subscribe: %s", topic);
    return esp_mqtt_client_subscribe_single(client, topic, qos);
}

int mqtt_subscribe_multi(const esp_mqtt_topic_t *topic_list, int size)
{
    ESP_LOGI(TAG, "Subscribe multi");
    return esp_mqtt_client_subscribe_multiple(client, topic_list, size);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    char topic[128];
    char data[128];
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        mqtt_connected();
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        sprintf(topic, "%.*s", event->topic_len, event->topic);
        sprintf(data, "%.*s", event->data_len, event->data);
        mqtt_data(topic, data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(char *lwt_topic, const char *lwt_data, int lwt_data_len, int qos, int retain)
{
    esp_mqtt_client_config_t mqtt_cfg = {
            .broker.address.uri = CONFIG_SHELLY_BROKER_URL,
            .session.last_will  = {
                    .topic   = lwt_topic,
                    .msg     = lwt_data,
                    .msg_len = lwt_data_len,
                    .qos     = qos,
                    .retain  = retain,
            },
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}