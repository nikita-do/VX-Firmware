/**
 * @file app_mqtt.c
 * @brief MQTT client implementation for ESP32, derived from esp-idf mqtt ssl example.
 * 
 * @version 0.2.0
 * @date 2024-04-09
 * 
 * --------------- Change log ----------------
 * v0.2.0 - Refactored topic subscription system
 * v0.1.0 - Initial version, basic publish/subscribe
 */
#include "app_main.h"
#include "mqtt_client.h"

static const char *TAG = "app_mqtt";
esp_mqtt_client_handle_t client = NULL;

#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_isrgrootx1_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_isrgrootx1_pem_start[]   asm("_binary_isrgrootx1_pem_start");
#endif
extern const uint8_t mqtt_isrgrootx1_pem_end[]   asm("_binary_isrgrootx1_pem_end");

extern void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void mqtt_app_start(void)
{
    char mqtt_topics_status_online[64];
    snprintf(mqtt_topics_status_online, sizeof(mqtt_topics_status_online), "device/%s/status_online", service_name);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = CONFIG_BROKER_URL,
            .verification.certificate = (const char *)mqtt_isrgrootx1_pem_start
        },
        .credentials ={               
            .username= CONFIG_BROKER_USERNAME,
            .authentication.password = CONFIG_BROKER_PASSWORD          
        },
        .session = {
            .last_will.topic = mqtt_topics_status_online,
            .last_will.msg = "false",
            .last_will.qos = 1,
            .last_will.retain = 1, // Ensure message is retained
            .keepalive = 30, // After 0.5 minutes of inactivity, the broker will disconnect the client -> status = offline
        }
    };

    client = esp_mqtt_client_init(&mqtt_cfg);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
