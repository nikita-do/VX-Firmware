/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "app_main.h"

#include "adpd144.h"
#include "cJSON.h"
#include "mqtt_client.h"

/**************************************************************************************************
 *                                      Macro Definition
 **************************************************************************************************/
#define SAMPLE_FREQUENCY 50 // Hz
#define SAMPLE_PERIOD (1000 / SAMPLE_FREQUENCY) // ms
#define RELOAD_TIMER_PERIOD pdMS_TO_TICKS(SAMPLE_PERIOD)

/**************************************************************************************************
 *                                     Global declaration
 **************************************************************************************************/
static const char *TAG = "app_main";
// Task handles
static TaskHandle_t xTimerTask = NULL;
// Extern varialbes
extern esp_mqtt_client_handle_t client;

static long unsigned int ppg_ir_value;
static long unsigned int ppg_red_value;

AdcConfig_t gsr = {
    .adc_unit = ADC_UNIT_1,
    .channel = ADC_CHANNEL_6,
    .unit_handle = NULL,
    .cali_handle = NULL,
    .raw_value = 0,
    .voltage_value = 0,
};

AdcConfig_t ecg = {
    .adc_unit = ADC_UNIT_2,
    .channel = ADC_CHANNEL_0,
    .unit_handle = NULL,
    .cali_handle = NULL,
    .raw_value = 0,
    .voltage_value = 0,
};

/**************************************************************************************************
 *                                  Timer Functions
 **************************************************************************************************/
static void prvAutoReloadTimerCallback(TimerHandle_t xTimer)
{
    // Notify the timer task
    if (xTimerTask != NULL)
    {
        xTaskNotifyGive(xTimerTask);
    }
}

esp_err_t timer_init(void)
{
    //-------------Timer Init---------------//
    TimerHandle_t xAutoReloadTimer = xTimerCreate("AutoReloadTimer", RELOAD_TIMER_PERIOD, pdTRUE, 0, prvAutoReloadTimerCallback);
    if (xAutoReloadTimer == NULL)
    {
        ESP_LOGE(TAG, "Timer Create Failed");
        return ESP_FAIL;
    }
    else
    {
        if (xTimerStart(xAutoReloadTimer, 0) != pdPASS)
        {
            ESP_LOGE(TAG, "Timer Start Failed");
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

/**************************************************************************************************
 *                                     MQTT Callback functions
 **************************************************************************************************/
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
/* MQTT Event Handler */
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to HiveMQ broker!");
        msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC("cmd"), 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC("status"), "online", 0, 1, 1);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Received MQTT message on topic: %.*s, data: %.*s", event->topic_len, event->topic, event->data_len, event->data);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to display the log

        // Check if the received cmd is start
        if (strncmp(event->data, "start", event->data_len) == 0)
        {
            ESP_LOGI(TAG, "Received 'start' command, publishing data...");
            vTaskResume(xTimerTask);
        }

        // Check if the received cmd is stop
        if (strncmp(event->data, "stop", event->data_len) == 0)
        {
            ESP_LOGI(TAG, "Received 'stop' command, stopping data publishing...");
            vTaskSuspend(xTimerTask);
        }
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
        // ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/**************************************************************************************************
 *	                                    Task functions
 **************************************************************************************************/
void timer_read_sensors(void *arg)
{
    // Create a JSON object
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "time", "");
    cJSON_AddNumberToObject(json, "gsr", 0);
    cJSON_AddNumberToObject(json, "ecg", 0);
    cJSON_AddNumberToObject(json, "ppg_ir", 0);
    cJSON_AddNumberToObject(json, "ppg_red", 0);

    int msg_id;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //-------------ADC1 Oneshot Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_read(gsr.unit_handle, gsr.channel, &gsr.raw_value));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(gsr.cali_handle, gsr.raw_value, &gsr.voltage_value));

        //-------------ADC2 Oneshot Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_read(ecg.unit_handle, ecg.channel, &ecg.raw_value));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(ecg.cali_handle, ecg.raw_value, &ecg.voltage_value));

        //-------------Read PPG Values---------------//
        ESP_ERROR_CHECK(adpd144_readIRValue(&ppg_ir_value, 1));
        ESP_ERROR_CHECK(adpd144_readRedValue(&ppg_red_value, 1));

        // Print data for serial visualizing 
        printf("gsr:%u,ecg:%u,ir:%ld,red:%ld\n", gsr.voltage_value, ecg.voltage_value, ppg_ir_value, ppg_red_value);

        cJSON_ReplaceItemInObject(json, "time", cJSON_CreateString(get_timestamp()));
        cJSON_ReplaceItemInObject(json, "gsr", cJSON_CreateNumber(gsr.voltage_value));
        cJSON_ReplaceItemInObject(json, "ecg", cJSON_CreateNumber(ecg.voltage_value));
        cJSON_ReplaceItemInObject(json, "ppg_ir", cJSON_CreateNumber(ppg_ir_value));
        cJSON_ReplaceItemInObject(json, "ppg_red", cJSON_CreateNumber(ppg_red_value));

        // Convert JSON to string
        char *message = cJSON_PrintUnformatted(json);

        // Publish the data with QoS 1
        esp_mqtt_client_publish(client, MQTT_TOPIC("data") , message, 0, 0, 0);

        // Free the JSON string
        free(message);
    }
}

/**************************************************************************************************
 *                                      Main application
 **************************************************************************************************/
void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_WARN);

    // Uncomment for debugging
    // esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    // esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    // esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    // esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    // esp_log_level_set("transport", ESP_LOG_VERBOSE);
    // esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    /* Initiate hardware */
    ESP_ERROR_CHECK(adc_oneshot_init(gsr.adc_unit, gsr.channel, &gsr.unit_handle, &gsr.cali_handle));
    ESP_ERROR_CHECK(adc_oneshot_init(ecg.adc_unit, ecg.channel, &ecg.unit_handle, &ecg.cali_handle));
    ESP_ERROR_CHECK(timer_init());
    ESP_ERROR_CHECK(adpd144_init());

    wifi_provisioning();

    check_time();

    mqtt_app_start();

    // Create the timer task
    xTaskCreate(timer_read_sensors, "TimerTask", 4096, NULL, 5, &xTimerTask);
    vTaskSuspend(xTimerTask);
}