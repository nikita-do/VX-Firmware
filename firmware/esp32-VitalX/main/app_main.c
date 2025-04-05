/**
 * @file app_main.c
 * @brief Main application file
 *
 * The application reads the GSR, ECG, PPG IR, and PPG Red values from the sensors and publishes them to the MQTT broker. The application also subscribes to the MQTT broker for the commands to start and stop the data publishing.
 *
 * @version 1.0.1
 * @date 2025-04-03
 */
#include "app_main.h"

#include "adpd144.h"
#include "cJSON.h"
#include "mqtt_client.h"
#include "esp_timer.h"

/**************************************************************************************************
 *                                      Macro Definition
 **************************************************************************************************/
#define SAMPLE_FREQUENCY 100                     // Hz
#define SAMPLE_PERIOD (1000 / SAMPLE_FREQUENCY) // ms
#define RELOAD_TIMER_PERIOD pdMS_TO_TICKS(SAMPLE_PERIOD)
#define SAMPLE_BATCH 10 // Number of samples to be sent in one batch

/**************************************************************************************************
 *                                     Global declaration
 **************************************************************************************************/
static const char *TAG = "app_main";
// Task handles
static TaskHandle_t xTimerTask = NULL;
static TaskHandle_t xMqttTask = NULL;

void timer_read_sensor_task(void *arg);
void mqtt_publish_task(void *arg);

// Extern varialbes
extern esp_mqtt_client_handle_t client;

static uint32_t gsr_array[SAMPLE_BATCH] = {0};
static uint32_t ecg_array[SAMPLE_BATCH] = {0};
static uint32_t ir_array[SAMPLE_BATCH] = {0};
static uint32_t red_array[SAMPLE_BATCH] = {0};
static uint32_t time_array[SAMPLE_BATCH] = {0};

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
            /* Initiate PPG */
            adpd144_start();
            // Create the timer task
            xTaskCreate(timer_read_sensor_task, "TimerTask", 4096, NULL, 5, &xTimerTask);
        }

        // Check if the received cmd is stop
        if (strncmp(event->data, "stop", event->data_len) == 0)
        {
            ESP_LOGI(TAG, "Received 'stop' command, stopping data publishing...");
            vTaskDelete(xTimerTask);
            xTimerTask = NULL; // Reset the task handle
            adpd144_stop();
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

void timer_read_sensor_task(void *arg)
{
    int64_t start_time = esp_timer_get_time();
    int64_t elapsed_time_ms = 0;
    uint8_t sample_count = 0;

    while (1)
    {
        // Wait for the timer to notify
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //-------------ADC1 Oneshot Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_read(gsr.unit_handle, gsr.channel, &gsr.raw_value));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(gsr.cali_handle, gsr.raw_value, &gsr.voltage_value));

        //-------------ADC2 Oneshot Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_read(ecg.unit_handle, ecg.channel, &ecg.raw_value));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(ecg.cali_handle, ecg.raw_value, &ecg.voltage_value));

        //-------------Read PPG Values---------------//
        ESP_ERROR_CHECK(adpd144_readIRValue(&ir_array[sample_count], 1));
        ESP_ERROR_CHECK(adpd144_readRedValue(&red_array[sample_count], 1));

        ecg_array[sample_count] = ecg.voltage_value;
        gsr_array[sample_count] = gsr.voltage_value;

        // Calculate elapsed time
        elapsed_time_ms = (esp_timer_get_time() - start_time) / 1000; // Convert microseconds to milliseconds

        time_array[sample_count] = elapsed_time_ms; // Store elapsed time in milliseconds

        sample_count++; // Increment sample count
        if (sample_count == SAMPLE_BATCH)
        {
            sample_count = 0; // Reset sample count
            // Notify the MQTT task to publish data
            xTaskNotifyGive(xMqttTask);
        }
    }
}
void mqtt_publish_task(void *arg)
{
    while (1)
    {
        // Wait for the timer task to notify
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Create a JSON object
        cJSON *json = cJSON_CreateObject();
        cJSON *gsr_msg_array = cJSON_CreateArray();
        cJSON *ecg_msg_array = cJSON_CreateArray();
        cJSON *ir_msg_array = cJSON_CreateArray();
        cJSON *red_msg_array = cJSON_CreateArray();
        cJSON *time_msg_array = cJSON_CreateArray();

        // Add stored sensor data to the JSON array
        for (int i = 0; i < SAMPLE_BATCH; i++)
        {
            cJSON_AddItemToArray(time_msg_array, cJSON_CreateNumber(time_array[i]));
            cJSON_AddItemToArray(gsr_msg_array, cJSON_CreateNumber(gsr_array[i]));
            cJSON_AddItemToArray(ecg_msg_array, cJSON_CreateNumber(ecg_array[i]));
            cJSON_AddItemToArray(ir_msg_array, cJSON_CreateNumber(ir_array[i]));
            cJSON_AddItemToArray(red_msg_array, cJSON_CreateNumber(red_array[i]));
        }

        cJSON_AddItemToObject(json, "time", time_msg_array);
        cJSON_AddItemToObject(json, "gsr", gsr_msg_array);
        cJSON_AddItemToObject(json, "ecg", ecg_msg_array);
        cJSON_AddItemToObject(json, "ir", ir_msg_array);
        cJSON_AddItemToObject(json, "red", red_msg_array);

        // Convert JSON to string
        char *message = cJSON_PrintUnformatted(json);

        // Publish the data with QoS 1
        esp_mqtt_client_publish(client, MQTT_TOPIC("data"), message, 0, 1, 0);

        // Free the JSON string
        free(message);
        cJSON_Delete(json); // Free the JSON object
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

    esp_log_level_set("*", ESP_LOG_MAX);

    /* Initialize hardware */
    ESP_ERROR_CHECK(adc_oneshot_init(gsr.adc_unit, gsr.channel, &gsr.unit_handle, &gsr.cali_handle));
    ESP_ERROR_CHECK(adc_oneshot_init(ecg.adc_unit, ecg.channel, &ecg.unit_handle, &ecg.cali_handle));
    ESP_ERROR_CHECK(timer_init());
    ESP_ERROR_CHECK(adpd144_init());

#ifdef CONFIG_EXAMPLE_WIFI_PROV_MODE
    wifi_provisioning();
#endif

#ifdef CONFIG_EXAMPLE_WIFI_STAT_MODE
    wifi_init_sta();
#endif

    check_time();

    mqtt_app_start();

    // Create the MQTT task
    xTaskCreate(mqtt_publish_task, "MqttTask", 4096, NULL, 5, &xMqttTask);
    if (xMqttTask == NULL)
    {
        ESP_LOGE(TAG, "Failed to create MQTT task");
        return;
    }
}