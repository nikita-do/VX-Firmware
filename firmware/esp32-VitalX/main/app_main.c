/**
 * @file app_main.c
 * @brief Main application file
 *
 * The application reads the GSR, ECG, PPG IR, and PPG Red values from the sensors and publishes them to the MQTT broker. The application also subscribes to the MQTT broker for the commands to start and stop the data publishing.
 *
 * @version 0.2.0
 * @date 2025-04-09
 *
 * ---------------------- Change Log ----------------------
 * v0.1.0 - 2025-03-20
 *  - Integrated wifi configuration manager, MQTT client, and sensor reading tasks.
 *  - Added GSR, ECG, PPG IR, and PPG Red sensor reading tasks with sampling rate of 50 Hz
 * v0.1.1 - 2025-04-03
 *  - Sent data in batches of 100 samples to the MQTT broker.
 *  - Updated sampling rate to 100 Hz.
 *  - ADPD144 component updated: read from PD4 instead of PD3 for better ambient light cancelation
 *  - Improved code readability and organization.
 * v0.2.0 - 2025-04-09
 *  - Updated firmware version override via CMake
 *  - Modified the MQTT topic structure to include device ID and firmware version.
 *  - Added LED indication for connection status
 */
#include "app_main.h"
#include "driver/gpio.h"

#include "adpd144.h"
#include "cJSON.h"
#include "mqtt_client.h"
#include "esp_timer.h"

#ifdef HEAP_MONITOR_ENABLE
#include "esp_heap_caps.h"
#endif

/**************************************************************************************************
 *                                      Macro Definition
 **************************************************************************************************/
#define SAMPLE_FREQUENCY 100                    // Hz
#define SAMPLE_PERIOD (1000 / SAMPLE_FREQUENCY) // ms
#define SENSOR_READ_PERIOD pdMS_TO_TICKS(SAMPLE_PERIOD)
#define SAMPLE_BATCH 100 // Number of samples to be sent in one batch = 1 seconds of data

#define LED_GPIO GPIO_NUM_2 // GPIO2 for LED

#define ADD_NUM_ARRAY_ITEM(array, val)                 \
    do                                                 \
    {                                                  \
        cJSON *num = cJSON_CreateNumber(val);          \
        if (!num || !cJSON_AddItemToArray(array, num)) \
        {                                              \
            printf("❌ Failed to add array item\n");   \
            goto cleanup;                              \
        }                                              \
    } while (0)
/**************************************************************************************************
 *                                     Global declaration
 **************************************************************************************************/
static const char *TAG = "app_main";
// Task handles
static TaskHandle_t xTimerTask = NULL;
static TaskHandle_t xMqttTask = NULL;

// Mutex for protecting shared arrays
static SemaphoreHandle_t xArrayMutex = NULL;

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

static TimerHandle_t xLedBlinkTimer = NULL;
static TimerHandle_t xSensorReadTimer = NULL;
static uint32_t led_blink_period = LED_BLINK_PERIOD_DISCONNECTED;

static int64_t read_start_time = 0; // Start time for reading sensors
static int64_t elapsed_time_ms = 0;
static uint16_t sample_count = 0;

/**************************************************************************************************
 *                                  Timer Functions
 **************************************************************************************************/
static void prvSensorReadTimerCallback(TimerHandle_t xTimer)
{
    // Notify the timer task
    if (xTimerTask != NULL)
    {
        xTaskNotifyGive(xTimerTask);
    }
}

esp_err_t sensor_read_timer_init(void)
{
    //-------------Timer Init---------------//
    xSensorReadTimer = xTimerCreate("SensorReadTimer", SENSOR_READ_PERIOD, pdTRUE, 0, prvSensorReadTimerCallback);
    if (xSensorReadTimer == NULL)
    {
        ESP_LOGE(TAG, "Timer Create Failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void prvLedBlinkTimerCallback(TimerHandle_t xTimer)
{
    static bool led_state = false;
    gpio_set_level(LED_GPIO, led_state);
    led_state = !led_state;
}

void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Create LED blink timer
    xLedBlinkTimer = xTimerCreate("LedBlinkTimer", pdMS_TO_TICKS(led_blink_period), pdTRUE, NULL, prvLedBlinkTimerCallback);
    if (xLedBlinkTimer == NULL)
    {
        ESP_LOGE(TAG, "LED Blink Timer Create Failed");
    }
    else
    {
        xTimerStart(xLedBlinkTimer, 0);
    }
}

void update_led_blink_period(uint32_t period)
{
    led_blink_period = period;
    if (xLedBlinkTimer != NULL)
    {
        xTimerChangePeriod(xLedBlinkTimer, pdMS_TO_TICKS(led_blink_period), 0);
    }
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
        mqtt_publish_startUpMsg();                                      // Publish startup message
        esp_mqtt_client_subscribe(client, MQTT_TOPIC("commands/#"), 1); // Subscribe to the command topic
        ESP_LOGI(TAG, "Subscribed to topic: %s", MQTT_TOPIC("commands/#"));
        xTimerStop(xLedBlinkTimer, 0); // Stop LED blinking
        gpio_set_level(LED_GPIO, 1);   // Turn on LED when connected
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        update_led_blink_period(LED_BLINK_PERIOD_DISCONNECTED);
        xTimerStart(xLedBlinkTimer, 0); // Start LED blinking
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Received MQTT message on topic: %.*s, data: %.*s", event->topic_len, event->topic, event->data_len, event->data);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to display the log

        if (strncmp(event->topic, MQTT_TOPIC("commands/start"), event->topic_len) == 0)
        {
            // Check if the received cmd is start
            if (strncmp(event->data, "true", event->data_len) == 0)
            {
                esp_mqtt_client_publish(client, MQTT_TOPIC("responses/start"), "true", 0, 1, 1); // Publish response message
                ESP_LOGI(TAG, "Received 'start' command, publishing data...");
                /* Initiate PPG */
                adpd144_start();
                /* Start the timer task */
                read_start_time = esp_timer_get_time(); // Get the start time for reading sensors
                xTimerStart(xSensorReadTimer, 0);       // Start the timer
            }

            // Check if the received cmd is stop
            if (strncmp(event->data, "false", event->data_len) == 0)
            {
                ESP_LOGI(TAG, "Received 'stop' command, stopping data publishing...");
                xTimerStop(xSensorReadTimer, 0); // Stop the timer
                adpd144_stop();

                time_array[0] = 0; // Reset the time array
                gsr_array[0] = 0;  // Reset the GSR array
                ecg_array[0] = 0;  // Reset the ECG array
                ir_array[0] = 0;   // Reset the IR array
                red_array[0] = 0;  // Reset the Red array

                sample_count = 0; // Reset the sample count

                // Stop the timer task and PPG
                esp_mqtt_client_publish(client, MQTT_TOPIC("responses/start"), "false", 0, 1, 1); // Publish response message
            }
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

    while (1)
    {
        // Wait for the timer to notify
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Calculate elapsed time
        elapsed_time_ms = (esp_timer_get_time() - read_start_time) / 1000; // Convert microseconds to milliseconds

        // Lock the mutex before accessing shared arrays
        if (xSemaphoreTake(xArrayMutex, portMAX_DELAY) == pdTRUE)
        {
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

            time_array[sample_count] = elapsed_time_ms; // Store elapsed time in milliseconds
            // Release the mutex
            xSemaphoreGive(xArrayMutex);

            sample_count++; // Increment sample count
            if (sample_count == SAMPLE_BATCH)
            {
                sample_count = 0; // Reset sample count
                // Notify the MQTT task to publish data
                xTaskNotifyGive(xMqttTask);
            }
        }
    }
}
void mqtt_publish_task(void *arg)
{

    char *message = NULL;

    while (1)
    {
        // Wait for the timer task to notify
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#ifdef HEAP_MONITOR_ENABLE
        check_heap_status();
#endif

        // Create root JSON object and arrays
        cJSON *json = cJSON_CreateObject();
        cJSON *time_arr = cJSON_CreateArray();
        cJSON *gsr_arr = cJSON_CreateArray();
        cJSON *ecg_arr = cJSON_CreateArray();
        cJSON *ir_arr = cJSON_CreateArray();
        cJSON *red_arr = cJSON_CreateArray();

        // Add arrays to root JSON object
        if (!cJSON_AddItemToObject(json, "time", time_arr) ||
            !cJSON_AddItemToObject(json, "gsr", gsr_arr) ||
            !cJSON_AddItemToObject(json, "ecg", ecg_arr) ||
            !cJSON_AddItemToObject(json, "ir", ir_arr) ||
            !cJSON_AddItemToObject(json, "red", red_arr)) {
            printf("❌ Failed to add arrays to root object\n");
            goto cleanup;
        }

        if (xSemaphoreTake(xArrayMutex, portMAX_DELAY) == pdTRUE)
        {
            // Fill arrays
            for (int i = 0; i < SAMPLE_BATCH; i++)
            {
                ADD_NUM_ARRAY_ITEM(time_arr, time_array[i]);
                ADD_NUM_ARRAY_ITEM(gsr_arr, gsr_array[i]);
                ADD_NUM_ARRAY_ITEM(ecg_arr, ecg_array[i]);
                ADD_NUM_ARRAY_ITEM(ir_arr, ir_array[i]);
                ADD_NUM_ARRAY_ITEM(red_arr, red_array[i]);
            }

            // Release the mutex
            xSemaphoreGive(xArrayMutex);
        }

        else
        {
            printf("❌ Failed to take xArrayMutex\n");
            goto cleanup;
        }

        // Print JSON to buffer
        message = cJSON_PrintUnformatted(json); // 0 = unformatted
        if (!message)
        {
            printf("❌ Failed to print JSON\n");
            esp_restart(); // Restart the ESP32
            goto cleanup;
        }

        // Publish and verify MQTT result
        int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC("data"), message, 0, 1, 0);
        if (msg_id < 0)
        {
            printf("❌ MQTT publish failed\n");
        }
        else
        {
            // printf("✅ Published: %s\n", message);
        }

     cleanup:
        if (message)
        {
            // printf("🗑️  Freeing message\n");
            free(message);
            message = NULL;
        }

        if (json)
        {
            // printf("🗑️  Freeing JSON\n");
            cJSON_Delete(json); // Frees all children arrays too
            json = NULL;
        }

#ifdef HEAP_MONITOR_ENABLE
        check_heap_status();
#endif
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
    ESP_ERROR_CHECK(sensor_read_timer_init());
    ESP_ERROR_CHECK(adpd144_init());

    led_init();
    xTimerStart(xLedBlinkTimer, 0); // Start LED blinking

    // // Initialize service_name
    // get_device_service_name(service_name, sizeof(service_name));

#ifdef CONFIG_EXAMPLE_WIFI_PROV_MODE
    wifi_provisioning();
#endif

#ifdef CONFIG_EXAMPLE_WIFI_STAT_MODE
    wifi_init_sta();
#endif

    check_time();

    mqtt_app_start();

    // Create the mutex
    xArrayMutex = xSemaphoreCreateMutex();
    if (xArrayMutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Create the MQTT task
    xTaskCreate(mqtt_publish_task, "MqttTask", 4096, NULL, 5, &xMqttTask);
    if (xMqttTask == NULL)
    {
        ESP_LOGE(TAG, "Failed to create MQTT task");
        return;
    }

    // Create the timer task
    xTaskCreate(timer_read_sensor_task, "ReadSensorTask", 4096, NULL, 5, &xTimerTask);
    if (xTimerTask == NULL)
    {
        ESP_LOGE(TAG, "Failed to create timer task");
        return;
    }
}