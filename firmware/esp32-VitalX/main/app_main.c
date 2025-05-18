/**
 * @file app_main.c
 * @brief Main application file
 *
 * The application reads the GSR, ECG, PPG IR, and PPG Red values from the sensors and publishes them to the MQTT broker. The application also subscribes to the MQTT broker for the commands to start and stop the data publishing.
 *
 * ---------------------- Change Log ----------------------
 * v0.1.0 - 2025-03-20
 *  - Integrated wifi configuration manager, MQTT client, and sensor reading tasks.
 *  - Added GSR, ECG, PPG IR, and PPG Red sensor reading tasks with sampling rate of 50 Hz
 * v0.1.1 - 2025-04-03
 *  - Sent data in batches of 100 samples to the MQTT broker.
 *  - Increased sampling rate to 100 Hz.
 *  - ADPD144 component updated: read from PD4 instead of PD3 for better ambient light cancelation
 *  - Improved code readability and organization.
 * v0.2.0 - 2025-04-09
 *  - Updated firmware version override via CMake
 *  - Modified the MQTT topic structure to include device ID and firmware version.
 *  - Added LED indicator for connection status
 * v0.2.1 - 2025-04-16
 *  - Increased sampling rate to 250 Hz.
 *  - Added heap monitoring feature to track memory usage.
 *  - Added TinyCBOR component: Encoded data as CBOR before sending to MQTT broker.
 *  - ADPD144 component updated: Sampling rate = 250 Hz
 *  - Improved LED indicator
 * v0.2.2 - 2025-05-04
 *  - Added circular buffers to store sensor data
 *  - Changed Freertos frequency to 512Hz
 *  - Increased ADC and ADPD144 sampling rate to 512Hz, with 512 samples per batch
 *  - Added command to clear NVS flash
 *  - Assigned specific DEVICE_ID to different devices
 */
#include "app_main.h"
#include "driver/gpio.h"
#include "esp_app_desc.h"

#include "adpd144.h"
#include "cbor.h"
#include "mqtt_client.h"
#include "esp_timer.h"

#ifdef HEAP_MONITOR_ENABLE
#include "esp_heap_caps.h"
#endif

/**************************************************************************************************
 *                                      Macro Definition
 **************************************************************************************************/
#define LED_GPIO GPIO_NUM_2 // GPIO2 for LED

// #define HEAP_MONITOR_ENABLE  // Uncomment to enable heap monitoring

#define TIMER_PERIOD_TICKS (configTICK_RATE_HZ / SAMPLE_FREQUENCY_HZ) // Calculate ticks for 512 Hz

/**************************************************************************************************
 *                                     Global declaration
 **************************************************************************************************/
static const char *TAG = "app_main";

static portMUX_TYPE buffer_mux = portMUX_INITIALIZER_UNLOCKED;

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

static CircularBuffer_t ecg_buffer, gsr_buffer, ir_buffer, red_buffer, time_buffer;

uint8_t cbor_buffer[10000] = {0}; // Buffer for CBOR encoding

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
static int64_t elapsed_time_us = 0;
static uint16_t sample_count = 0;

static char mqtt_topics_firmware_version[64];
static char mqtt_topics_sampling_rate[64];
static char mqtt_topics_sample_batch[64];
static char mqtt_topics_status_online[64];
static char mqtt_topics_commands[64];
static char mqtt_topics_commands_start[64];
static char mqtt_topics_commands_reset[64];
static char mqtt_topics_responses_start[64];
static char mqtt_topics_responses_reset[64];
static char mqtt_topics_data[64];

/**************************************************************************************************
 *                                  Helper Functions
 **************************************************************************************************/
/*------------------------------- CBOR function ----------------------------*/
static void encode_int_array(CborEncoder *parent, const char *key, uint32_t *array, size_t len)
{
    cbor_encode_text_stringz(parent, key);

    CborEncoder arrEnc;
    cbor_encoder_create_array(parent, &arrEnc, len);
    for (size_t i = 0; i < len; i++)
    {
        cbor_encode_int(&arrEnc, array[i]);
    }
    cbor_encoder_close_container(parent, &arrEnc);
}
/*------------------------------ Timer functions -----------------------------*/
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
    // Create the timer with the calculated period
    xSensorReadTimer = xTimerCreate("SensorReadTimer", TIMER_PERIOD_TICKS, pdTRUE, 0, prvSensorReadTimerCallback);
    if (xSensorReadTimer == NULL)
    {
        ESP_LOGE(TAG, "Timer Create Failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* -------------------- LED functions ----------------------------*/
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

/**
 * @brief Updates the LED blink period based on the provided value.
 *
 * This function adjusts the LED blink behavior depending on the specified period:
 * - If the period is 0, the LED is turned off.
 * - If the period is `portMAX_DELAY`, the LED remains solid (always on).
 * - Otherwise, the LED blinks with the specified period.
 *
 * @param period The desired blink period in milliseconds. Use 0 to turn off the LED,
 *               or `portMAX_DELAY` to keep the LED solid.
 */
void update_led_blink_period(uint32_t period)
{
    led_blink_period = period;
    if (xLedBlinkTimer != NULL)
    {
        if (period == 0)
        {
            // Turn off the LED
            gpio_set_level(LED_GPIO, 0);
            xTimerStop(xLedBlinkTimer, 0);
        }
        else if (period == portMAX_DELAY)
        {
            // Make the LED solid (always on)
            gpio_set_level(LED_GPIO, 1);
            xTimerStop(xLedBlinkTimer, 0);
        }
        else
        {
            // Update the blink period
            xTimerChangePeriod(xLedBlinkTimer, pdMS_TO_TICKS(led_blink_period), 0);
            xTimerStart(xLedBlinkTimer, 0);
        }
    }
}

/* -------------------- Heap statistics ----------------------------*/
void check_heap_status()
{
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);

    printf("🔍 Heap Status:\n");
    printf("  🧠 Current Free Heap:        %d bytes\n", free_heap);
    printf("  📉 Minimum Ever Free Heap:   %d bytes\n", min_free_heap);
    printf("  🔎 Largest Free Block:       %d bytes\n", heap_info.largest_free_block);
    printf("  🧩 Total Free Blocks:        %d\n", heap_info.free_blocks);
    printf("  ⚠️  Free Blocks < 32 Bytes:   %d\n", heap_info.total_blocks < 32);
}

/**************************************************************************************************
 *                                     MQTT Callback functions
 **************************************************************************************************/
void initialize_mqtt_topics(const char *service_name)
{
    snprintf(mqtt_topics_firmware_version, sizeof(mqtt_topics_firmware_version), "device/%s/attributes/firmware_version", service_name);
    snprintf(mqtt_topics_sampling_rate, sizeof(mqtt_topics_sampling_rate), "device/%s/attributes/sampling_rate", service_name);
    snprintf(mqtt_topics_sample_batch, sizeof(mqtt_topics_sample_batch), "device/%s/attributes/sample_batch", service_name);
    snprintf(mqtt_topics_status_online, sizeof(mqtt_topics_status_online), "device/%s/status_online", service_name);
    snprintf(mqtt_topics_commands, sizeof(mqtt_topics_commands), "device/%s/commands/#", service_name);
    snprintf(mqtt_topics_commands_start, sizeof(mqtt_topics_commands_start), "device/%s/commands/start", service_name);
    snprintf(mqtt_topics_commands_reset, sizeof(mqtt_topics_commands_reset), "device/%s/commands/reset", service_name);
    snprintf(mqtt_topics_responses_start, sizeof(mqtt_topics_responses_start), "device/%s/responses/start", service_name);
    snprintf(mqtt_topics_responses_reset, sizeof(mqtt_topics_responses_reset), "device/%s/responses/reset", service_name);
    snprintf(mqtt_topics_data, sizeof(mqtt_topics_data), "device/%s/data", service_name);
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to HiveMQ broker!");
        initialize_mqtt_topics(service_name);

        ESP_LOGI(TAG, "Device name: %s", service_name);
        esp_mqtt_client_publish(client, "device", service_name, 0, 1, 1);
        esp_mqtt_client_publish(client, mqtt_topics_status_online, "true", 0, 1, 1);

        const esp_app_desc_t *app_desc = esp_app_get_description();
        esp_mqtt_client_publish(client, mqtt_topics_firmware_version, app_desc->version, 0, 1, 1);

        char sampling_rate_str[10];
        sprintf(sampling_rate_str, "%d", SAMPLE_FREQUENCY_HZ);
        esp_mqtt_client_publish(client, mqtt_topics_sampling_rate, sampling_rate_str, 0, 1, 1);

        char sample_batch[10];
        sprintf(sample_batch, "%d", SAMPLE_BATCH);
        esp_mqtt_client_publish(client, mqtt_topics_sample_batch, sample_batch, 0, 1, 1);

        esp_mqtt_client_subscribe(client, mqtt_topics_commands, 1);
        ESP_LOGI(TAG, "Subscribed to topic: %s", mqtt_topics_commands);

        update_led_blink_period(portMAX_DELAY);
        break;

    case MQTT_EVENT_DISCONNECTED:
        static int disconnected_time = 0;
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        disconnected_time++;
        if (disconnected_time >= 3)
        {
            ESP_LOGW(TAG, "Disconnected for too long. Restarting ESP...");
            ESP_ERROR_CHECK(nvs_flash_erase());
            esp_restart();
        }
        update_led_blink_period(LED_BLINK_PERIOD_DISCONNECTED);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Received MQTT message on topic: %.*s, data: %.*s", event->topic_len, event->topic, event->data_len, event->data);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to display the log

        if (strncmp(event->topic, mqtt_topics_commands_start, event->topic_len) == 0)
        {
            if (strncmp(event->data, "true", event->data_len) == 0)
            {
                esp_mqtt_client_publish(client, mqtt_topics_responses_start, "true", 0, 1, 1);
                ESP_LOGI(TAG, "Received 'start' command, publishing data...");
                adpd144_start();
                read_start_time = esp_timer_get_time();
                xTimerStart(xSensorReadTimer, 0);
            }
            else if (strncmp(event->data, "false", event->data_len) == 0)
            {
                esp_mqtt_client_publish(client, mqtt_topics_responses_start, "false", 0, 1, 1);
                ESP_LOGI(TAG, "Received 'stop' command, stopping data publishing...");
                sample_count = 0;
                xTimerStop(xSensorReadTimer, 0);
                adpd144_stop();
                time_array[0] = gsr_array[0] = ecg_array[0] = ir_array[0] = red_array[0] = 0;
            }
        }

        else if (strncmp(event->topic, mqtt_topics_commands_reset, event->topic_len) == 0)
        {
            if (strncmp(event->data, "true", event->data_len) == 0)
            {
                esp_mqtt_client_publish(client, mqtt_topics_responses_reset, "true", 0, 1, 1);
                ESP_LOGW(TAG, "Restarting ESP...");
                ESP_ERROR_CHECK(nvs_flash_erase());
                esp_restart();
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
    uint32_t ir_value = 0;
    uint32_t red_value = 0;

    while (1)
    {
        // Wait for the timer to notify
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Calculate elapsed time
        elapsed_time_us = (esp_timer_get_time() - read_start_time); // in microseconds
        //-------------ADC1 Oneshot Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_read(gsr.unit_handle, gsr.channel, &gsr.raw_value));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(gsr.cali_handle, gsr.raw_value, &gsr.voltage_value));

        //-------------ADC2 Oneshot Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_read(ecg.unit_handle, ecg.channel, &ecg.raw_value));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(ecg.cali_handle, ecg.raw_value, &ecg.voltage_value));

        //-------------Read PPG Values---------------//
        ESP_ERROR_CHECK(adpd144_readIRValue(&ir_value, 1));
        ESP_ERROR_CHECK(adpd144_readRedValue(&red_value, 1));

        buffer_put(&ecg_buffer, ecg.voltage_value);
        buffer_put(&gsr_buffer, gsr.voltage_value);
        buffer_put(&ir_buffer, ir_value);
        buffer_put(&red_buffer, red_value);
        buffer_put(&time_buffer, elapsed_time_us);

        sample_count++; // Increment sample count

        if (sample_count >= SAMPLE_BATCH)
        {
            sample_count = 0; // Reset sample count
            // Notify the MQTT task to publish data
            xTaskNotifyGive(xMqttTask);
        }
    }
}

void mqtt_publish_task(void *arg)
{
    CborEncoder encoder, mapEncoder;
    bool data_ready = false;
    int msg_id = 0;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Enter critical section to ensure thread safety
        taskENTER_CRITICAL(&buffer_mux);

        // Retrieve data from buffers
        data_ready = buffer_get_chunk(&ecg_buffer, ecg_array) && buffer_get_chunk(&gsr_buffer, gsr_array) &&
                     buffer_get_chunk(&ir_buffer, ir_array) && buffer_get_chunk(&red_buffer, red_array) &&
                     buffer_get_chunk(&time_buffer, time_array);

        // Exit critical section
        taskEXIT_CRITICAL(&buffer_mux);

        if (!data_ready)
        {
            ESP_LOGW(TAG, "Buffer is empty or not enough data to publish");
            continue;
        }

#ifdef HEAP_MONITOR_ENABLE
        check_heap_status();
#endif

        // Initialize CBOR encoder
        cbor_encoder_init(&encoder, cbor_buffer, sizeof(cbor_buffer), 0);
        cbor_encoder_create_map(&encoder, &mapEncoder, 5);

        // Encode arrays into the map
        encode_int_array(&mapEncoder, "time", time_array, SAMPLE_BATCH);
        encode_int_array(&mapEncoder, "gsr", gsr_array, SAMPLE_BATCH);
        encode_int_array(&mapEncoder, "ecg", ecg_array, SAMPLE_BATCH);
        encode_int_array(&mapEncoder, "ir", ir_array, SAMPLE_BATCH);
        encode_int_array(&mapEncoder, "red", red_array, SAMPLE_BATCH);

        // Close the CBOR map
        cbor_encoder_close_container(&encoder, &mapEncoder);

        // Publish the encoded data to MQTT
        size_t encoded_len = cbor_encoder_get_buffer_size(&encoder, cbor_buffer);

        msg_id = esp_mqtt_client_publish(client, mqtt_topics_data, (const char *)cbor_buffer, encoded_len, 0, 0);

        if (msg_id < 0)
        {
            ESP_LOGE(TAG, "Failed to publish MQTT message");
        }
        else
        {
            ESP_LOGI(TAG, "Published CBOR data to MQTT (msg_id: %d, size: %zu bytes)", msg_id, encoded_len);
        }

#ifdef HEAP_MONITOR_ENABLE
        check_heap_status();
#endif
    }
}

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

    wifi_provisioning();

    check_time();

    mqtt_app_start();

    // Initialize the circular buffers
    buffer_init(&ecg_buffer, "ECG Buffer");
    buffer_init(&gsr_buffer, "GSR Buffer");
    buffer_init(&ir_buffer, "IR Buffer");
    buffer_init(&red_buffer, "Red Buffer");
    buffer_init(&time_buffer, "Time Buffer");

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