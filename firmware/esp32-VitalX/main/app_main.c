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
 * v0.3.0 - 2025-05-30
 *  - Changed batch duration to 3 seconds (1536 samples)
 *  - Restructured CBOR encoding to get data directly from circular buffers
 *  - Changed uint32_t to uint16_t for sensor data to save memory
 *  - Added comments and documentation
 */
#include "app_main.h"
#include "driver/gpio.h"
#include "esp_app_desc.h"

#include "adpd144.h"
#include "cbor.h"
#include "mqtt_client.h"
#include "esp_timer.h"

#include "esp_wifi.h"

/**************************************************************************************************
 *                                      Macro Definition
 **************************************************************************************************/
#define TIMER_PERIOD_TICKS (configTICK_RATE_HZ / SAMPLING_RATE) // Calculate ticks for 512 Hz
#define LED_GPIO GPIO_NUM_1                                     // GPIO1 for LED

#define CBOR_BUFFER_SIZE (10000) // Size of the CBOR buffer

// #define HEAP_MONITOR_ENABLE // Uncomment to enable heap monitoring

/**************************************************************************************************
 *                                     Global declaration
 **************************************************************************************************/
static const char *TAG = "app_main";
static uint32_t led_blink_period = LED_BLINK_PERIOD_DISCONNECTED; // initial LED status: disconnected
static uint64_t batchStartTime_ms = 0;                            // Start time for reading sensors
static uint16_t sampleCount = 0;
static CircularBuffer_t gsrBuffer, ecgBuffer, irBuffer, redBuffer;
static uint16_t gsrTempBuffer[N_SAMPLE];
static uint16_t ecgTempBuffer[N_SAMPLE];
static uint16_t irTempBuffer[N_SAMPLE];
static uint16_t redTempBuffer[N_SAMPLE];
static uint8_t cborBuffer[CBOR_BUFFER_SIZE];
static uint32_t packet_id = 0; // Packet ID for CBOR messages

// Task handles
static TaskHandle_t xTimerTask = NULL;
static TaskHandle_t xMqttTask = NULL;

// Timer handles
static TimerHandle_t xLedBlinkTimer = NULL;
static TimerHandle_t xSensorReadTimer = NULL;

// ADC configuration structures
AdcConfig_t gsr = {
    .adc_unit = ADC_UNIT_1,
    .channel = ADC_CHANNEL_6,
    .unit_handle = NULL,
    .cali_handle = NULL,
    .raw_value = 0,
    .voltage_value = 0};

AdcConfig_t ecg = {
    .adc_unit = ADC_UNIT_2,
    .channel = ADC_CHANNEL_0,
    .unit_handle = NULL,
    .cali_handle = NULL,
    .raw_value = 0,
    .voltage_value = 0};

// MQTT topics
char mqtt_topics_firmware_version[64];
char mqtt_topics_sampling_rate[64];
char mqtt_topics_sample_batch[64];
char mqtt_topics_status_online[64];
char mqtt_topics_commands[64];
char mqtt_topics_commands_start[64];
char mqtt_topics_commands_reset[64];
char mqtt_topics_responses_start[64];
char mqtt_topics_responses_reset[64];
char mqtt_topics_data[64];
char mqtt_topics_rssi[64];

// Extern varialbes
extern esp_mqtt_client_handle_t client;

/**************************************************************************************************
 *                                  Helper Functions
 **************************************************************************************************/

// Encode the entire message as a CBOR map with a timestamp, packet_id, and 4 sensor data arrays.
esp_err_t encode_cbor_message(uint8_t *buffer, uint64_t timestamp, size_t buffer_size, size_t *encoded_length)
{
    CborEncoder encoder;
    CborEncoder map_encoder;

    cbor_encoder_init(&encoder, buffer, buffer_size, 0);

    // Start CBOR map with 6 key-value pairs (added packet_id)
    cbor_encoder_create_map(&encoder, &map_encoder, 6);

    // "id": packet_id
    cbor_encode_text_stringz(&map_encoder, "id");
    cbor_encode_uint(&map_encoder, packet_id);

    // "t": timestamp
    cbor_encode_text_stringz(&map_encoder, "t");
    cbor_encode_uint(&map_encoder, timestamp);

    // Encode temp buffers instead of circular buffers
    cbor_encode_text_stringz(&map_encoder, "ecg");
    CborEncoder ecg_array_encoder;
    cbor_encoder_create_array(&map_encoder, &ecg_array_encoder, N_SAMPLE);
    for (size_t i = 0; i < N_SAMPLE; ++i)
    {
        cbor_encode_uint(&ecg_array_encoder, ecgTempBuffer[i]);
    }
    cbor_encoder_close_container(&map_encoder, &ecg_array_encoder);

    cbor_encode_text_stringz(&map_encoder, "gsr");
    CborEncoder gsr_array_encoder;
    cbor_encoder_create_array(&map_encoder, &gsr_array_encoder, N_SAMPLE);
    for (size_t i = 0; i < N_SAMPLE; ++i)
    {
        cbor_encode_uint(&gsr_array_encoder, gsrTempBuffer[i]);
    }
    cbor_encoder_close_container(&map_encoder, &gsr_array_encoder);

    cbor_encode_text_stringz(&map_encoder, "ir");
    CborEncoder ir_array_encoder;
    cbor_encoder_create_array(&map_encoder, &ir_array_encoder, N_SAMPLE);
    for (size_t i = 0; i < N_SAMPLE; ++i)
    {
        cbor_encode_uint(&ir_array_encoder, irTempBuffer[i]);
    }
    cbor_encoder_close_container(&map_encoder, &ir_array_encoder);

    cbor_encode_text_stringz(&map_encoder, "red");
    CborEncoder red_array_encoder;
    cbor_encoder_create_array(&map_encoder, &red_array_encoder, N_SAMPLE);
    for (size_t i = 0; i < N_SAMPLE; ++i)
    {
        cbor_encode_uint(&red_array_encoder, redTempBuffer[i]);
    }
    cbor_encoder_close_container(&map_encoder, &red_array_encoder);

    cbor_encoder_close_container(&encoder, &map_encoder);

    *encoded_length = cbor_encoder_get_buffer_size(&encoder, buffer);
    return ESP_OK;
}

/*------------------------------ Timer functions -----------------------------*/
/**
 * @brief Timer callback function for sensor reading.
 *
 * This function is called by the FreeRTOS timer to notify the sensor reading task
 * that it is time to read sensor data. It uses `vTaskNotifyGiveFromISR` to notify
 * timer_read_sensor_task, which will then read the sensor data and process it.
 *
 * @param xTimer The timer that triggered this callback.
 */
static void prvSensorReadTimerCallback(TimerHandle_t xTimer)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the timer task
    if (xTimerTask != NULL)
    {
        // Notify Sensor Task with TIMER_TICK bit
        vTaskNotifyGiveFromISR(xTimerTask, &xHigherPriorityTaskWoken);
        // Check if the task was woken by the notification
        if (xHigherPriorityTaskWoken == pdTRUE)
        {
            // If a higher priority task was woken, perform a context switch
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief Initializes the sensor read timer.
 *
 * This function creates a FreeRTOS timer that triggers at a frequency
 * defined by the SAMPLING_RATE. The timer callback function is set to
 * `prvSensorReadTimerCallback`, which will notify the sensor reading task
 * to read sensor data.
 *
 * @return ESP_OK on success, or ESP_FAIL if the timer creation fails.
 */
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
/**
 * @brief Callback function for the LED blink timer.
 *
 * This function toggles the LED state each time the timer expires.
 * It uses a static variable to keep track of the current LED state.
 *
 * @param xTimer The timer that triggered this callback.
 */
static void prvLedBlinkTimerCallback(TimerHandle_t xTimer)
{
    static bool led_state = false;
    gpio_set_level(LED_GPIO, led_state);
    led_state = !led_state;
}

/**
 * @brief Initializes the LED GPIO and creates a timer for blinking.
 *
 * This function configures the GPIO pin for the LED as an output and creates
 * a FreeRTOS timer that toggles the LED state at a frequency defined by
 * `led_blink_period`. The timer callback function is set to `prvLedBlinkTimerCallback`.
 */
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
            xTimerStop(xLedBlinkTimer, 0);
            gpio_set_level(LED_GPIO, 1);
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
#ifdef HEAP_MONITOR_ENABLE
#include "esp_heap_caps.h"

/**
 * @brief Checks the heap status and prints relevant information.
 *
 * This function retrieves the current free heap size, minimum ever free heap size,
 * and largest free block size. It also checks the number of free blocks and whether
 * there are any free blocks smaller than 32 bytes. The information is printed to the console.
 */
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
#endif // HEAP_MONITOR_ENABLE

/**************************************************************************************************
 *                                     MQTT functions
 **************************************************************************************************/
// Initialize MQTT topics based on the service name
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
    snprintf(mqtt_topics_rssi, sizeof(mqtt_topics_rssi), "device/%s/rssi", service_name);
}

/**
 * @brief MQTT event handler function.
 *
 * This function handles various MQTT events such as connection, disconnection,
 * data reception, and errors. It performs actions like publishing device information,
 * subscribing to command topics, and handling commands received from the MQTT broker.
 *
 */
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
        esp_mqtt_client_publish(client, "device/", service_name, 0, 1, 1);
        esp_mqtt_client_publish(client, mqtt_topics_status_online, "true", 0, 1, 1);

        const esp_app_desc_t *app_desc = esp_app_get_description();
        esp_mqtt_client_publish(client, mqtt_topics_firmware_version, app_desc->version, 0, 1, 1);

        char sampling_rate_str[10];
        sprintf(sampling_rate_str, "%d", SAMPLING_RATE);
        esp_mqtt_client_publish(client, mqtt_topics_sampling_rate, sampling_rate_str, 0, 1, 1);

        char sample_batch[10];
        sprintf(sample_batch, "%d", N_SAMPLE);
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
                batchStartTime_ms = get_timestamp(); // Get the current timestamp
                xTimerStart(xSensorReadTimer, 0);
            }
            else if (strncmp(event->data, "false", event->data_len) == 0)
            {
                esp_mqtt_client_publish(client, mqtt_topics_responses_start, "false", 0, 1, 1);
                ESP_LOGI(TAG, "Received 'stop' command, stopping data publishing...");
                xTimerStop(xSensorReadTimer, 0);
                sampleCount = 0;
                adpd144_stop();
                packet_id = 0; // Reset packet_id on stop command
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
    case MQTT_EVENT_DELETED:
        ESP_LOGI(TAG, "MQTT message deleted from outbox (msg_id: %d)", event->msg_id);
        break;
    default:
        // ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/**************************************************************************************************
 *	                                    Task functions
 **************************************************************************************************/
void rssi_publish_task(void *pvParameters)
{
    wifi_ap_record_t ap_info;

    while (1)
    {
        esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);

        if (ret == ESP_OK)
        {
            char msg[64];
            snprintf(msg, sizeof(msg), "{ \"ssid\": \"%s\", \"rssi\": %d }", ap_info.ssid, ap_info.rssi);
            esp_mqtt_client_publish(client, mqtt_topics_rssi, msg, 0, 1, 0);
            ESP_LOGI(TAG, "Published RSSI to MQTT: %s", msg);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get AP info: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(60 * 1000));
    }
}
/**
 * @brief Task to read sensor data at regular intervals using a FreeRTOS timer.
 *
 * This task waits for the timer to notify it, reads the GSR and ECG values in voltage using ADC,
 * and reads the PPG IR and Red values using the ADPD144 sensor. The read values are
 * stored in circular buffers for later processing.
 */
void timer_read_sensor_task(void *arg)
{
    uint32_t ir_value_32 = 0;
    uint32_t red_value_32 = 0;

    while (1)
    {
        // Wait for the timer to notify
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //-------------ADC1 Oneshot Read---------------//
        adc_oneshot_read(gsr.unit_handle, gsr.channel, &gsr.raw_value);
        adc_cali_raw_to_voltage(gsr.cali_handle, gsr.raw_value, &gsr.voltage_value);

        //-------------ADC2 Oneshot Read---------------//
        adc_oneshot_read(ecg.unit_handle, ecg.channel, &ecg.raw_value);
        adc_cali_raw_to_voltage(ecg.cali_handle, ecg.raw_value, &ecg.voltage_value);

        //-------------Read PPG Values---------------//
        adpd144_readIRValue(&ir_value_32, 1);
        adpd144_readRedValue(&red_value_32, 1);

        uint16_t ir_value = (uint16_t)(ir_value_32 & 0xFFFF);   // Convert to 16-bit
        uint16_t red_value = (uint16_t)(red_value_32 & 0xFFFF); // Convert to 16-bit
        uint16_t ecg_value = (uint16_t)(ecg.voltage_value);
        uint16_t gsr_value = (uint16_t)(gsr.voltage_value);

        // Store the values in circular buffers
        buffer_put(&gsrBuffer, gsr_value); // Store GSR value
        buffer_put(&ecgBuffer, ecg_value); // Store ECG value
        buffer_put(&irBuffer, ir_value);   // Store IR value
        buffer_put(&redBuffer, red_value); // Store Red value

        sampleCount++; // Increment sample count

        if (sampleCount >= N_SAMPLE)
        {
            // Notify the MQTT task to publish data
            xTaskNotifyGive(xMqttTask);
            sampleCount = 0; // Reset sample count
        }
    }
}

/**
 * @brief Task to publish sensor data to the MQTT broker.
 */
void mqtt_publish_task(void *arg)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#ifdef HEAP_MONITOR_ENABLE
        check_heap_status();
#endif

        size_t encoded_size = 0;
        batchStartTime_ms += (N_SAMPLE / SAMPLING_RATE) * 1000; // Update start time for the next batch
        buffer_get_chunk(&gsrBuffer, gsrTempBuffer, N_SAMPLE);
        buffer_get_chunk(&ecgBuffer, ecgTempBuffer, N_SAMPLE);
        buffer_get_chunk(&irBuffer, irTempBuffer, N_SAMPLE);
        buffer_get_chunk(&redBuffer, redTempBuffer, N_SAMPLE);
        encode_cbor_message(cborBuffer, batchStartTime_ms, CBOR_BUFFER_SIZE, &encoded_size);

        // Publish the data to MQTT broker
        esp_err_t err = esp_mqtt_client_publish(client, mqtt_topics_data, (const char *)cborBuffer, encoded_size, 0, 0);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to publish data: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "Data published successfully, Encoded size: %zu bytes", encoded_size);
        }
        packet_id++; // Increment packet_id

#ifdef HEAP_MONITOR_ENABLE
        check_heap_status();
#endif
    }
}

/**
 * @brief Main application entry point.
 */
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

    // Create the MQTT task
    xTaskCreate(mqtt_publish_task, "MqttTask", 2048, NULL, 5, &xMqttTask);
    if (xMqttTask == NULL)
    {
        ESP_LOGE(TAG, "Failed to create MQTT task");
        return;
    }

    // Create the timer task
    xTaskCreate(timer_read_sensor_task, "ReadSensorTask", 2048, NULL, 6, &xTimerTask);
    if (xTimerTask == NULL)
    {
        ESP_LOGE(TAG, "Failed to create timer task");
        return;
    }

    xTaskCreate(&rssi_publish_task, "rssi_publish_task", 4096, NULL, 5, NULL);
}