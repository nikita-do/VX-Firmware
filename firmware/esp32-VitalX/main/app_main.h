/**
 * @file app_main.h
 * @brief Contain shared libraries, structures, functions, and macros between source files
 * @todo Create header files for each source file
 * 
 * @version 0.2.0
 * @date 2024-04-09
 * 
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#define DEVICE_ID "VX_CEA362" // Device specific ID
#define MQTT_TOPIC(subtopic) "device/" DEVICE_ID "/" subtopic

#define SAMPLE_FREQUENCY 250                    // Hz
#define SAMPLE_BATCH 250 // Number of samples to be sent in one batch

#define LED_BLINK_PERIOD_DISCONNECTED 200 // ms
#define LED_BLINK_PERIOD_PROVISIONING 500 // ms

/* C-Standard headers */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

/* FreeRTOS headers */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

/* ESP32 supported headers */
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

// #include "driver/gpio.h"

// app_adc.c
typedef struct
{
    adc_unit_t adc_unit;                   // ADC unit
    adc_channel_t channel;                 // ADC channel
    adc_oneshot_unit_handle_t unit_handle; // ADC oneshot unit handle
    adc_cali_handle_t cali_handle;         // ADC calibration handle
    int raw_value;                         // Raw ADC reading
    int voltage_value;                     // Converted voltage value
} AdcConfig_t;

esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_cali_handle_t *out_handle);
void adc_calibration_deinit(adc_cali_handle_t handle);
esp_err_t adc_oneshot_init(adc_unit_t unit, adc_channel_t channel, adc_oneshot_unit_handle_t *out_adc_handle, adc_cali_handle_t *out_cali_handle);

// app_mqtt.c
void log_error_if_nonzero(const char *message, int error_code);
void mqtt_app_start(void);

//app_time.c
void check_time(void);
const char *get_timestamp();

// wifi_prov.c and wifi_stat.c
void wifi_provisioning(void);
void update_led_blink_period(uint32_t period);

// circular_buffer.c
#define BUFFER_SIZE 512
#define PROCESS_SIZE 250

#if (BUFFER_SIZE & (BUFFER_SIZE - 1)) != 0
#error "BUFFER_SIZE must be a power of two."
#endif

typedef struct {
    uint32_t buffer[BUFFER_SIZE];
    size_t head;
    size_t tail;
    size_t process_index;
    bool full;
} CircularBuffer_t;

void buffer_init(CircularBuffer_t *cb);
bool buffer_is_empty(CircularBuffer_t *cb);
bool buffer_is_full(CircularBuffer_t *cb);
size_t buffer_data_count(CircularBuffer_t *cb);
size_t buffer_distance(size_t from, size_t to);
bool buffer_put(CircularBuffer_t *cb, uint32_t data);
bool buffer_get_chunk(CircularBuffer_t *cb, uint32_t *temp_buffer);
void buffer_print(CircularBuffer_t *cb);

        
#endif // APP_MAIN_H