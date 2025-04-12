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

#define LED_BLINK_PERIOD_CONNECTED 1000 // ms (TODO: not blink, but constantly on)
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

typedef struct
{
    adc_unit_t adc_unit;                   // ADC unit
    adc_channel_t channel;                 // ADC channel
    adc_oneshot_unit_handle_t unit_handle; // ADC oneshot unit handle
    adc_cali_handle_t cali_handle;         // ADC calibration handle
    int raw_value;                         // Raw ADC reading
    int voltage_value;                     // Converted voltage value
} AdcConfig_t;

/*---------------------------------------------------
                Function Prototypes
-----------------------------------------------------*/
// app_adc.c
esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_cali_handle_t *out_handle);
void adc_calibration_deinit(adc_cali_handle_t handle);
esp_err_t adc_oneshot_init(adc_unit_t unit, adc_channel_t channel, adc_oneshot_unit_handle_t *out_adc_handle, adc_cali_handle_t *out_cali_handle);

// app_mqtt.c
void log_error_if_nonzero(const char *message, int error_code);
void mqtt_app_start(void);
void mqtt_publish_startUpMsg(void);

//app_time.c
void check_time(void);
const char *get_timestamp();

// wifi_prov.c and wifi_stat.c
void wifi_provisioning(void);
void get_device_service_name(char *service_name, size_t max);
extern char service_name[12];
extern SemaphoreHandle_t wifi_prov_semaphore;

void update_led_blink_period(uint32_t period);
        
#endif // APP_MAIN_H