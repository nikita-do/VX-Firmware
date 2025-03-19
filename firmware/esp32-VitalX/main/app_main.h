/**
 * Contain shared structures, macros between source files
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#define CLIENT_ID "VitalX_001"  // Device specific ID
#define MQTT_TOPIC(subtopic) CLIENT_ID "/" subtopic

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

typedef struct {
    adc_unit_t adc_unit;  // ADC unit
    adc_channel_t channel;  // ADC channel
    adc_oneshot_unit_handle_t unit_handle; // ADC oneshot unit handle
    adc_cali_handle_t cali_handle;  // ADC calibration handle
    int raw_value;  // Raw ADC reading
    int voltage_value;  // Converted voltage value
} AdcConfig_t;

/*---------------------------------------------------
                Function Prototypes
-----------------------------------------------------*/
esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_cali_handle_t *out_handle);
void adc_calibration_deinit(adc_cali_handle_t handle);
esp_err_t adc_oneshot_init(adc_unit_t unit, adc_channel_t channel, adc_oneshot_unit_handle_t *out_adc_handle, adc_cali_handle_t *out_cali_handle);


void log_error_if_nonzero(const char *message, int error_code);
void mqtt_app_start(void);

void check_time(void);
const char* get_timestamp();

void wifi_provisioning(void);

#endif // APP_MAIN_H