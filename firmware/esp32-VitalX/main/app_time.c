/* LwIP SNTP example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "app_main.h"

#include <time.h>

#include <sys/time.h>
#include "esp_attr.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

static const char *TAG = "app_sntp";

static struct timeval tv;

/*--------------------------------------------------------
        Get current time in microseconds
------------------------------------------------------*/
const char* get_timestamp() {
    static char buffer[84];  // Static buffer to store the formatted time
    struct tm timeinfo;

    // Get current time
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &timeinfo);

    // Format: YYYY-MM-DD HH:MM:SS.microseconds
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d.%06ld",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec);

    return buffer;  // Return static buffer
}

static void obtain_time(void)
{
    ESP_LOGI(TAG, "Initializing and starting SNTP");
    /*
     * This is the basic default config with one server and starting the service
     */
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);

    esp_netif_sntp_init(&config);

    ESP_LOGI(TAG, "sntp server: %s", esp_sntp_getservername(0));

    // wait for time to be set
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }

    esp_netif_sntp_deinit();
}

void check_time(void)
{
    gettimeofday(&tv, NULL); // Get current time
    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&tv.tv_sec);
    }

    char strftime_buf[64];

    // Set timezone to Indonesia Central Time (Vietnam)
    setenv("TZ", "ICT-7", 1);
    tzset();
    localtime_r(&tv.tv_sec, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Vietnam is: %s", strftime_buf);
}