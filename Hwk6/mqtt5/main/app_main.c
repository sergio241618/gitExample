#include <time.h>
#include <stdio.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include <stdlib.h> 
#include <time.h>   
#include <limits.h> 
#include <assert.h> 
#include <inttypes.h> 

// --- Wi-Fi Credentials ---
#define WIFI_SSID       "S24 Ultra Sergio"
#define WIFI_PASS       "12345678"

// Set to 0 to use Adafruit IO 
// Set to 1 to use Local Python Logger
#define USE_LOCAL_PYTHON_LOGGER 1
// --- End Switch ---


// --- Broker Configurations ---
#if USE_LOCAL_PYTHON_LOGGER
    // --- Config for Part B (Local Python Logger) ---
    #define MQTT_BROKER_URI "mqtt://10.121.44.169:1883"
    #define MQTT_USERNAME   NULL
    #define MQTT_PASSWORD   NULL
    // --- UPDATED: Added all 4 topics ---
    #define MQTT_TOPIC_TEMP   "u77/sensors/temperature"
    #define MQTT_TOPIC_GPS    "u77/sensors/gps"
    #define MQTT_TOPIC_STATUS "u77/data/status"
    #define MQTT_TOPIC_UPTIME "u77/data/uptime"

#else
    // --- Config for Part C (Adafruit IO) ---
    #define MQTT_BROKER_URI "mqtt://io.adafruit.com:1883"
    #define MQTT_USERNAME   "sergio241618" 
    #define MQTT_PASSWORD   "aio_ZXzM52w7a5E6chR3v3MobGOVTVt0" 
    #define MQTT_TOPIC_TEMP "sergio241618/feeds/u77.temperature"
    #define MQTT_TOPIC_GPS  "sergio241618/feeds/u77.gps"
#endif
// --- End Broker Configurations ---


static const char *TAG = "MQTT_EXAMPLE";

// 13-point heart shape (Guadalajara)
static const float heart_lats[] = {
    20.6700, 20.6725, 20.6750, 20.6775, 20.6800, 20.6790, 20.6780,
    20.6790, 20.6800, 20.6775, 20.6750, 20.6725, 20.6700
};
static const float heart_lons[] = {
    -103.3600, -103.3650, -103.3700, -103.3675, -103.3650, -103.3625, -103.3600,
    -103.3575, -103.3550, -103.3525, -103.3500, -103.3550, -103.3600
};
static int gps_coord_index = 0;
static const int num_coords = sizeof(heart_lats) / sizeof(heart_lats[0]);

// --- Uptime counter ---
static uint32_t g_uptime_seconds = 0;
#define LOOP_DELAY_MS 5000


static void mqtt_publish_task(void *pvParameters)
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameters;
    int msg_id;
    char temp_payload[10];
    char gps_payload[100]; 
    char uptime_payload[12]; // For uint32_t string

    srand(time(NULL));
    
    // --- Send one-time "online" message on connect ---
    #if USE_LOCAL_PYTHON_LOGGER
        msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_STATUS, "online", 0, 1, 0);
        ESP_LOGI(TAG, "Sent one-time status: online");
    #endif

    while (1)
    {
        // --- 1. Publish Temperature ---
        float random_temp = ((float)(rand() % 601) / 10.0) - 20.0;
        snprintf(temp_payload, sizeof(temp_payload), "%.1f", random_temp);

        msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_TEMP, temp_payload, 0, 1, 0);
        ESP_LOGI(TAG, "Sent publish successful, msg_id=%d", msg_id);
        ESP_LOGI(TAG, "  Topic: %s, Payload: %s", MQTT_TOPIC_TEMP, temp_payload);


        // --- 2. Publish GPS (JSON METHOD) ---
        float lat = heart_lats[gps_coord_index];
        float lon = heart_lons[gps_coord_index];
        
        snprintf(gps_payload, sizeof(gps_payload), 
                 "{\"lat\": %.4f, \"lon\": %.4f, \"ele\": 0}", 
                 lat, lon);
        
        msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_GPS, gps_payload, 0, 1, 0);
        ESP_LOGI(TAG, "Sent publish successful, msg_id=%d", msg_id);
        ESP_LOGI(TAG, "  Topic: %s, Payload: %s", MQTT_TOPIC_GPS, gps_payload);


        // --- 3. Update GPS index ---
        gps_coord_index++;
        if (gps_coord_index >= num_coords) {
            gps_coord_index = 0;
        }

        // --- 4. Publish Uptime (ONLY FOR LOCAL LOGGER) ---
        #if USE_LOCAL_PYTHON_LOGGER
            // Update and publish uptime
            g_uptime_seconds += (LOOP_DELAY_MS / 1000);
            snprintf(uptime_payload, sizeof(uptime_payload), "%" PRIu32, g_uptime_seconds);
            msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_UPTIME, uptime_payload, 0, 1, 0);
            ESP_LOGI(TAG, "  Topic: %s, Payload: %s", MQTT_TOPIC_UPTIME, uptime_payload);
        #endif

        // --- 5. Delay ---
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}



static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            // We subscribe to temp just to confirm connection, not required
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_TEMP, 0);
            xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, client, 5, NULL);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("Received data on topic: %.*s\r\n", event->topic_len, event->topic);
            printf("Message: %.*s\r\n", event->data_len, event->data);
            break;

        default:
            ESP_LOGI(TAG, "Other MQTT event id:%d", event->event_id);
            break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
            },
        },
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication = {
                .password = MQTT_PASSWORD,
            },
        },
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying Wi-Fi connection...");

    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi connected!");
        mqtt_app_start();
    }
}

void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &wifi_event_handler,
                                        NULL,
                                        &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler,
                                        NULL,
                                        &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}
void app_main(void)
{
    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated, erase and retry
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi
    wifi_init_sta();
}