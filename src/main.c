#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include <driver/gpio.h>

#include "ESP32_GPIO.h"
#include <dht.h>

/* Adafruit Details */
#define BROKER_URL "mqtts://io.adafruit.com"
#define IO_TOPIC "XXXXX"
#define AIO_USERNAME    "XXXXXX"
#define AIO_KEY         "XXXXXX"

#define EXAMPLE_ESP_WIFI_SSID      "XXXXXXX"
#define EXAMPLE_ESP_WIFI_PASS      "XXXXXXX"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* DHT11 Defines */
#define DHT11_PIN               17
#define BUZZER_PIN              18
#define BUZZER_PIN_SEL          (1ULL << BUZZER_PIN)
#define HUMIDITY_THRESHOLD      800
#define TEMPERATURE_THRESHOLD   150

/* User PV Functions */
static void init_hw(void);
static void check_alarm(void);

/* User PV Task Functions */
static void beep(void *args);
static void dht11Task(void *args);

/* User PV Variables */
static int16_t temperature;
static int16_t humidity;
static bool temperature_alarm = false;
static bool humidity_alarm = false;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static const char *TAG = "MQTTS_EXAMPLE";

/* io.adafruit.com root CA */
static const uint8_t s_adafruitio_root_ca[] = \
    "-----BEGIN CERTIFICATE-----\n" \
    "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
    "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
    "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
    "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
    "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
    "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
    "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
    "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
    "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
    "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
    "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
    "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
    "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
    "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
    "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
    "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
    "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
    "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
    "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
    "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
    "-----END CERTIFICATE-----\n";

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void){
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

esp_mqtt_client_handle_t mqtt_client;
int mqtt_connected = 0;

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event){
    esp_mqtt_client_handle_t client = event->client;
    mqtt_client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, IO_TOPIC, "11", 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            mqtt_connected = 1;

            // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
            // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
            // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = 0;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);

            } else {
                ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
                break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}


static void mqtt_app_start(void){
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER_URL,
        .cert_pem = (const char*)s_adafruitio_root_ca,
        .port = 8883,
        .event_handle = mqtt_event_handler,
        .username = AIO_USERNAME,
        .password = AIO_KEY
        // .user_context = (void *)your_context
    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);

    ESP_LOGI(TAG, "[APP] Starting mqtt client");

    esp_mqtt_client_start(client);
}


void app_main(){

    ESP_LOGI(TAG, "[APP] Startup..");
    
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    // esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    // esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    // esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    // esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    // esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    mqtt_app_start();

    init_hw();

    int app_cpu = xPortGetCoreID();
    ESP_LOGI(TAG, "app_cpu is %d (%s core)\n", app_cpu, app_cpu > 0 ? "Dual" : "Single");

    xTaskCreatePinnedToCore(dht11Task, "dht11Task", 2048, NULL, 5, 0, app_cpu);
}

/* User PV Functions */
static void init_hw(void){
    ESP32_GPIO_init_output(BUZZER_PIN);
}

static void check_alarm(void){
    bool is_alarm = temperature >= TEMPERATURE_THRESHOLD;
    bool run_beep = is_alarm && !temperature_alarm;
    temperature_alarm = is_alarm;

    if(run_beep){
        xTaskCreate(beep, "beep", configMINIMAL_STACK_SIZE, (void*)3, 5, NULL);
        temperature_alarm = false;
        return;
    }

    is_alarm = humidity >= HUMIDITY_THRESHOLD;
    run_beep = is_alarm && !humidity_alarm;
    humidity_alarm = is_alarm;

    if(run_beep){
        xTaskCreate(beep, "beep", configMINIMAL_STACK_SIZE, (void*)2, 5, NULL);
        humidity_alarm = false;
        return;
    }

}

/* User PV Task Functions */
static void beep(void *args){

    int cnt = 2 * (int)args;
    bool state = true;

    for(int i = 0; i < cnt; ++i, state = !state){
        gpio_set_level(BUZZER_PIN, state);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
    
}

static void dht11Task(void *args){

    char str[32] = {0};
    int msg_id;

    while (1) {

        if(dht_read_data(DHT_TYPE_DHT11, (gpio_num_t)DHT11_PIN, &humidity, &temperature) == ESP_OK){
            
            check_alarm();
            if(mqtt_connected){
                sprintf(str, "%0.2f",temperature/10.0f);
                msg_id = esp_mqtt_client_publish(mqtt_client, IO_TOPIC, str, 0, 1, 0);
                // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                ESP_LOGI(TAG, "temperature=%d, humidity=%d", temperature, humidity);
            }
        }
        else{
            ESP_LOGI(TAG, "could not read data from dht11 sensor");
        }

        /**
         * The sampling rate of DHT11 should not exceed 1 Hz (that is, one reading per
         * second). Otherwise, the sensor gets hot and the readings will be inaccurate.
         * 
         */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}