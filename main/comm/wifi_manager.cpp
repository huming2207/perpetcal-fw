#include <esp_netif.h>
#include <esp_wifi_default.h>
#include <esp_wifi.h>
#include <cstring>
#include "wifi_manager.hpp"

esp_err_t wifi_manager::init()
{
    wifi_evt = xEventGroupCreate();
    if (wifi_evt == nullptr) {
        ESP_LOGE(TAG, "WiFi event group init fail");
        return ESP_ERR_NO_MEM;
    }

    auto ret = esp_netif_init();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ret = ret ?: esp_wifi_init(&init_config);
    ret = ret ?: esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, this, &wifi_any_id);
    ret = ret ?: esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, this, &sta_got_ip);

    return ret;
}

void wifi_manager::event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    auto *ctx = (wifi_manager *)(arg);
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            esp_wifi_connect();
            ctx->retry_cnt += 1;
            if (ctx->retry_cnt > ctx->max_retry) {
                xEventGroupSetBits(ctx->wifi_evt, wifi::WIFI_FAIL_BIT);
            }
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            auto* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
            ctx->retry_cnt = 0;
            xEventGroupSetBits(ctx->wifi_evt, wifi::WIFI_CONNECT_BIT);
        }
    }
}

esp_err_t wifi_manager::connect_sta(const char *ssid, const char *passwd, uint32_t timeout_ms, uint32_t _max_retry)
{
    if (ssid == nullptr || passwd == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    max_retry = _max_retry;

    wifi_config_t config = {};
    strcpy((char *)config.sta.ssid, ssid);
    strcpy((char *)config.sta.password, passwd);
    config.sta.threshold.authmode = WIFI_AUTH_WEP;

    auto ret = esp_wifi_set_mode(WIFI_MODE_STA);
    ret = ret ?: esp_wifi_set_config(WIFI_IF_STA, &config);
    ret = ret ?: esp_wifi_start();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi");
        return ret;
    }

    auto bits = xEventGroupWaitBits(wifi_evt, (wifi::WIFI_CONNECT_BIT | wifi::WIFI_FAIL_BIT), pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    if (bits & wifi::WIFI_CONNECT_BIT) {
        ESP_LOGI(TAG, "WiFi connected");
    } else if (bits & wifi::WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi fail state set");
        return ESP_FAIL;
    }

    return ret;
}
