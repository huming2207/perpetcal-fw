#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_wifi.h>

namespace wifi
{
    enum events : uint32_t {
        WIFI_CONNECT_BIT = BIT(0),
        WIFI_FAIL_BIT = BIT(1),
    };
}

class wifi_manager
{
public:
    static wifi_manager &instance()
    {
        static wifi_manager _instance;
        return _instance;
    }

    wifi_manager operator=(wifi_manager const&) = delete;
    wifi_manager(wifi_manager const&) = delete;

public:
    esp_err_t init();
    esp_err_t connect_sta(const char *ssid, const char *passwd, uint32_t timeout_ms = 20000, uint32_t _max_retry = 5);

private:
    static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

private:
    wifi_manager() = default;
    volatile uint32_t retry_cnt = 0;
    uint32_t max_retry = 5;
    EventGroupHandle_t wifi_evt = nullptr;
    esp_event_handler_instance_t wifi_any_id;
    esp_event_handler_instance_t sta_got_ip;
    static const constexpr char *TAG = "wifi_mgr";
};

