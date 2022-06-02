#include <cstring>
#include <esp_http_client.h>
#include <esp_log.h>
#include "cbr_siri_decoder.hpp"

esp_err_t cbr_siri_decoder::init(const char *_api_key, const char *_url, size_t recv_max_len)
{
    if (_api_key == nullptr || _url == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    api_key = strdup(_api_key);
    url = strdup(_url);

    evt_group = xEventGroupCreate();
    if (evt_group == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    recv_buf = (uint8_t *) heap_caps_calloc(recv_max_len, sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    if (recv_buf == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate recv buffer");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t cbr_siri_decoder::poll_stop_prediction(uint32_t stop_id, uint32_t duration_minutes, uint32_t max_records, uint32_t max_txt_len, uint32_t route, cbr_siri::record *records_out)
{
    esp_http_client_config_t client_config = {};
    client_config.url = url;
    client_config.disable_auto_redirect = false;
    client_config.event_handler = http_evt_handler;
    client_config.user_data = this;
    client_config.user_agent = "PerpetCal/1.0";

    esp_http_client_handle_t client = esp_http_client_init(&client_config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "Failed to create client");
        return ESP_ERR_NO_MEM;
    }

    char post_field[1024] = {};
    snprintf(post_field, sizeof(post_field), cbr_siri::STOP_MONITOR_XML_TEMPLATE, api_key, duration_minutes, stop_id, max_records, max_txt_len, route);

    auto ret = esp_http_client_set_method(client, HTTP_METHOD_POST);
    // ret = ret ?: esp_http_client_set_post_field(client, post_field, );

    return ESP_OK;
}

esp_err_t cbr_siri_decoder::http_evt_handler(esp_http_client_event_t *evt)
{
    if (evt == nullptr || evt->user_data == nullptr) {
        ESP_LOGE(TAG, "Invalid context ptr");
        return ESP_ERR_INVALID_STATE;
    }

    auto *ctx = (cbr_siri_decoder *)evt->user_data;
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR: {
            xEventGroupSetBits(ctx->evt_group, cbr_siri::resp_events::REQ_ERROR);
            break;
        }

        case HTTP_EVENT_ON_DATA: {
            memcpy(ctx->recv_buf + ctx->curr_recv_len, evt->data, evt->data_len);
            ctx->curr_recv_len += evt->data_len;
            if (ctx->curr_recv_len > ctx->recv_len) {
                xEventGroupSetBits(ctx->evt_group, cbr_siri::resp_events::REQ_ERROR);
            } else {
                xEventGroupSetBits(ctx->evt_group, cbr_siri::resp_events::DATA_AVAILABLE);
            }

            break;
        }

        case HTTP_EVENT_ON_FINISH: {
            xEventGroupSetBits(ctx->evt_group, cbr_siri::resp_events::REQUEST_FINISH);
            break;
        }

        default: {
            break;
        }
    }

    return ESP_OK;
}
