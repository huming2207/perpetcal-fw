#include <cstring>
#include <esp_http_client.h>
#include <esp_log.h>
#include "cbr_siri_decoder.hpp"

#include <pugixml.hpp>

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

esp_err_t cbr_siri_decoder::poll_stop_prediction(uint32_t stop_id, uint32_t duration_minutes, uint32_t max_records, uint32_t max_txt_len, uint32_t route)
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
    post_field[sizeof(post_field) - 1] = '\0';

    auto ret = esp_http_client_set_method(client, HTTP_METHOD_POST);
    ret = ret ?: esp_http_client_set_post_field(client, post_field, (int)strlen(post_field));
    ret = ret ?: esp_http_client_set_header(client, "Content-Type", "application/xml"); // Interestingly, Canberra SIRI doesn't care about this too!
    ret = ret ?: esp_http_client_perform(client);

    return ret;
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
            ESP_LOGI(TAG, "Request finished");
            break;
        }

        default: {
            break;
        }
    }

    return ESP_OK;
}

esp_err_t cbr_siri_decoder::decode_result(cbr_siri::record *records_out, size_t record_cnt, uint32_t timeout_ms)
{
    if (records_out == nullptr || record_cnt < 1) {
        return ESP_ERR_INVALID_ARG;
    }

    auto evt_ret = xEventGroupWaitBits(evt_group, cbr_siri::REQUEST_FINISH, pdTRUE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    if (evt_ret == 0) {
        ESP_LOGE(TAG, "Parse timeout");
        return ESP_ERR_TIMEOUT;
    }

    pugi::xml_document doc;
    auto doc_result = doc.load_buffer_inplace(recv_buf, curr_recv_len);
    if (!doc_result) {
        ESP_LOGE(TAG, "Failed to parse SIRI XML, reason: %s", doc_result.description());
        ESP_LOGE(TAG, "Error offset: %d", doc_result.offset);
        return ESP_FAIL;
    }

    auto journeys = doc.select_nodes("//MonitoredVehicleJourney");
    size_t ctr = 0;
    for (auto &journey : journeys) {
        auto *dest = journey.node().child("DestinationName").text().as_string();
        auto *route = journey.node().child("PublishedLineName").text().as_string();
        auto *aimed_arrival = journey.node().child("MonitoredCall").child("AimedArrivalTime").text().as_string();
        auto *expected_arrival = journey.node().child("MonitoredCall").child("ExpectedArrivalTime").text().as_string();

        strncpy(records_out[ctr].dest, dest, 96);
        strncpy(records_out[ctr].route, route, 16);
        parse_siri_iso8601(&records_out[ctr].aimed_arrival, aimed_arrival);
        parse_siri_iso8601(&records_out[ctr].predicted_arrival, expected_arrival);

        ESP_LOGI(TAG, "Dest %s, route %s, aimed arrival %d:%d", records_out->dest, records_out->route, records_out->aimed_arrival.tm_hour, records_out->aimed_arrival.tm_min);

        ctr += 1;
        if (ctr >= record_cnt) {
            break;
        }
    }



    return ESP_OK;
}

esp_err_t cbr_siri_decoder::parse_siri_iso8601(tm *time_out, const char *text, int *_utc_off)
{
    if (time_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0, utc_off = 0;
    if (sscanf(text, "%d-%d-%dT%d:%d:%d+%d:00", &year, &month, &day, &hour, &minute, &second, &utc_off) < 7) {
        return ESP_ERR_INVALID_ARG;
    }

    time_out->tm_year = year - 1900;
    time_out->tm_mon = month - 1;
    time_out->tm_mday = day;
    time_out->tm_hour = hour;
    time_out->tm_min = minute;
    time_out->tm_sec = second;

    if (_utc_off != nullptr) {
        *_utc_off = utc_off;
    }

    return ESP_OK;
}
