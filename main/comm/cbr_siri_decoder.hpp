#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <ctime>
#include <esp_http_client.h>
#include <esp_err.h>

namespace cbr_siri
{
    struct record
    {
        char route[16];
        tm aimed_arrival; // The time on timetable
        tm predicted_arrival; // The time predicted by the server
        char dest[96];
    };

    typedef void(*resp_fn)(record *, size_t);

    enum resp_events : uint32_t {
        DATA_AVAILABLE = BIT(0),
        REQUEST_FINISH = BIT(1),
        REQ_ERROR = BIT(2),
    };

    static const constexpr char STOP_MONITOR_XML_TEMPLATE[] =
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
            "<Siri xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" version=\"2.0\" xmlns=\"http://www.siri.org.uk/siri\">"
            "<ServiceRequest>"
            "<RequestTimestamp>1970-01-01T00:00:00</RequestTimestamp>" // For Canberra SIRI, any valid timestamp string is fine here
            "<RequestorRef>%s</RequestorRef>" // First arg: API key, must be string
            "<StopMonitoringRequest version=\"2.0\">"
            "<RequestTimestamp>1970-01-01T00:00:00</RequestTimestamp>"  // For Canberra SIRI, any valid timestamp string is fine here
            "<PreviewInterval>PT%uM</PreviewInterval>" // Second arg: prediction duration in minutes
            "<MonitoringRef>%u</MonitoringRef>" // Third arg: Bus stop ID
            "<MaximumStopVisits>%u</MaximumStopVisits>" // Fourth arg: max records
            "<MaximumTextLength>%u</MaximumTextLength>" // Fifth arg: max text record length
            "<LineRef>ACT_%u</LineRef>" // Sixth arg: route number
            "</StopMonitoringRequest>"
            "</ServiceRequest>"
            "</Siri>";
}

class cbr_siri_decoder
{
public:
    cbr_siri_decoder() = default;
    esp_err_t init(const char *_api_key, const char *_url, size_t recv_max_len = 131072);
    esp_err_t poll_stop_prediction(uint32_t stop_id, uint32_t duration_minutes, uint32_t max_records, uint32_t max_txt_len, uint32_t route);
    esp_err_t decode_result(cbr_siri::record *records_out, size_t record_cnt, uint32_t timeout_ms = 3000);

private:
    static esp_err_t http_evt_handler(esp_http_client_event_t *evt);
    static esp_err_t parse_siri_iso8601(tm *time_out, const char *text, int *_utc_off = nullptr);

private:
    EventGroupHandle_t evt_group = nullptr;
    uint8_t *recv_buf = nullptr;
    size_t recv_len = 0;
    volatile size_t curr_recv_len = 0;
    const char *api_key = nullptr;
    const char *url = nullptr;
    static const constexpr char *TAG = "cbr_siri";

};
