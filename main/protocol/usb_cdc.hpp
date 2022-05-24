#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <tinyusb.h>
#include <tusb_cdc_acm.h>
#include <esp_err.h>

#define SLIP_END            0xc0
#define SLIP_ESC            0xdb
#define SLIP_ESC_END        0xdc
#define SLIP_ESC_ESC        0xdd
#define CDC_OTA_PARTITION_AUTO_MAGIC "WHATEVER_NEXT"

#ifndef CONFIG_SI_DEVICE_MODEL
#define CDC_DEVICE_MODEL "PerpetCal Demo"
#else
#define CDC_DEVICE_MODEL CONFIG_SI_DEVICE_MODEL
#endif

#ifndef CONFIG_SI_DEVICE_BUILD
#define CDC_DEVICE_FW_VER "0.0.0-UNKNOWN"
#else
#define CDC_DEVICE_FW_VER CONFIG_SI_DEVICE_BUILD
#endif

namespace cdc_def
{
    enum chunk_xfer_type : uint8_t {
        XFER_FILE = 0,
        XFER_OTA = 1,
    };

    enum event : uint32_t {
        EVT_NEW_PACKET = BIT(0),
        EVT_READING_PKT = BIT(1),
        EVT_SLIP_ERROR = BIT(2),
    };

    enum pkt_type : uint8_t {
        PKT_ACK             = 0,
        PKT_DEVICE_INFO     = 0x01,
        PKT_PING            = 0x02,
        PKT_CHUNK_METADATA  = 0x10,
        PKT_DATA_CHUNK      = 0x11,
        PKT_CHUNK_ACK       = 0x12,
        PKT_KV_SET_U32      = 0x20,
        PKT_KV_GET_U32      = 0x21,
        PKT_KV_SET_I32      = 0x22,
        PKT_KV_GET_I32      = 0x23,
        PKT_KV_SET_STR      = 0x24,
        PKT_KV_GET_STR      = 0x25,
        PKT_KV_SET_BLOB     = 0x26,
        PKT_KV_GET_BLOB     = 0x27,
        PKT_KV_ENTRY_COUNT  = 0x28,
        PKT_KV_FLUSH        = 0x29,
        PKT_KV_DELETE       = 0x2a,
        PKT_KV_NUKE         = 0x2b,
        PKT_NACK = 0xff,
    };

    enum chunk_ack : uint8_t {
        CHUNK_XFER_DONE = 0,
        CHUNK_XFER_NEXT = 1,
        CHUNK_ERR_CRC32_FAIL = 2,
        CHUNK_ERR_INTERNAL = 3,
        CHUNK_ERR_ABORT_REQUESTED = 4,
        CHUNK_ERR_NAME_TOO_LONG = 5,
    };

    struct __attribute__((packed)) chunk_metadata_pkt {
        cdc_def::chunk_xfer_type type;
        uint32_t len;
        uint32_t crc; // Unused for OTA, as Espressif already has that SHA256 anyway??
        char path[128]; // Path for files, or partition name for OTA; leave CDC_OTA_PARTITION_AUTO_MAGIC (string "WHATEVER_NEXT") to let it automatically pick one
    };

    struct __attribute__((packed)) chunk_ack_pkt {
        chunk_ack state;

        // Can be:
        // 1. Next chunk offset (when state == 1)
        // 2. Expected CRC32 (when state == 2)
        // 3. Max length allowed (when state == 3)
        // 4. Just 0 (when state == anything else?)
        uint32_t aux_info;
    };

    struct __attribute__((packed)) header {
        pkt_type type;
        uint8_t len;
        uint16_t crc;
    };

    struct __attribute__((packed)) ack_pkt {
        pkt_type type;
        uint8_t len;
        uint16_t crc;
    };

    struct __attribute__((packed)) device_info {
        uint8_t mac_addr[6];
        uint8_t flash_id[8];
        char esp_idf_ver[32];
        char dev_model[32];
        char dev_build[32];
    };

    struct __attribute__((packed)) chunk_pkt {
        uint8_t len;
        uint8_t buf[UINT8_MAX];
    };

    struct __attribute__((packed)) kv_set_u32_pkt {
        char key[16];
        uint32_t value;
    };

    struct __attribute__((packed)) kv_set_i32_pkt {
        char key[16];
        uint8_t sign;
        uint32_t abs_val;
    };

    struct __attribute__((packed)) kv_set_blob_pkt {
        char key[16];
        uint8_t len;
        uint8_t buf[UINT8_MAX];
    };

    struct __attribute__((packed)) kv_set_str_pkt {
        char key[16];
        uint8_t len;
        char buf[UINT8_MAX];
    };

    struct __attribute__((packed)) kv_get_delete_pkt {
        char key[16];
    };

    struct __attribute__((packed)) kv_get_u32_pkt {
        uint32_t value;
    };

    struct __attribute__((packed)) kv_get_i32_pkt {
        uint8_t sign;
        uint32_t abs_val;
    };

    struct __attribute__((packed)) kv_get_blob_pkt {
        uint8_t len;
        uint8_t buf[UINT8_MAX];
    };

    struct __attribute__((packed)) kv_get_str_pkt {
        uint8_t len;
        char buf[UINT8_MAX];
    };
}

class usb_cdc
{
public:
    static usb_cdc& instance()
    {
        static usb_cdc instance;
        return instance;
    }

    usb_cdc(usb_cdc const &) = delete;
    void operator=(usb_cdc const &) = delete;

private:
    usb_cdc() = default;
    static void serial_rx_cb(int itf, cdcacm_event_t *event);
    [[noreturn]] static void rx_handler_task(void *ctx);
    static esp_err_t send_pkt(cdc_def::pkt_type type, const uint8_t *buf, size_t len, uint32_t timeout_ms = portMAX_DELAY);
    static esp_err_t encode_and_tx(const uint8_t *header_buf, size_t header_len, const uint8_t *buf, size_t len, uint32_t timeout_ms = portMAX_DELAY);
    static inline uint16_t get_crc16(const uint8_t *buf, size_t len, uint16_t init = 0x0000);

public:
    esp_err_t init();
    esp_err_t pause_usb();
    esp_err_t unpause_usb();

private:
    void parse_pkt();
    void parse_chunk();
    void parse_chunk_metadata();
    void parse_kv_get_u32();
    void parse_kv_set_u32();
    void parse_kv_get_i32();
    void parse_kv_set_i32();
    void parse_kv_get_str();
    void parse_kv_set_str();
    void parse_kv_get_blob();
    void parse_kv_set_blob();
    static void parse_kv_cnt();
    static void parse_kv_flush();
    void parse_kv_delete();
    static void parse_kv_nuke();

private:
    static esp_err_t send_ack();
    static esp_err_t send_nack();
    static esp_err_t send_dev_info(uint32_t timeout_ms = portMAX_DELAY);
    static esp_err_t send_chunk_ack(cdc_def::chunk_ack state, uint32_t aux = 0, uint32_t timeout_ms = portMAX_DELAY);

private:
    static const constexpr char *TAG = "usb_cdc";
    EventGroupHandle_t rx_event = nullptr;
    volatile bool busy_decoding = false;
    volatile bool paused = false;
    volatile bool on_chunk_xfer = false;
    volatile bool on_ota_xfer = false;
    volatile size_t decoded_len = 0;
    volatile size_t raw_len = 0;
    size_t file_expect_len = 0;
    size_t file_curr_offset = 0;
    uint32_t file_crc = 0;
    uint8_t *raw_buf = nullptr;
    uint8_t *decoded_buf = nullptr;
    uint8_t *algo_buf = nullptr;
    FILE *file_handle = nullptr;
    esp_ota_handle_t ota_handle = 0;
};
