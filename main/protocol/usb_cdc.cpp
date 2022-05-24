#include <esp_log.h>
#include <esp_crc.h>
#include <esp_mac.h>
#include <esp_flash.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>

#include "usb_cdc.hpp"
#include "config_manager.hpp"
#include "file_utils.hpp"

esp_err_t usb_cdc::init()
{
    static char sn_str[32] = {};
    uint8_t sn_buf[16] = { 0 };
    esp_efuse_mac_get_default(sn_buf);
    esp_flash_read_unique_chip_id(esp_flash_default_chip, reinterpret_cast<uint64_t *>(sn_buf + 6));

    snprintf(sn_str, 32, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
             sn_buf[0], sn_buf[1], sn_buf[2], sn_buf[3], sn_buf[4], sn_buf[5], sn_buf[6], sn_buf[7],
             sn_buf[8], sn_buf[9], sn_buf[10], sn_buf[11], sn_buf[12], sn_buf[13]);

    ESP_LOGI(TAG, "Initialised with SN: %s", sn_str);

    static const char lang[2] = {0x09, 0x04};
    static tusb_desc_strarray_device_t desc_str = {
            // array of pointer to string descriptors
            lang,                // 0: is supported language is English (0x0409)
            "Jackson Hu", // 1: Manufacturer
            "PerpetCal Configurator",      // 2: Product
            sn_str,       // 3: Serials, should use chip ID
            "PerpetCal Configurator",          // 4: CDC Interface
            "",
            "",
    };
    static tusb_desc_device_t desc_device = {};

    tinyusb_config_t tusb_cfg = {}; // the configuration using default values
    tusb_cfg.string_descriptor = desc_str;

    auto ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB driver install failed");
        return ret;
    }

    tinyusb_config_cdcacm_t acm_cfg = {};
    acm_cfg.usb_dev = TINYUSB_USBDEV_0;
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_0;
    acm_cfg.rx_unread_buf_sz = 512;
    acm_cfg.callback_rx = &serial_rx_cb;
    acm_cfg.callback_rx_wanted_char = nullptr;
    acm_cfg.callback_line_state_changed = nullptr;
    acm_cfg.callback_line_coding_changed = nullptr;

    ret = ret ?: tusb_cdc_acm_init(&acm_cfg);

    rx_event = xEventGroupCreate();
    if (rx_event == nullptr) {
        ESP_LOGE(TAG, "Failed to create Rx event group");
        return ESP_ERR_NO_MEM;
    }

    xTaskCreatePinnedToCore(rx_handler_task, "cdc_rx", 16384, this, tskIDLE_PRIORITY + 1, nullptr, 0);

    decoded_buf = static_cast<uint8_t *>(heap_caps_malloc(CONFIG_TINYUSB_CDC_RX_BUFSIZE, MALLOC_CAP_INTERNAL));
    if (decoded_buf == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate SLIP decode buf");
        return ESP_ERR_NO_MEM;
    }

    raw_buf = static_cast<uint8_t *>(heap_caps_malloc(CONFIG_TINYUSB_CDC_RX_BUFSIZE, MALLOC_CAP_INTERNAL));
    if (raw_buf == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate SLIP raw buf");
        free(decoded_buf);
        return ESP_ERR_NO_MEM;
    }

    return ret;
}

void usb_cdc::serial_rx_cb(int itf, cdcacm_event_t *event)
{
    auto &ctx = usb_cdc::instance();

    uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE] = { 0 };
    size_t rx_size = 0;
    auto ret = tinyusb_cdcacm_read(static_cast<tinyusb_cdcacm_itf_t>(itf), rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB read fail!");
        return;
    }

    if (rx_size < 1) {
        return;
    } else {
        memcpy(ctx.raw_buf + ctx.raw_len, rx_buf, rx_size);
        ctx.raw_len += rx_size;
    }

    // Start to decode if this is the last packet, otherwise continue to cache
    if (rx_buf[rx_size - 1] == SLIP_END) {
        size_t idx = 0;
        while (idx < ctx.raw_len && ctx.decoded_len < CONFIG_TINYUSB_CDC_RX_BUFSIZE) {
            if (ctx.raw_buf[idx] == SLIP_END) {
                if (ctx.decoded_len > 0) {
                    ESP_LOGI(TAG, "Before SLIP, size %u:", ctx.raw_len);
                    xEventGroupSetBits(ctx.rx_event, cdc_def::EVT_NEW_PACKET);
                    ctx.raw_len = 0;
                    memset(ctx.raw_buf, 0, CONFIG_TINYUSB_CDC_RX_BUFSIZE);
                } else {
                    xEventGroupClearBits(ctx.rx_event, cdc_def::EVT_NEW_PACKET);
                }
            } else if (ctx.raw_buf[idx] == SLIP_ESC) {
                idx += 1;
                if (ctx.raw_buf[idx] == SLIP_ESC_END) {
                    ctx.decoded_buf[ctx.decoded_len] = SLIP_END;
                } else if (ctx.raw_buf[idx] == SLIP_ESC_ESC) {
                    ctx.decoded_buf[ctx.decoded_len] = SLIP_ESC;
                } else {
                    xEventGroupSetBits(ctx.rx_event, cdc_def::EVT_SLIP_ERROR);
                    ESP_LOGE(TAG, "SLIP decoding detected a corrupted packet");
                    return;
                }

                ctx.decoded_len += 1;
            } else {
                ctx.decoded_buf[ctx.decoded_len] = ctx.raw_buf[idx];
                ctx.decoded_len += 1;
            }

            idx += 1;
        }
    }
}

[[noreturn]] void usb_cdc::rx_handler_task(void *_ctx)
{
    ESP_LOGI(TAG, "Rx handler task started");
    auto &ctx = usb_cdc::instance();
    while(true) {
        if (xEventGroupWaitBits(ctx.rx_event, cdc_def::EVT_NEW_PACKET, pdTRUE, pdFALSE, portMAX_DELAY) == pdTRUE) {
            // Pause Rx
            tinyusb_cdcacm_unregister_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_RX);

            ESP_LOGI(TAG, "Now in buffer, len: %u :", ctx.decoded_len);
            // ESP_LOG_BUFFER_HEX(TAG, ctx.decoded_buf, ctx.decoded_len);

            // Now do parsing
            ctx.parse_pkt();

            // Clear up the mess
            ctx.decoded_len = 0;
            memset(ctx.decoded_buf, 0, CONFIG_TINYUSB_CDC_RX_BUFSIZE);

            // Restart Rx
            tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_RX, serial_rx_cb);
        }
    }
}

esp_err_t usb_cdc::send_ack()
{
    return send_pkt(cdc_def::PKT_ACK, nullptr, 0);
}

esp_err_t usb_cdc::send_nack()
{
    return send_pkt(cdc_def::PKT_NACK, nullptr, 0);
}

esp_err_t usb_cdc::send_dev_info(uint32_t timeout_ms)
{
    const char *idf_ver = IDF_VER;
    const char *dev_model = CDC_DEVICE_MODEL;
    const char *dev_build = CDC_DEVICE_FW_VER;

    cdc_def::device_info dev_info = {};
    auto ret = esp_efuse_mac_get_default(dev_info.mac_addr);
    ret = ret ?: esp_flash_read_unique_chip_id(esp_flash_default_chip, (uint64_t *)dev_info.flash_id);
    strcpy(dev_info.esp_idf_ver, idf_ver);
    strcpy(dev_info.dev_build, dev_build);
    strcpy(dev_info.dev_model, dev_model);

    ret = ret ?: send_pkt(cdc_def::PKT_DEVICE_INFO, (uint8_t *)&dev_info, sizeof(dev_info), timeout_ms);

    return ret;
}

esp_err_t usb_cdc::send_chunk_ack(cdc_def::chunk_ack state, uint32_t aux, uint32_t timeout_ms)
{
    cdc_def::chunk_ack_pkt pkt = {};
    pkt.aux_info = aux;
    pkt.state = state;

    return send_pkt(cdc_def::PKT_CHUNK_ACK, (uint8_t *)&pkt, sizeof(pkt), timeout_ms);
}

esp_err_t usb_cdc::send_pkt(cdc_def::pkt_type type, const uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (buf == nullptr && len > 0) return ESP_ERR_INVALID_ARG;

    cdc_def::header header = {};
    header.type = type;
    header.len = len;
    header.crc = 0; // Set later
    uint16_t crc = get_crc16((uint8_t *) &header, sizeof(header));

    // When packet has no data body, just send header (e.g. ACK)
    if (buf == nullptr || len < 1) {
        header.crc = crc;
        return encode_and_tx((uint8_t *)&header, sizeof(header), nullptr, 0, timeout_ms);
    } else {
        crc = get_crc16(buf, len, crc);
        header.crc = crc;
        return encode_and_tx((uint8_t *)&header, sizeof(header), buf, len, timeout_ms);
    }
}

esp_err_t usb_cdc::encode_and_tx(const uint8_t *header_buf, size_t header_len,
                                 const uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    const uint8_t slip_esc_end[] = { SLIP_ESC, SLIP_ESC_END };
    const uint8_t slip_esc_esc[] = { SLIP_ESC, SLIP_ESC_ESC };

    if (header_buf == nullptr || header_len < 1) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t end = SLIP_END;

    if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, &end, 1) < 1) {
        ESP_LOGE(TAG, "Failed to encode and tx end char");
        return ESP_ERR_INVALID_STATE;
    }

    size_t header_idx = 0;
    while (header_idx < header_len) {
        if (header_buf[header_idx] == SLIP_END) {
            if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, slip_esc_end, sizeof(slip_esc_end)) < sizeof(slip_esc_end)) {
                ESP_LOGE(TAG, "Failed to encode and tx SLIP_END");
                return ESP_ERR_INVALID_STATE;
            }
        } else if (header_buf[header_idx] == SLIP_ESC) {
            if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, slip_esc_esc, sizeof(slip_esc_esc)) < sizeof(slip_esc_esc)) {
                ESP_LOGE(TAG, "Failed to encode and tx SLIP_ESC");
                return ESP_ERR_INVALID_STATE;
            }
        } else {
            if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, &header_buf[header_idx], 1) < 1) {
                ESP_LOGE(TAG, "Failed to encode and tx data");
                return ESP_ERR_INVALID_STATE;
            }
        }

        header_idx += 1;
    }

    if (buf != nullptr && len > 1) {
        size_t payload_idx = 0;
        while (payload_idx < len) {
            if (buf[payload_idx] == SLIP_END) {
                if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, slip_esc_end, sizeof(slip_esc_end)) < sizeof(slip_esc_end)) {
                    ESP_LOGE(TAG, "Failed to encode and tx SLIP_END");
                    return ESP_ERR_INVALID_STATE;
                }
            } else if (buf[payload_idx] == SLIP_ESC) {
                if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, slip_esc_esc, sizeof(slip_esc_esc)) < sizeof(slip_esc_esc)) {
                    ESP_LOGE(TAG, "Failed to encode and tx SLIP_ESC");
                    return ESP_ERR_INVALID_STATE;
                }
            } else {
                if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, &buf[payload_idx], 1) < 1) {
                    ESP_LOGE(TAG, "Failed to encode and tx data");
                    return ESP_ERR_INVALID_STATE;
                }
            }

            payload_idx += 1;
        }
    }

    if (tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, &end, 1) < 1) {
        ESP_LOGE(TAG, "Failed to encode and tx end char");
        return ESP_ERR_INVALID_STATE;
    }

    return tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(timeout_ms));
}

uint16_t usb_cdc::get_crc16(const uint8_t *buf, size_t len, uint16_t init)
{
//  * CRC-16/XMODEM, poly= 0x1021, init = 0x0000, refin = false, refout = false, xorout = 0x0000
// *     crc = ~crc16_be((uint16_t)~0x0000, buf, length);
    if (buf == nullptr || len < 1) {
        return 0;
    }

    return ~esp_crc16_be((uint16_t)~init, buf, len);
}

void usb_cdc::parse_pkt()
{
    if (decoded_len < sizeof(cdc_def::header)) {
        ESP_LOGW(TAG, "Packet too short, failed to decode header: %u", decoded_len);
        recv_state = cdc_def::FILE_RECV_NONE;
        send_nack();
        return;
    }

    auto *header = (cdc_def::header *)decoded_buf;

    uint16_t expected_crc = header->crc;
    header->crc = 0;

    uint16_t actual_crc = get_crc16(decoded_buf, decoded_len);
    if (actual_crc != expected_crc) {
        ESP_LOGW(TAG, "Incoming packet CRC corrupted, expect 0x%x, actual 0x%x", expected_crc, actual_crc);
        send_nack();
        return;
    }

    if (recv_state != cdc_def::FILE_RECV_NONE && header->type != cdc_def::PKT_DATA_CHUNK) {
        ESP_LOGW(TAG, "Invalid state - data chunk expected while received type 0x%x", header->type);
        send_nack();
        return;
    }

    switch (header->type) {
        case cdc_def::PKT_PING: {
            send_ack();
            break;
        }

        case cdc_def::PKT_DEVICE_INFO: {
            send_dev_info();
            break;
        }

        case cdc_def::PKT_DATA_CHUNK: {
            if (recv_state != cdc_def::FILE_RECV_NONE) {
                parse_chunk();
            } else {
                ESP_LOGW(TAG, "Invalid state - no chunk expected to come or should have EOL'ed??");
                send_chunk_ack(cdc_def::CHUNK_ERR_INTERNAL, 0);
            }
            break;
        }

        default: {
            ESP_LOGW(TAG, "Unknown packet type 0x%x received", header->type);
            send_nack();
            break;
        }
    }
}

void usb_cdc::send_curr_config()
{
    auto &cfg_mgr = config_manager::instance();

    if (!cfg_mgr.has_valid_cfg()) {
        if(cfg_mgr.load_default_cfg() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load default config");
            send_nack();
            return;
        }
    }

    uint8_t buf[sizeof(cfg_def::config_pkt)] = { 0 };
    cfg_mgr.read_cfg(buf, sizeof(cfg_def::config_pkt));
    send_pkt(cdc_def::PKT_CURR_CONFIG, buf, sizeof(cfg_def::config_pkt));
}

void usb_cdc::parse_set_config()
{
    auto *buf = (uint8_t *)(decoded_buf + sizeof(cdc_def::header));

    auto &cfg_mgr = config_manager::instance();
    auto ret = cfg_mgr.save_cfg(buf, decoded_len - sizeof(cdc_def::header));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set config failed, returned 0x%x: %s", ret, esp_err_to_name(ret));
        send_nack();
    } else {
        send_ack();
    }
}

void usb_cdc::parse_get_algo_info()
{

}

void usb_cdc::parse_set_algo_metadata()
{
    auto *algo_info = (cdc_def::algo_info *)(decoded_buf + sizeof(cdc_def::header));
    if (algo_info->len > CFG_MGR_FLASH_ALGO_MAX_SIZE || heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) < algo_info->len) {
        ESP_LOGE(TAG, "Flash algo metadata len too long: %u, free block: %u", algo_info->len, heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        send_nack();
        return;
    }

    file_expect_len = algo_info->len;
    file_crc = algo_info->crc;
    algo_buf = static_cast<uint8_t *>(heap_caps_malloc(algo_info->len, MALLOC_CAP_INTERNAL));
    memset(algo_buf, 0, algo_info->len);
    recv_state = cdc_def::FILE_RECV_ALGO;
    send_chunk_ack(cdc_def::CHUNK_XFER_NEXT, 0);
}

void usb_cdc::parse_get_fw_info()
{

}

void usb_cdc::parse_chunk_metadata()
{
    auto *fw_info = (cdc_def::chunk_metadata_pkt *)(decoded_buf + sizeof(cdc_def::header));
    if (fw_info->len > UINT32_MAX || heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) < fw_info->len) {
        ESP_LOGE(TAG, "Firmware metadata len too long: %u, free heap: %u", fw_info->len, heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        heap_caps_dump(MALLOC_CAP_INTERNAL);
        send_nack();
        return;
    }

    file_expect_len = fw_info->len;
    file_crc = fw_info->crc;
    if (fw_info->type == cdc_def::XFER_OTA) {
        fw_info->path[sizeof(fw_info->path) - 1] = '\0';
        const esp_partition_t *part;
        if (strlen(fw_info->path) < 1 || strcmp(fw_info->path, CDC_OTA_PARTITION_AUTO_MAGIC) == 0) {
            part = esp_ota_get_next_update_partition(nullptr);
        } else {
            part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, fw_info->path);
        }

        ESP_LOGW(TAG, "OTA part: %s, addr 0x%x, now start OTA xfer", part->label, part->address);
        if (esp_ota_begin(part, fw_info->len, &ota_handle) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start OTA");
            send_nack();
            return;
        }

        on_chunk_xfer = true;
        on_ota_xfer = true;
        ESP_LOGW(TAG, "OTA xfer started");
    } else if (fw_info->type == cdc_def::XFER_FILE) {
        file_handle = fopen(fw_info->path, "wb");
        if (file_handle == nullptr) {
            ESP_LOGE(TAG, "Failed to open firmware path");
            send_nack();
            return;
        }

        on_chunk_xfer = true;
        on_ota_xfer = false;
    }


    send_chunk_ack(cdc_def::CHUNK_XFER_NEXT, 0);
}

void usb_cdc::parse_chunk()
{
    auto *chunk = (cdc_def::chunk_pkt *)(decoded_buf + sizeof(cdc_def::header));

    // Scenario -1: something screwed up
    if (!on_chunk_xfer) {
        ESP_LOGE(TAG, "No metadata received for xfer");
        send_chunk_ack(cdc_def::CHUNK_ERR_INTERNAL, 0);
    }

    // Scenario 0: if len == 0 then that's force abort, discard the buffer and set back the states
    if (chunk->len == 0) {
        ESP_LOGE(TAG, "Zero len chunk - force abort!");
        file_expect_len = 0;
        file_curr_offset = 0;
        file_crc = 0;
        if (algo_buf != nullptr) {
            free(algo_buf);
            algo_buf = nullptr;
        }
        send_chunk_ack(cdc_def::CHUNK_ERR_ABORT_REQUESTED, 0);
        return;
    }

    // Scenario 1: if len is too long, reject & abort.
    if (chunk->len + file_curr_offset > file_expect_len) {
        ESP_LOGE(TAG, "Chunk recv buffer is full, incoming %u while expect %u only", chunk->len + file_curr_offset, file_expect_len);
        file_expect_len = 0;
        file_curr_offset = 0;
        file_crc = 0;
        if (algo_buf != nullptr) {
            free(algo_buf);
            algo_buf = nullptr;
        }

        if (file_handle != nullptr) {
            fclose(file_handle);
            file_handle = nullptr;
        }

        send_chunk_ack(cdc_def::CHUNK_ERR_NAME_TOO_LONG, chunk->len + file_curr_offset);
        return;
    }

    // Scenario 2: Normal recv
    if (on_ota_xfer) {
        ESP_LOGD(TAG, "Copy to: %p; len: %u, off: %u", algo_buf + file_curr_offset, chunk->len, file_curr_offset);
        file_curr_offset += chunk->len; // Add offset
    } else {
        if (fwrite(chunk->buf, 1, chunk->len, file_handle) < chunk->len) {
            ESP_LOGE(TAG, "Error occur when processing recv buffer - write failed");
            send_chunk_ack(cdc_def::CHUNK_ERR_INTERNAL, ESP_ERR_NO_MEM);
            return;
        }

        fflush(file_handle);
        file_curr_offset += chunk->len; // Add offset
    }

    if (file_curr_offset == file_expect_len) {
        bool crc_match = false;
        if (recv_state == cdc_def::FILE_RECV_ALGO) {
            crc_match = (esp_crc32_le(0, algo_buf, file_expect_len) == file_crc);
        } else if(recv_state == cdc_def::FILE_RECV_FW) {
            if (file_handle != nullptr) {
                fflush(file_handle);
                fclose(file_handle);
                file_handle = nullptr;
            }

            crc_match = (file_utils::validate_firmware_file(config_manager::FIRMWARE_PATH, file_crc) == ESP_OK);
        }

        if (crc_match) {
            ESP_LOGI(TAG, "Chunk recv successful, got %u bytes", file_expect_len);

            auto ret = ESP_OK;
            auto &cfg_mgr = config_manager::instance();
            if (recv_state == cdc_def::FILE_RECV_ALGO) {
                ret = cfg_mgr.save_algo(algo_buf, file_expect_len);

                if (algo_buf != nullptr) {
                    free(algo_buf);
                    algo_buf = nullptr;
                }
            } else if(recv_state == cdc_def::FILE_RECV_FW) {
                ret = cfg_mgr.set_fw_crc(file_crc);
            }

            file_expect_len = 0;
            file_curr_offset = 0;
            file_crc = 0;
            recv_state = cdc_def::FILE_RECV_NONE;

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Error occur when processing recv buffer, returned 0x%x: %s", ret, esp_err_to_name(ret));
                send_chunk_ack(cdc_def::CHUNK_ERR_INTERNAL, ret);
            } else {
                ESP_LOGI(TAG, "Chunk transfer done!");
                send_chunk_ack(cdc_def::CHUNK_XFER_DONE, file_curr_offset);
            }
        } else {
            ESP_LOGE(TAG, "Chunk recv CRC mismatched!");
            send_chunk_ack(cdc_def::CHUNK_ERR_CRC32_FAIL, 0);
        }
    } else {
        ESP_LOGI(TAG, "Chunk recv - await next @ %u, total %u", file_curr_offset, file_expect_len);
        send_chunk_ack(cdc_def::CHUNK_XFER_NEXT, file_curr_offset);
    }
}

esp_err_t usb_cdc::pause_usb()
{
    if (!tusb_inited() || paused) return ESP_ERR_INVALID_STATE;
    xEventGroupClearBits(rx_event, cdc_def::EVT_NEW_PACKET);

    paused = true;
    return tinyusb_cdcacm_unregister_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_RX);
}

esp_err_t usb_cdc::unpause_usb()
{
    if (!tusb_inited() || !paused) return ESP_ERR_INVALID_STATE;
    decoded_len = 0;
    memset(decoded_buf, 0, CONFIG_TINYUSB_CDC_RX_BUFSIZE);

    paused = false;
    return tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_RX, serial_rx_cb);
}

void usb_cdc::parse_kv_get_u32()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_get_delete_pkt *)(decoded_buf + sizeof(cdc_def::header));

    uint32_t val = 0;
    if (cfg.get_u32(buf->key, val) != ESP_OK) {
        send_nack();
    } else {
        cdc_def::kv_get_u32_pkt pkt = {};
        pkt.value = val;
        send_pkt(cdc_def::PKT_KV_GET_U32, (uint8_t *)&pkt, sizeof(pkt));
    }
}

void usb_cdc::parse_kv_set_u32()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_set_u32_pkt *)(decoded_buf + sizeof(cdc_def::header));
    if (cfg.set_u32(buf->key, buf->value) != ESP_OK) {
        send_nack();
    } else {
        send_ack();
    }
}

void usb_cdc::parse_kv_get_i32()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_get_delete_pkt *)(decoded_buf + sizeof(cdc_def::header));

    int32_t val = 0;
    if (cfg.get_i32(buf->key, val) != ESP_OK) {
        send_nack();
    } else {
        cdc_def::kv_get_i32_pkt pkt = {};
        pkt.abs_val = abs(val);
        pkt.sign = val < 0 ? 1 : 0;
        send_pkt(cdc_def::PKT_KV_GET_I32, (uint8_t *)&pkt, sizeof(pkt));
    }
}

void usb_cdc::parse_kv_set_i32()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_set_i32_pkt *)(decoded_buf + sizeof(cdc_def::header));
    auto value = (int32_t)(buf->abs_val * (buf->sign ? -1 : 1));
    if (cfg.set_i32(buf->key, value) != ESP_OK) {
        send_nack();
    } else {
        send_ack();
    }
}

void usb_cdc::parse_kv_get_str()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_get_delete_pkt *)(decoded_buf + sizeof(cdc_def::header));

    cdc_def::kv_get_str_pkt pkt = {};
    if (cfg.get_str(buf->key, pkt.buf, sizeof(pkt.buf) - 1) != ESP_OK) {
        send_nack();
    } else {
        send_pkt(cdc_def::PKT_KV_GET_STR, (uint8_t *)&pkt, sizeof(pkt));
    }
}

void usb_cdc::parse_kv_set_str()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_set_str_pkt *)(decoded_buf + sizeof(cdc_def::header));
    buf->buf[sizeof(buf->buf) - 1] = '\0';
    if (buf->len != strlen(buf->buf)) {
        send_nack();
        return;
    }

    if (cfg.set_str(buf->key, buf->buf) != ESP_OK) {
        send_nack();
    } else {
        send_ack();
    }
}

void usb_cdc::parse_kv_get_blob()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_get_delete_pkt *)(decoded_buf + sizeof(cdc_def::header));

    cdc_def::kv_get_blob_pkt pkt = {};
    if (cfg.get_blob(buf->key, pkt.buf, sizeof(pkt.buf) - 1) != ESP_OK) {
        send_nack();
    } else {
        send_pkt(cdc_def::PKT_KV_GET_BLOB, (uint8_t *)&pkt, sizeof(pkt));
    }
}

void usb_cdc::parse_kv_set_blob()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_set_blob_pkt *)(decoded_buf + sizeof(cdc_def::header));
    if (cfg.set_blob(buf->key, buf->buf, buf->len) != ESP_OK) {
        send_nack();
    } else {
        send_ack();
    }
}

void usb_cdc::parse_kv_cnt()
{
    cdc_def::kv_get_u32_pkt pkt = {};
    auto &cfg = config_manager::instance();
    pkt.value = cfg.used_count();
    send_pkt(cdc_def::PKT_KV_ENTRY_COUNT, (uint8_t *)(&pkt), sizeof(pkt));
}

void usb_cdc::parse_kv_flush()
{
    auto &cfg = config_manager::instance();
    if (cfg.flush() != ESP_OK) {
        send_nack();
    } else {
        send_ack();
    }
}

void usb_cdc::parse_kv_delete()
{
    auto &cfg = config_manager::instance();
    auto *buf = (cdc_def::kv_get_delete_pkt *)(decoded_buf + sizeof(cdc_def::header));
    if (cfg.erase(buf->key) != ESP_OK) {
        send_nack();
    } else {
        send_ack();
    }
}

void usb_cdc::parse_kv_nuke()
{
    auto &cfg = config_manager::instance();
    if (cfg.nuke() != ESP_OK) {
        send_nack();
    } else {
        send_ack();
    }
}



