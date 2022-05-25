#pragma once

#include <esp_err.h>
#include <memory>
#include <nvs_handle.hpp>

#include "eternal.hpp"

#define CFG_MGR_NVS_NAMESPACE "perpetcal"

namespace config
{
    enum entry_type : uint8_t
    {
        ENTRY_INVALID = 0,
        ENTRY_U32 = 0x01,
        ENTRY_I32 = 0x02,
        ENTRY_STR = 0x03,
        ENTRY_BLOB = 0x04,
    };

    enum entry_policy : uint8_t
    {
        POLICY_USB_UNREADABLE = BIT0,
        POLICY_BLE_UNREADABLE = BIT1,
        POLICY_CLOUD_UNREADABLE = BIT2, // Reserved for Guanlan Cloud platform?
        POLICY_EXT_UNREADABLE = (BIT0 | BIT1 | BIT2),
    };
}

class config_manager
{
public:
    static config_manager& instance()
    {
        static config_manager instance;
        return instance;
    }

    config_manager(config_manager const &) = delete;
    void operator=(config_manager const &) = delete;

private:
    config_manager() = default;
    static const constexpr char *TAG = "cfg_mgr";
    std::shared_ptr<nvs::NVSHandle> nvs_handle = nullptr;

public:
    esp_err_t init();
    esp_err_t get_u32(const char *key, uint32_t &out);
    esp_err_t get_i32(const char *key, int32_t &out);
    esp_err_t set_u32(const char *key, uint32_t in);
    esp_err_t set_i32(const char *key, int32_t in);
    esp_err_t get_str(const char *key, char *out, size_t len);
    esp_err_t get_blob(const char *key, uint8_t *out, size_t len);
    esp_err_t set_str(const char *key, const char *in);
    esp_err_t set_blob(const char *key, const uint8_t *in, size_t len);
    esp_err_t erase(const char *key);
    esp_err_t nuke();
    esp_err_t flush();
    size_t used_count();
};

