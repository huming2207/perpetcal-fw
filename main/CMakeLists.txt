idf_component_register(
        SRCS
            "cal_main.cpp"
            "comm/usb_cdc.cpp" "comm/usb_cdc.hpp" "comm/wifi_manager.cpp" "comm/wifi_manager.hpp"
            "comm/cbr_siri_decoder.cpp" "comm/cbr_siri_decoder.hpp"
            "storage/config_manager.cpp" "storage/config_manager.hpp"
            "driver/lhs154kc.c" "driver/lhs154kc.h" "driver/lvgl_wrapper.c" "driver/lvgl_wrapper.h"
            "misc/file_utils.hpp" "misc/eternal.hpp"
            "external/pugixml.cpp" "external/pugixml.hpp" "external/pugiconfig.hpp"

        INCLUDE_DIRS
            "." "driver" "comm" "misc" "storage" "external"

        REQUIRES
            esp_timer lvgl driver bisheng freertos tinyusb spi_flash nvs_flash app_update esp_http_client

        EMBED_FILES
            "resource/wqy.ttf"
)
