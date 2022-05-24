idf_component_register(
        SRCS
            "cal_main.cpp"
            "protocol/usb_cdc.cpp" "protocol/usb_cdc.hpp" "protocol/config_manager.cpp" "protocol/config_manager.hpp"
            "driver/lhs154kc.c" "driver/lhs154kc.h" "driver/lvgl_wrapper.c" "driver/lvgl_wrapper.h"
            "misc/file_utils.hpp"

        INCLUDE_DIRS
            "." "driver" "protocol" "misc"

        REQUIRES
            esp_timer lvgl driver bisheng freertos tinyusb spi_flash nvs_flash app_update

        EMBED_FILES
            "resource/wqy.ttf"
)
