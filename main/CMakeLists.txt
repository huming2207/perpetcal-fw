idf_component_register(
        SRCS
            "cal_main.cpp"
            "comm/usb_cdc.cpp" "comm/usb_cdc.hpp" "comm/config_manager.cpp" "comm/config_manager.hpp"
            "driver/lhs154kc.c" "driver/lhs154kc.h" "driver/lvgl_wrapper.c" "driver/lvgl_wrapper.h"
            "misc/file_utils.hpp" "misc/eternal.hpp"

        INCLUDE_DIRS
            "." "driver" "comm" "misc"

        REQUIRES
            esp_timer lvgl driver bisheng freertos tinyusb spi_flash nvs_flash app_update

        EMBED_FILES
            "resource/wqy.ttf"
)
