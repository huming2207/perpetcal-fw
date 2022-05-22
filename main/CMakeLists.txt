idf_component_register(
        SRCS
            "cal_main.cpp"
            "protocol/usb_cdc.cpp" "protocol/usb_cdc.hpp" "protocol/config_manager.cpp" "protocol/config_manager.hpp"
            "driver/lhs154kc.c" "driver/lhs154kc.h" "driver/lvgl_wrapper.c" "driver/lvgl_wrapper.h"

        INCLUDE_DIRS
            "." "driver" "protocol"

        REQUIRES
            esp_timer lvgl driver bisheng freertos tinyusb spi_flash nvs_flash

        EMBED_FILES
            "resource/wqy.ttf"
)
