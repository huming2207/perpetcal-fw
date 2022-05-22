#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stb_font_view.hpp>
#include "lvgl_wrapper.h"

extern const uint8_t wqy_font_start[]   asm("_binary_wqy_ttf_start");
extern const uint8_t wqy_font_end[]     asm("_binary_wqy_ttf_end");

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(cal_disp_init());

    stb_font_view font;
    ESP_ERROR_CHECK(font.init(wqy_font_start, (wqy_font_end - wqy_font_start), 72));

    lv_obj_t *label = lv_label_create(lv_scr_act());

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_text_color(&style, lv_color_black());

    font.decorate_font_style(&style);
    lv_obj_set_width(label, 240);
    lv_obj_add_style(label, &style, 0);
    lv_label_set_text(label, "爱情的\n号码牌");

    while(true) {
        vTaskDelay(1);
        lv_task_handler();
    }
}
