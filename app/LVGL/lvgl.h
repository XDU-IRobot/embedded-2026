#ifndef LVGL_H
#define LVGL_H

#ifdef __cplusplus
extern "C" {
#endif

#define LVGL_VERSION_MAJOR 8
#define LVGL_VERSION_MINOR 3
#define LVGL_VERSION_PATCH 6
#define LVGL_VERSION_INFO "dev"

#define LV_VERSION_CHECK(x,y,z) (x == LVGL_VERSION_MAJOR && (y < LVGL_VERSION_MINOR || (y == LVGL_VERSION_MINOR && z <= LVGL_VERSION_PATCH)))

#include "src/hal/lv_hal.h"
#include "src/core/lv_obj.h"
#include "src/core/lv_group.h"
#include "src/core/lv_indev.h"
#include "src/core/lv_refr.h"
#include "src/core/lv_disp.h"
#include "src/core/lv_theme.h"
#include "src/core/lv_indev_scroll.h"
#include "src/core/lv_obj_class.h"
#include "src/core/lv_obj_pos.h"
#include "src/core/lv_obj_tree.h"
#include "src/core/lv_obj_draw.h"
#include "src/core/lv_obj_style.h"
#include "src/core/lv_obj_scroll.h"
#include "src/core/lv_event.h"

#include "src/font/lv_font.h"
#include "src/font/lv_font_loader.h"
#include "src/font/lv_font_fmt_txt.h"
#include "src/draw/lv_draw.h"

#include "src/misc/lv_log.h"
#include "src/misc/lv_timer.h"
#include "src/misc/lv_math.h"
#include "src/misc/lv_mem.h"
#include "src/misc/lv_async.h"
#include "src/misc/lv_anim_timeline.h"
#include "src/misc/lv_printf.h"
#include "src/misc/lv_style.h"
#include "src/misc/lv_anim.h"
#include "src/misc/lv_color.h"
#include "src/misc/lv_area.h"
#include "src/misc/lv_txt.h"
#include "src/misc/lv_bidi.h"
#include "src/misc/lv_txt_ap.h"

#include "src/widgets/lv_arc.h"
#include "src/widgets/lv_btn.h"
#include "src/widgets/lv_img.h"
#include "src/widgets/lv_label.h"
#include "src/widgets/lv_line.h"
#include "src/widgets/lv_table.h"
#include "src/widgets/lv_checkbox.h"
#include "src/widgets/lv_bar.h"
#include "src/widgets/lv_slider.h"
#include "src/widgets/lv_btnmatrix.h"
#include "src/widgets/lv_dropdown.h"
#include "src/widgets/lv_roller.h"
#include "src/widgets/lv_textarea.h"
#include "src/widgets/lv_canvas.h"
#include "src/widgets/lv_switch.h"
#include "src/widgets/lv_table.h"

#include "src/extra/lv_extra.h"
#include "src/lv_api_map.h"

#ifdef __cplusplus
}
#endif

#endif /*LVGL_H*/
