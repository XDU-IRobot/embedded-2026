#define _DEFAULT_SOURCE /* needed for usleep() */
#include <stdlib.h>
#include <stdint.h>
#include "../../LVGL/lvgl.h" // 引用真正的 LVGL 库内头文件，而不是当前目录下的同名文件
#include "lcd_init.h"
#include "sd_card.h" // 引入自定义的 SD 卡存储及读取封装函数
#include "ff.h"
// 实体定义，确保链接器能找到
float Pitch[4] = {11.11f, 22.22f, 33.33f, 44.44f};
float Yaw[4]   = {45.55f, 45.66f, 45.77f, 45.88f};

// 定义三个界面的容器
static lv_obj_t * view_main;
static lv_obj_t * view_list;
static lv_obj_t * view_edit; // 独立的修改数值界面

// 定义用来显示具体数值的Label
static lv_obj_t * label_list_btns[4]; // 列表界面的4个按钮文本
static lv_obj_t * label_edit_val;     // 在编辑界面显示的 Label

int current_category = 0; // 0代表PITCH, 1代表YAW
int current_index = 0;    // 记录列表选中的序号
float temp_val = 0;
static const float steps[4] = {10.0f, 1.0f, 0.1f, 0.01f};

static lv_indev_t * keypad_indev; // 全局记录输入设备，以便切换不同的焦点组
static lv_group_t * g_main;
static lv_group_t * g_list;
static lv_group_t * g_edit; // 编辑界面的焦点组

bool is_editing_val = false; // Add a flag to indicate if we are inside "edit" mode

// 记录下需要被聚焦的按钮对象
static lv_obj_t * btn_pitch;
static lv_obj_t * btn_list_first;
static lv_obj_t * btn_edit_first; // 编辑界面第一个步长按钮

static void hal_init(void);

// === 事件回调函数 ===
static void update_list_labels(void) {
    for(int i = 0; i < 4; i++) {
        float val = (current_category == 0) ? Pitch[i] : Yaw[i];
        int val_int = (int)val;
        int val_frac = (int)(val * 100.0f) % 100;
        if (val_frac < 0) val_frac = -val_frac;

        if (val < 0 && val_int == 0) {
            lv_label_set_text_fmt(label_list_btns[i], "%d\n-0.%02d", i + 1, val_frac);
        } else {
            lv_label_set_text_fmt(label_list_btns[i], "%d\n%d.%02d", i + 1, val_int, val_frac);
        }
    }
}

static void update_edit_label_value(float val) {
    int val_int = (int)val;
    int val_frac = (int)(val * 100.0f) % 100;
    if (val_frac < 0) val_frac = -val_frac;

    if (val < 0 && val_int == 0) {
        lv_label_set_text_fmt(label_edit_val, "-0.%02d", val_frac);
    } else {
        lv_label_set_text_fmt(label_edit_val, "%d.%02d", val_int, val_frac);
    }
}

static void btn_main_cb(lv_event_t * e) {
    int category = (int)(intptr_t)lv_event_get_user_data(e);
    current_category = category;

    update_list_labels(); // 跳转前更新列表中显示的数值

    // 隐藏主菜单，显示列表菜单
    lv_obj_add_flag(view_main, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(view_list, LV_OBJ_FLAG_HIDDEN);

    // 切换到列表菜单组
    lv_indev_set_group(keypad_indev, g_list);
    if (btn_list_first) lv_group_focus_obj(btn_list_first);
}

static void btn_list_to_edit_cb(lv_event_t * e) {
    int index = (int)(intptr_t)lv_event_get_user_data(e);
    current_index = index;

    // 隐藏列表菜单，显示编辑菜单
    lv_obj_add_flag(view_list, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(view_edit, LV_OBJ_FLAG_HIDDEN);

    temp_val = (current_category == 0) ? Pitch[current_index] : Yaw[current_index];
    update_edit_label_value(temp_val);

    is_editing_val = true; // Enter edit mode

    // 切换到编辑菜单组，赋焦点给第一个步长按钮
    lv_indev_set_group(keypad_indev, g_edit);
    if (btn_edit_first) lv_group_focus_obj(btn_edit_first);
}

static void btn_list_return_cb(lv_event_t * e) {
    // 隐藏列表菜单，显示主菜单
    lv_obj_add_flag(view_list, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(view_main, LV_OBJ_FLAG_HIDDEN);

    // 切换回主菜单组
    lv_indev_set_group(keypad_indev, g_main);
    if (btn_pitch) lv_group_focus_obj(btn_pitch);
}

// 增加一个红色的焦点样式，当按键选中此项目时高亮
static lv_style_t style_btn_focused;
static void style_init(void) {
    lv_style_init(&style_btn_focused);
    lv_style_set_border_color(&style_btn_focused, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_border_width(&style_btn_focused, 3);
}

// --- 警告弹窗动画回调 ---
static lv_obj_t * limit_toast_obj = NULL;

static void toast_anim_cb(void * var, int32_t v) {
    lv_obj_set_style_opa((lv_obj_t *)var, v, 0); // 这里的0是lv_style_selector_t
}
static void toast_anim_ready_cb(lv_anim_t * a) {
    lv_obj_del((lv_obj_t *)a->var);
    if ((lv_obj_t *)a->var == limit_toast_obj) {
        limit_toast_obj = NULL; // 动画播完销毁并清空指针
    }
}

static void show_limit_warning(void) {
    // 避免重复弹出
    if (limit_toast_obj != NULL) return;

    limit_toast_obj = lv_label_create(lv_layer_top());

    // 注：若默认字体未开启中文字库，中文会乱码。这里暂时使用英文替代"不可超出限度"。如果有中文字库可用，你可以将其改回 "不可超出限度"
    lv_label_set_text(limit_toast_obj, "Limit Exceeded!");

    // 红色背景框样式
    lv_obj_set_style_bg_color(limit_toast_obj, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_bg_opa(limit_toast_obj, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(limit_toast_obj, lv_color_white(), 0);
    lv_obj_set_style_pad_all(limit_toast_obj, 10, 0);
    lv_obj_set_style_radius(limit_toast_obj, 8, 0);

    // 在屏幕顶部居中
    lv_obj_align(limit_toast_obj, LV_ALIGN_TOP_MID, 0, 10);

    // 渐隐飞出动画
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, limit_toast_obj);
    lv_anim_set_time(&a, 300);      // 动画持续 300ms
    lv_anim_set_delay(&a, 600);     // 延时 600ms 后开始隐没
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_exec_cb(&a, toast_anim_cb);
    lv_anim_set_ready_cb(&a, toast_anim_ready_cb);
    lv_anim_start(&a);
}

static void show_save_success_toast(void) {
    lv_obj_t * toast = lv_label_create(lv_layer_top());
    lv_label_set_text(toast, "Saved successfully!");

    // 绿色背景框样式
    lv_obj_set_style_bg_color(toast, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_bg_opa(toast, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(toast, lv_color_white(), 0);
    lv_obj_set_style_pad_all(toast, 10, 0);
    lv_obj_set_style_radius(toast, 8, 0);

    // 在屏幕中央
    lv_obj_align(toast, LV_ALIGN_CENTER, 0, 0);

    // 渐隐飞出动画
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, toast);
    lv_anim_set_time(&a, 300);      // 动画持续 300ms
    lv_anim_set_delay(&a, 800);     // 延时 800ms 后开始隐没
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_exec_cb(&a, toast_anim_cb);
    lv_anim_set_ready_cb(&a, toast_anim_ready_cb); // 可以复用这个清理回调
    lv_anim_start(&a);
}

extern int last_sd_error_step;
extern FRESULT last_sd_error_code;

static void show_save_failed_toast(void) {
    lv_obj_t * toast = lv_label_create(lv_layer_top());

    if (last_sd_error_step == 1) {
        lv_label_set_text_fmt(toast, "Mount Fail: %d", last_sd_error_code);
    } else if (last_sd_error_step == 2) {
        lv_label_set_text_fmt(toast, "Open Fail: %d", last_sd_error_code);
    } else {
        lv_label_set_text(toast, "Save failed!");
    }

    // 红色背景框样式
    lv_obj_set_style_bg_color(toast, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_bg_opa(toast, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(toast, lv_color_white(), 0);
    lv_obj_set_style_pad_all(toast, 10, 0);
    lv_obj_set_style_radius(toast, 8, 0);

    // 在屏幕中央对齐
    lv_obj_align(toast, LV_ALIGN_CENTER, 0, 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, toast);
    lv_anim_set_time(&a, 300);
    lv_anim_set_delay(&a, 800);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_exec_cb(&a, toast_anim_cb);
    lv_anim_set_ready_cb(&a, toast_anim_ready_cb);
    lv_anim_start(&a);
}

static void btn_edit_confirm_cb(lv_event_t * e) {
    if (current_category == 0) Pitch[current_index] = temp_val;
    else Yaw[current_index] = temp_val;

    update_list_labels(); // 更新列表页面的 label

    // 将改变后的数值结构存入 SD 卡文件并提示
    bool ok = Save_Params_To_SD(Pitch, Yaw);
    if (ok) {
        show_save_success_toast();
    } else {
        show_save_failed_toast();
    }

    // 隐藏编辑菜单，返回列表菜单
    lv_obj_add_flag(view_edit, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(view_list, LV_OBJ_FLAG_HIDDEN);

    is_editing_val = false; // Exit edit mode

    // 切换焦点组回列表组
    lv_indev_set_group(keypad_indev, g_list);
    if (btn_list_first) lv_group_focus_obj(btn_list_first); // 手动确认焦点落点
}


static void btn_step_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_KEY) {
        int idx = (int)(intptr_t)lv_event_get_user_data(e);
        float step = steps[idx];
        uint32_t key = lv_event_get_key(e);

        float next_val = temp_val;

        if (key == LV_KEY_UP) { // 增
            next_val += step;
        } else if (key == LV_KEY_DOWN) { // 减
            next_val -= step;
        }

        // 限制 Yaw 的数值范围
        if (current_category == 1) { // 1 代表 YAW
            if (next_val > 52.60f) {
                next_val = 52.60f;
                show_limit_warning();
            } else if (next_val < 35.50f) {
                next_val = 35.50f;
                show_limit_warning();
            }
        }

        if (temp_val != next_val) {
            temp_val = next_val;
            update_edit_label_value(temp_val);
        }
    }
}

// === 分离界面的构建函数 ===

static void create_view_main(lv_obj_t * parent) {
    g_main = lv_group_create();

    view_main = lv_obj_create(parent);
    lv_obj_set_size(view_main, LV_PCT(100), LV_PCT(100)); // 占满全屏
    lv_obj_clear_flag(view_main, LV_OBJ_FLAG_SCROLLABLE);

    // PITCH 按钮 (左边)
    btn_pitch = lv_btn_create(view_main);
    lv_obj_add_style(btn_pitch, &style_btn_focused, LV_STATE_FOCUSED); // 添加焦点高亮
    lv_obj_set_size(btn_pitch, 150, 80);
    lv_obj_align(btn_pitch, LV_ALIGN_LEFT_MID, 50, 0);
    lv_obj_t * lbl_pitch = lv_label_create(btn_pitch);
    lv_label_set_text(lbl_pitch, "PITCH");
    lv_obj_center(lbl_pitch);
    lv_obj_add_event_cb(btn_pitch, btn_main_cb, LV_EVENT_CLICKED, (void*)(intptr_t)0); // 传入 0
    lv_group_add_obj(g_main, btn_pitch); // 加入主界面的焦点组

    // YAW 按钮 (右边)
    lv_obj_t * btn_yaw = lv_btn_create(view_main);
    lv_obj_add_style(btn_yaw, &style_btn_focused, LV_STATE_FOCUSED); // 添加焦点高亮
    lv_obj_set_size(btn_yaw, 150, 80);
    lv_obj_align(btn_yaw, LV_ALIGN_RIGHT_MID, -50, 0);
    lv_obj_t * lbl_yaw = lv_label_create(btn_yaw);
    lv_label_set_text(lbl_yaw, "YAW");
    lv_obj_center(lbl_yaw);
    lv_obj_add_event_cb(btn_yaw, btn_main_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1); // 传入 1
    lv_group_add_obj(g_main, btn_yaw); // 加入主界面的焦点组
}

static void create_view_list(lv_obj_t * parent) {
    g_list = lv_group_create();

    view_list = lv_obj_create(parent);
    lv_obj_set_size(view_list, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(view_list, LV_OBJ_FLAG_HIDDEN); // 初始时隐藏
    lv_obj_clear_flag(view_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(view_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(view_list, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // 用一个透明行容器装下 1234 四个按钮，使其水平排列
    lv_obj_t * list_container = lv_obj_create(view_list);
    lv_obj_set_size(list_container, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(list_container, 0, 0);
    lv_obj_set_style_border_opa(list_container, 0, 0);
    lv_obj_set_flex_flow(list_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(list_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // 循环创建 1,2,3,4 按钮
    for(int i = 0; i < 4; i++) {
        lv_obj_t * btn = lv_btn_create(list_container);
        if (i == 0) btn_list_first = btn; // 记录列表第一个按钮用于跳转焦点
        lv_obj_add_style(btn, &style_btn_focused, LV_STATE_FOCUSED); // 添加焦点高亮
        lv_obj_set_size(btn, 70, 70);
        label_list_btns[i] = lv_label_create(btn);
        lv_label_set_text_fmt(label_list_btns[i], "%d\n0.00", i + 1);
        lv_obj_center(label_list_btns[i]);
        lv_obj_add_event_cb(btn, btn_list_to_edit_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i); // 传入序号 0,1,2,3
        lv_group_add_obj(g_list, btn); // 加入列表界面的焦点组
    }

    // 第二层返回按钮
    lv_obj_t * btn_list_ret = lv_btn_create(view_list);
    lv_obj_add_style(btn_list_ret, &style_btn_focused, LV_STATE_FOCUSED); // 添加焦点高亮
    lv_obj_set_size(btn_list_ret, 120, 60);
    lv_obj_t * lbl_list_ret = lv_label_create(btn_list_ret);
    lv_label_set_text(lbl_list_ret, "RETURN");
    lv_obj_center(lbl_list_ret);
    lv_obj_add_event_cb(btn_list_ret, btn_list_return_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g_list, btn_list_ret); // 加入列表界面的焦点组
}

static void create_view_edit(lv_obj_t * parent) {
    g_edit = lv_group_create();

    view_edit = lv_obj_create(parent);
    lv_obj_set_size(view_edit, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(view_edit, LV_OBJ_FLAG_HIDDEN); // 初始隐藏
    lv_obj_clear_flag(view_edit, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(view_edit, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(view_edit, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // 数值显示(大框)
    lv_obj_t * btn_val = lv_btn_create(view_edit);
    lv_obj_add_style(btn_val, &style_btn_focused, LV_STATE_FOCUSED);
    lv_obj_set_size(btn_val, 160, 60);
    label_edit_val = lv_label_create(btn_val);
    lv_label_set_text(label_edit_val, "0.00");
    lv_obj_center(label_edit_val);

    // 步长选择容器 (套一层透明 flex 布局实现水平摆放4个按钮)
    lv_obj_t * step_container = lv_obj_create(view_edit);
    lv_obj_set_size(step_container, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(step_container, 0, 0);
    lv_obj_set_style_border_opa(step_container, 0, 0);
    lv_obj_set_flex_flow(step_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(step_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    const char* step_labels[4] = {"10", "1", "0.1", "0.01"};
    for(int i = 0; i < 4; i++) {
        lv_obj_t * btn = lv_btn_create(step_container);
        if (i == 0) btn_edit_first = btn;
        lv_obj_add_style(btn, &style_btn_focused, LV_STATE_FOCUSED); // 添加焦点高亮
        lv_obj_set_size(btn, 65, 50);
        lv_obj_t * lbl = lv_label_create(btn);
        lv_label_set_text(lbl, step_labels[i]);
        lv_obj_center(lbl);
        // 这里拦截 UP 和 DOWN key 等事件，使得在焦点停留在此按钮时可以更改数值
        lv_obj_add_event_cb(btn, btn_step_cb, LV_EVENT_KEY, (void*)(intptr_t)i);
        lv_group_add_obj(g_edit, btn);
    }

    // 确认返回按钮(底部)
    lv_obj_t * btn_ok = lv_btn_create(view_edit);
    lv_obj_add_style(btn_ok, &style_btn_focused, LV_STATE_FOCUSED); // 添加焦点高亮
    lv_obj_set_size(btn_ok, 120, 50);
    lv_obj_t * lbl_ok = lv_label_create(btn_ok);
    lv_label_set_text(lbl_ok, "OK");
    lv_obj_center(lbl_ok);
    lv_obj_add_event_cb(btn_ok, btn_edit_confirm_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g_edit, btn_ok);
}

// === 构建界面的主函数 ===
void demo_multi_level(void) {
    lv_obj_t * scr = lv_scr_act(); // 获取当前活跃屏幕

    style_init(); // 初始化按键焦点样式

    /* ======================== 界 面 1: 主 菜 单 ======================== */
    create_view_main(scr);

    /* ======================== 界 面 2: 列 表 选 项 ======================== */
    create_view_list(scr);

    /* ======================== 界 面 3: 独 立 编 辑 ======================== */
    create_view_edit(scr);

    // 默认开机赋给键盘主界面的控制组，并选中第一个 PITCH
    lv_indev_set_group(keypad_indev, g_main);
    lv_group_focus_obj(btn_pitch);
}

extern "C" void init_lvgl_demo(void)
{
  /*Initialize LVGL*/
  lv_init();

  /* 尝试从 SD 卡加载上次保存的 Pitch 和 Yaw 数组；如果不存在则维持默认值 */
  Load_Params_From_SD(Pitch, Yaw);

  /* 关闭 LVGL 默认的滚动和状态切换动画，提升响应速度 */
  lv_disp_set_theme(NULL, lv_theme_default_init(NULL, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT));

  /*Initialize the HAL (display, input devices, tick) for LVGL*/
  hal_init();

  // 运行多层菜单逻辑
  demo_multi_level();

  // 在初始化完成并且加载了SD卡参数后，主动更新一次列表页面的标签显示
  update_list_labels();
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

extern "C" void my_disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

// === 按键非阻塞防抖处理 ===
static uint32_t last_key_time = 0;
static uint32_t current_key = 0;
static lv_indev_state_t current_state = LV_INDEV_STATE_REL;

// 提供给外部中断 `HAL_GPIO_EXTI_Callback` 调用的触发接收器
extern "C" void lvgl_button_exti_trigger(uint16_t GPIO_Pin) {
    uint32_t now = HAL_GetTick();
    if (now - last_key_time < 30) return; // 防抖时间缩短到 30ms，使连按响应更灵敏

    if (GPIO_Pin == GPIO_PIN_4) {        // LEFT
        current_key = LV_KEY_PREV;  // 改为前一个，更符合组内焦点的左移逻辑
    } else if (GPIO_Pin == GPIO_PIN_10) { // RIGHT
        current_key = LV_KEY_NEXT;  // 改为后一个，更符合组内焦点的右移逻辑
    } else if (GPIO_Pin == GPIO_PIN_9) {  // ENSURE
        current_key = LV_KEY_ENTER;
    } else if (GPIO_Pin == GPIO_PIN_5) {  // PLUS
        current_key = LV_KEY_UP;    // PLUS 映射为 UP
    } else if (GPIO_Pin == GPIO_PIN_1) {  // MINUS
        current_key = LV_KEY_DOWN;  // MINUS 映射为 DOWN
    } else {
        return; // 不相关的管脚不处理
    }

    last_key_time = now;
    current_state = LV_INDEV_STATE_PR; // 登记按下
}

// 绑定给LVGL不断轮询读取按键状态的神奇非阻塞回调接口
static void my_keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {
    data->key = current_key;
    data->state = current_state;

    // 当LVGL判定目前为按下状态时，我们需要通过非阻塞读取引脚是否恢复高电平
    // 如果采用外部中断捕获下降沿，其不能捕获电平恢复事件，由此机制作为补偿
    if (current_state == LV_INDEV_STATE_PR && (HAL_GetTick() - last_key_time > 40)) { // 检测松开时间缩短到 40ms
        bool is_released = false;

        // 分别轮询判断这几个按键引脚此刻是否恢复常态高电平
        if (current_key == LV_KEY_PREV  && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)  == GPIO_PIN_SET) is_released = true;
        if (current_key == LV_KEY_NEXT  && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10) == GPIO_PIN_SET) is_released = true;
        if (current_key == LV_KEY_ENTER && HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_9)  == GPIO_PIN_SET) is_released = true;
        if (current_key == LV_KEY_UP    && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)  == GPIO_PIN_SET) is_released = true;
        if (current_key == LV_KEY_DOWN  && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)  == GPIO_PIN_SET) is_released = true;

        if (is_released) {
            current_state = LV_INDEV_STATE_REL; // 登记为松开
        }
    }
}

static void hal_init(void)
{
  /* 使用你自己的屏幕分辨率，请根据实际屏幕自行修改 */

  /*Create a display buffer*/
  static lv_disp_draw_buf_t disp_buf;
  static lv_color_t buf[W * 20];
  lv_disp_draw_buf_init(&disp_buf, buf, NULL, W * 20);

  /*Create a display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
  disp_drv.draw_buf = &disp_buf;
  disp_drv.flush_cb = my_disp_flush;      /*Set your driver function*/
  disp_drv.hor_res = W;
  disp_drv.ver_res = H;
  lv_disp_drv_register(&disp_drv);

  /* 注册物理按键输入设备 */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_KEYPAD; // 定义输入为普通按键组
  indev_drv.read_cb = my_keypad_read;    // 接入非阻塞查询防抖回调

  /* 关键：显著缩短 LVGL 内部的长按及连续触发周期判定，提升调节体验 */
  indev_drv.long_press_time = 300;       // 按下 300ms 后算长按（原来默认通常是400ms）
  indev_drv.long_press_repeat_time = 50; // 长按时，每 50ms 触发一次加减（原来默认通常是100ms）

  keypad_indev = lv_indev_drv_register(&indev_drv);
}
