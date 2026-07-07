/**
 * 显示驱动 —— LVGL UI、双缓冲预览、按键状态管理
 *
 * 双缓冲架构:
 *   g_preview_buf  — 前端缓冲区 (LVGL 读取, MIPI-DSI 送显)
 *   g_render_buf   — 后端缓冲区 (主循环渲染, 弹珠绘制)
 *   display_refresh_preview() — 原子交换前后端缓冲区
 *
 * 封装所有显示相关逻辑:
 *   - LVGL 界面构建 (标题、预览图像、状态标签、按钮)
 *   - 预览缓冲分配与 ImageTransformer 缩放 (SIMD 加速)
 *   - 检测框绘制 (ESP-DL draw_hollow_rectangle)
 *   - 按钮回调与状态标志
 *   - 线程安全的 BSP 显示锁
 */

#include <stdio.h>
#include <string.h>
#include <vector>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "dl_image_process.hpp"
#include "dl_image_draw.hpp"
#include "display_driver.h"
#include "config.h"
#include "physics/marble_physics.h"
#include "tasks/power_mgmt_task.h"

static const char *TAG = "display";

/* 预览尺寸: 保持 5:4 宽高比, 宽度 640 时高度为 512 */
#define PREVIEW_W  640
#define PREVIEW_H  512

/* ---- 内部状态 ---- */
static lv_obj_t  *g_status_label = NULL;
static lv_obj_t  *g_preview_img  = NULL;

/* 双缓冲: front=LVGL读取, back=主循环写入 */
static uint8_t       *g_preview_buf = NULL;   // 前端缓冲区
static uint8_t       *g_render_buf  = NULL;   // 后端缓冲区
static lv_image_dsc_t g_preview_dsc;
static lv_image_dsc_t g_render_dsc;

static volatile bool g_trigger_detect  = false;
static volatile bool g_live_view       = false;
static volatile bool g_edge_view       = false;
static volatile bool g_track_capture   = false;

/* ---- Game mode state ---- */
static volatile bool g_game_capture    = false;
static volatile bool g_game_exit       = false;
static bool          g_game_active     = false;

/* Game double buffers (640x640 RGB565, PSRAM) */
#define GAME_BUF_W  640
#define GAME_BUF_H  640
static uint8_t       *g_game_front_buf  = NULL;
static uint8_t       *g_game_back_buf   = NULL;
static lv_image_dsc_t g_game_front_dsc;
static lv_image_dsc_t g_game_back_dsc;

/* Game mode LVGL widgets (created in init_game_mode, deleted in exit) */
static lv_obj_t *g_game_top_bar    = NULL;
static lv_obj_t *g_game_top_status = NULL;
static lv_obj_t *g_game_img       = NULL;
static lv_obj_t *g_game_panel     = NULL;

/* HUD labels in right panel */
static lv_obj_t *g_hud_pos        = NULL;
static lv_obj_t *g_hud_speed      = NULL;
static lv_obj_t *g_hud_tilt       = NULL;
static lv_obj_t *g_hud_bounce     = NULL;
static lv_obj_t *g_hud_wallpass   = NULL;
static lv_obj_t *g_hud_objects    = NULL;
static lv_obj_t *g_hud_end_msg    = NULL;  /* win/lose overlay on game screen */

/* Saved references to original UI widgets (for hide/restore) */
static lv_obj_t *g_orig_scr       = NULL;
static lv_obj_t *g_orig_buttons[5] = {NULL};
static lv_obj_t *g_orig_title     = NULL;

/* ---- RGB565→Gray LUT (for edge preview grayscale conversion) ---- */
static uint8_t r_lut[32];
static uint8_t g_lut[64];
static uint8_t b_lut[32];

static void init_gray_luts(void)
{
    for (int i = 0; i < 32; i++) {
        uint8_t r8 = (i * 255 + 15) / 31;
        r_lut[i] = (uint8_t)(((uint16_t)r8 * 77) >> 8);
    }
    for (int i = 0; i < 64; i++) {
        uint8_t g8 = (i * 255 + 31) / 63;
        g_lut[i] = (uint8_t)(((uint16_t)g8 * 150) >> 8);
    }
    for (int i = 0; i < 32; i++) {
        uint8_t b8 = (i * 255 + 15) / 31;
        b_lut[i] = (uint8_t)(((uint16_t)b8 * 29) >> 8);
    }
}

/* 预处理运行时状态 */
static const uint32_t g_preproc_presets[] = {
    PREPROC_PRESET_0,   /* OFF       */
    PREPROC_PRESET_1,   /* Gamma     */
    PREPROC_PRESET_2,   /* Gamma+DN  */
    PREPROC_PRESET_3,   /* DN        */
    PREPROC_PRESET_4,   /* HE+DN     */
    PREPROC_PRESET_5,   /* ALL       */
};

static const char *g_preproc_names[] = {
    "OFF",
    "Gamma",
    "Ga+DN",
    "DN",
    "HE+DN",
    "ALL",
};

static int        g_preproc_idx   = 0;  /* 当前预设索引 */
static lv_obj_t  *g_btn_preproc_label = NULL;

/* ---- 缩放: 800x640 → PREVIEW_W×PREVIEW_H (ImageTransformer, NN) ---- */
static void scale_frame(const camera_frame_t *frame, uint8_t *dst_buf)
{
    dl::image::img_t src;
    src.data     = frame->buffer;
    src.width    = frame->width;
    src.height   = frame->height;
    src.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

    dl::image::img_t dst;
    dst.data     = dst_buf;
    dst.width    = PREVIEW_W;
    dst.height   = PREVIEW_H;
    dst.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

    dl::image::ImageTransformer()
        .set_src_img(src)
        .set_dst_img(dst)
        .transform();
}

/* ---- 在渲染缓冲上绘制检测框 ---- */
static void draw_boxes_on_preview(const detection_result_t *detections, int det_count)
{
    if (!detections || det_count <= 0) return;

    dl::image::img_t preview_img;
    preview_img.data     = g_render_buf;
    preview_img.width    = PREVIEW_W;
    preview_img.height   = PREVIEW_H;
    preview_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

    /* 相机坐标 → 预览坐标 */
    const float scale_x = (float)PREVIEW_W / (float)CAMERA_H_RES;
    const float scale_y = (float)PREVIEW_H / (float)CAMERA_V_RES;

    /* RGB565 颜色定义 (LE 字节序): 绿色框 */
    std::vector<uint8_t> green = {0xE0, 0x07};  /* 0x07E0 = pure green */
    std::vector<uint8_t> red   = {0x00, 0xF8};  /* 0xF800 = pure red   */

    for (int i = 0; i < det_count; i++) {
        int px1 = (int)(detections[i].box_camera[0] * scale_x);
        int py1 = (int)(detections[i].box_camera[1] * scale_y);
        int px2 = (int)(detections[i].box_camera[2] * scale_x);
        int py2 = (int)(detections[i].box_camera[3] * scale_y);

        /* 裁剪到预览范围 */
        if (px1 < 0) px1 = 0;
        if (py1 < 0) py1 = 0;
        if (px2 >= PREVIEW_W) px2 = PREVIEW_W - 1;
        if (py2 >= PREVIEW_H) py2 = PREVIEW_H - 1;

        /* 跳过退化框 */
        if (px2 <= px1 || py2 <= py1) continue;

        /* 用高置信度颜色 (score >= 0.5 用绿色, 否则红色) */
        const std::vector<uint8_t> &color = (detections[i].score >= 0.5f) ? green : red;

        dl::image::draw_hollow_rectangle(preview_img,
                                         px1, py1, px2, py2,
                                         color, 2);
    }
}

/* ---- 刷新 LVGL 预览图像 (内部, 假定已在 BSP 锁内) ---- */
static void update_preview_display(void)
{
    if (!g_preview_img) return;
    lv_image_set_src(g_preview_img, &g_preview_dsc);
    lv_obj_invalidate(g_preview_img);
}

/* ---- 按键回调 ---- */
static void btn_preview_callback(lv_event_t *e)
{
    (void)e;
    notify_user_activity();
    g_live_view = !g_live_view;
    if (g_live_view) g_edge_view = false;
}

static void btn_edge_callback(lv_event_t *e)
{
    (void)e;
    notify_user_activity();
    g_edge_view = !g_edge_view;
    if (g_edge_view) g_live_view = false;
}

static void btn_detect_callback(lv_event_t *e)
{
    (void)e;
    notify_user_activity();
    g_trigger_detect = true;
}

static void btn_preproc_callback(lv_event_t *e)
{
    (void)e;
    notify_user_activity();
    g_preproc_idx = (g_preproc_idx + 1) % PREPROC_PRESET_COUNT;

    /* 更新按钮标签显示当前模式 */
    if (g_btn_preproc_label) {
        lv_label_set_text(g_btn_preproc_label, g_preproc_names[g_preproc_idx]);
    }
}

static void btn_game_callback(lv_event_t *e)
{
    (void)e;
    notify_user_activity();
    if (g_game_active) {
        g_game_exit = true;
    } else {
        g_game_capture = true;
    }
}

/* 全局触摸回调: 屏幕任意位置触摸即唤醒 (省电模式恢复) */
static void screen_touch_callback(lv_event_t *e)
{
    (void)e;
    notify_user_activity();
    pm_resume_all();
}

/* ========================================================================
 * 公开接口
 * ======================================================================== */

esp_err_t display_init(void)
{
    ESP_LOGI(TAG, "Initializing display and LVGL (double-buffered)...");

    /* 1. 启动显示 */
    lv_display_t *disp = bsp_display_start();
    if (!disp) {
        ESP_LOGE(TAG, "Display init FAILED");
        return ESP_FAIL;
    }
    bsp_display_backlight_on();
    ESP_LOGI(TAG, "Display OK: %dx%d", BSP_LCD_H_RES, BSP_LCD_V_RES);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 2. 分配双缓冲 (PSRAM) */
    size_t buf_size = PREVIEW_W * PREVIEW_H * 2;  // 640*512*2 = 655,360 bytes

    g_preview_buf = (uint8_t *)heap_caps_calloc(1, buf_size,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_render_buf  = (uint8_t *)heap_caps_calloc(1, buf_size,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_preview_buf || !g_render_buf) {
        ESP_LOGE(TAG, "Failed to allocate preview buffers (dual)");
        if (g_preview_buf) heap_caps_free(g_preview_buf);
        if (g_render_buf)  heap_caps_free(g_render_buf);
        g_preview_buf = NULL;
        g_render_buf  = NULL;
        return ESP_ERR_NO_MEM;
    }
    memset(g_preview_buf, 0x00, buf_size);
    memset(g_render_buf,  0x00, buf_size);

    /* 前端描述符 (LVGL 读取) */
    g_preview_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    g_preview_dsc.header.w      = PREVIEW_W;
    g_preview_dsc.header.h      = PREVIEW_H;
    g_preview_dsc.header.stride = PREVIEW_W * 2;
    g_preview_dsc.data          = g_preview_buf;
    g_preview_dsc.data_size     = buf_size;

    /* 后端描述符 (主循环写入) */
    g_render_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    g_render_dsc.header.w      = PREVIEW_W;
    g_render_dsc.header.h      = PREVIEW_H;
    g_render_dsc.header.stride = PREVIEW_W * 2;
    g_render_dsc.data          = g_render_buf;
    g_render_dsc.data_size     = buf_size;

    ESP_LOGI(TAG, "Dual buffers: %zu bytes each (PSRAM)", buf_size);

    /* 3. 构建 UI */
    bsp_display_lock(portMAX_DELAY);

    /* 初始化灰度 LUT */
    init_gray_luts();

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101010), 0);

    /* Save original screen */
    g_orig_scr = scr;

    /* 全局触摸唤醒: 屏幕任意位置触摸 → 退出省电模式 */
    lv_obj_add_event_cb(scr, screen_touch_callback, LV_EVENT_PRESSED, NULL);

    /* 标题 */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Camera Preview | ESP-DL Detection");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
    g_orig_title = title;

    /* 预览图像 (640x512 RGB565, 居中, 指向前端缓冲区) */
    g_preview_img = lv_image_create(scr);
    lv_image_set_src(g_preview_img, &g_preview_dsc);
    lv_obj_align(g_preview_img, LV_ALIGN_CENTER, 0, -10);

    /* 状态标签 (在预览下方) */
    g_status_label = lv_label_create(scr);
    lv_label_set_text(g_status_label, "Status: Init camera...");
    lv_obj_set_style_text_color(g_status_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(g_status_label, &lv_font_montserrat_14, 0);
    lv_obj_align(g_status_label, LV_ALIGN_BOTTOM_MID, 0, -55);

    /* 查找初始预设索引 (匹配 PREPROC_DEFAULT_FLAGS) */
    for (int i = 0; i < PREPROC_PRESET_COUNT; i++) {
        if (g_preproc_presets[i] == PREPROC_DEFAULT_FLAGS) {
            g_preproc_idx = i;
            break;
        }
    }

    /* 按钮 (5 个, 底部居中排列) */
    lv_obj_t *btn_preview = lv_button_create(scr);
    lv_obj_set_size(btn_preview, 80, 40);
    lv_obj_align(btn_preview, LV_ALIGN_BOTTOM_MID, -180, -5);
    lv_obj_add_event_cb(btn_preview, btn_preview_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_preview_label = lv_label_create(btn_preview);
    lv_label_set_text(btn_preview_label, "Live");
    lv_obj_center(btn_preview_label);
    g_orig_buttons[0] = btn_preview;

    lv_obj_t *btn_edge = lv_button_create(scr);
    lv_obj_set_size(btn_edge, 80, 40);
    lv_obj_align(btn_edge, LV_ALIGN_BOTTOM_MID, -90, -5);
    lv_obj_add_event_cb(btn_edge, btn_edge_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_edge_label = lv_label_create(btn_edge);
    lv_label_set_text(btn_edge_label, "Edge");
    lv_obj_center(btn_edge_label);
    g_orig_buttons[1] = btn_edge;

    lv_obj_t *btn_game = lv_button_create(scr);
    lv_obj_set_size(btn_game, 80, 40);
    lv_obj_align(btn_game, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_add_event_cb(btn_game, btn_game_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_game_label = lv_label_create(btn_game);
    lv_label_set_text(btn_game_label, "Game");
    lv_obj_center(btn_game_label);
    g_orig_buttons[2] = btn_game;

    lv_obj_t *btn_preproc = lv_button_create(scr);
    lv_obj_set_size(btn_preproc, 80, 40);
    lv_obj_align(btn_preproc, LV_ALIGN_BOTTOM_MID, 90, -5);
    lv_obj_add_event_cb(btn_preproc, btn_preproc_callback, LV_EVENT_CLICKED, NULL);
    g_btn_preproc_label = lv_label_create(btn_preproc);
    lv_label_set_text(g_btn_preproc_label, g_preproc_names[g_preproc_idx]);
    lv_obj_center(g_btn_preproc_label);
    g_orig_buttons[3] = btn_preproc;

    lv_obj_t *btn_detect = lv_button_create(scr);
    lv_obj_set_size(btn_detect, 80, 40);
    lv_obj_align(btn_detect, LV_ALIGN_BOTTOM_MID, 180, -5);
    lv_obj_add_event_cb(btn_detect, btn_detect_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_detect_label = lv_label_create(btn_detect);
    lv_label_set_text(btn_detect_label, "Detect");
    lv_obj_center(btn_detect_label);
    g_orig_buttons[4] = btn_detect;

    bsp_display_unlock();

    ESP_LOGI(TAG, "Display UI ready (double-buffered)");
    return ESP_OK;
}

esp_err_t display_update_preview(const camera_frame_t *frame,
                                  const detection_result_t *detections,
                                  int det_count)
{
    esp_err_t ret = display_prepare_preview(frame, detections, det_count);
    if (ret != ESP_OK) return ret;
    display_refresh_preview();
    return ESP_OK;
}

void display_set_status(const char *text, uint32_t color)
{
    if (!g_status_label) return;

    bsp_display_lock(portMAX_DELAY);
    lv_label_set_text(g_status_label, text ? text : "");
    lv_obj_set_style_text_color(g_status_label, lv_color_hex(color), 0);
    bsp_display_unlock();
}

bool display_is_live_view(void)
{
    return g_live_view;
}

bool display_detect_triggered(void)
{
    if (g_trigger_detect) {
        g_trigger_detect = false;
        return true;
    }
    return false;
}

uint32_t display_get_preproc_flags(void)
{
    return g_preproc_presets[g_preproc_idx];
}

bool display_is_edge_view(void)
{
    return g_edge_view;
}

bool display_track_capture_triggered(void)
{
    if (g_track_capture) {
        g_track_capture = false;
        return true;
    }
    return false;
}

esp_err_t display_update_edge_preview(const camera_frame_t *frame,
                                       const uint8_t *edge_map,
                                       int ew, int eh)
{
    if (!frame || !frame->buffer || !g_render_buf) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 1. 缩放相机帧到渲染缓冲 (800x640 → 640x512) */
    scale_frame(frame, g_render_buf);

    /* 2. 渲染缓冲转换为灰度 RGB565 */
    uint16_t *pixels = (uint16_t *)g_render_buf;
    int total = PREVIEW_W * PREVIEW_H;
    for (int i = 0; i < total; i++) {
        uint16_t p = pixels[i];
        uint8_t r5 = (p >> 11) & 0x1F;
        uint8_t g6 = (p >> 5)  & 0x3F;
        uint8_t b5 =  p        & 0x1F;
        uint8_t y = r_lut[r5] + g_lut[g6] + b_lut[b5];
        /* 重新打包为灰度 RGB565 */
        pixels[i] = (uint16_t)(((y >> 3) << 11) | ((y >> 2) << 5) | (y >> 3));
    }

    /* 3. 叠加彩色边缘线条 (最近邻上采样 edge_map → 预览尺寸) */
    if (edge_map && ew > 0 && eh > 0) {
        uint32_t x_scale = ((uint32_t)ew << 16) / PREVIEW_W;
        uint32_t y_scale = ((uint32_t)eh << 16) / PREVIEW_H;

        for (int py = 0; py < PREVIEW_H; py++) {
            int ey = (py * y_scale) >> 16;
            if (ey >= eh) ey = eh - 1;

            const uint8_t *edge_row = edge_map + ey * ew;

            for (int px = 0; px < PREVIEW_W; px++) {
                int ex = (px * x_scale) >> 16;
                if (ex >= ew) ex = ew - 1;

                if (edge_row[ex] == 255) {
                    pixels[py * PREVIEW_W + px] = EDGE_COLOR_RGB565;
                }
            }
        }
    }

    /* 不在此刷新 LVGL — 由调用者在 marble_draw 后统一 display_refresh_preview() */
    return ESP_OK;
}

uint8_t *display_get_preview_buf(int *w, int *h)
{
    if (w) *w = PREVIEW_W;
    if (h) *h = PREVIEW_H;
    return g_preview_buf;
}

uint8_t *display_get_render_buf(int *w, int *h)
{
    if (w) *w = PREVIEW_W;
    if (h) *h = PREVIEW_H;
    return g_render_buf;
}

esp_err_t display_prepare_preview(const camera_frame_t *frame,
                                   const detection_result_t *detections,
                                   int det_count)
{
    if (!frame || !frame->buffer || !g_render_buf) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 缩放相机帧到后端渲染缓冲 */
    scale_frame(frame, g_render_buf);

    /* 在后端缓冲上绘制检测框 */
    draw_boxes_on_preview(detections, det_count);

    return ESP_OK;
}

void display_refresh_preview(void)
{
    bsp_display_lock(portMAX_DELAY);

    /* 原子交换前后端缓冲区 */
    uint8_t *tmp_buf = g_preview_buf;
    g_preview_buf = g_render_buf;
    g_render_buf  = tmp_buf;

    /* 更新描述符中的数据指针 */
    g_preview_dsc.data = g_preview_buf;
    g_render_dsc.data  = g_render_buf;

    /* 通知 LVGL 重绘 (此时 g_preview_buf 是完整的) */
    update_preview_display();

    bsp_display_unlock();
}

/* ========================================================================
 * Game mode API
 * ======================================================================== */

bool display_game_capture_triggered(void)
{
    if (g_game_capture) {
        g_game_capture = false;
        return true;
    }
    return false;
}

bool display_game_exit_triggered(void)
{
    if (g_game_exit) {
        g_game_exit = false;
        return true;
    }
    return false;
}

esp_err_t display_init_game_mode(void)
{
    if (g_game_active) return ESP_OK;

    ESP_LOGI(TAG, "Entering game mode (portrait 720x1280)...");

    /* 1. Allocate 640x640 double buffers in PSRAM */
    size_t buf_size = GAME_BUF_W * GAME_BUF_H * 2;
    g_game_front_buf = (uint8_t *)heap_caps_calloc(1, buf_size,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_game_back_buf  = (uint8_t *)heap_caps_calloc(1, buf_size,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_game_front_buf || !g_game_back_buf) {
        ESP_LOGE(TAG, "Failed to allocate game buffers");
        if (g_game_front_buf) heap_caps_free(g_game_front_buf);
        if (g_game_back_buf)  heap_caps_free(g_game_back_buf);
        g_game_front_buf = NULL;
        g_game_back_buf  = NULL;
        return ESP_ERR_NO_MEM;
    }
    memset(g_game_front_buf, 0x00, buf_size);
    memset(g_game_back_buf,  0x00, buf_size);

    g_game_front_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    g_game_front_dsc.header.w      = GAME_BUF_W;
    g_game_front_dsc.header.h      = GAME_BUF_H;
    g_game_front_dsc.header.stride = GAME_BUF_W * 2;
    g_game_front_dsc.data          = g_game_front_buf;
    g_game_front_dsc.data_size     = buf_size;

    g_game_back_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    g_game_back_dsc.header.w      = GAME_BUF_W;
    g_game_back_dsc.header.h      = GAME_BUF_H;
    g_game_back_dsc.header.stride = GAME_BUF_W * 2;
    g_game_back_dsc.data          = g_game_back_buf;
    g_game_back_dsc.data_size     = buf_size;

    ESP_LOGI(TAG, "Game buffers: %zu bytes each (PSRAM)", buf_size);

    /* 2. Build game UI on a NEW screen (portrait 720x1280, no rotation) */
    bsp_display_lock(portMAX_DELAY);

    /* Create new screen — this will be the game screen */
    lv_obj_t *game_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(game_scr, lv_color_hex(0x1A1A2E), 0);

    /* 全局触摸唤醒: 游戏界面任意位置触摸 → 退出省电模式 */
    lv_obj_add_event_cb(game_scr, screen_touch_callback, LV_EVENT_PRESSED, NULL);

    /* -- Top bar (full width, 30px) -- */
    g_game_top_bar = lv_obj_create(game_scr);
    lv_obj_set_size(g_game_top_bar, 720, 30);
    lv_obj_align(g_game_top_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(g_game_top_bar, lv_color_hex(0x16213E), 0);
    lv_obj_set_style_border_width(g_game_top_bar, 0, 0);
    lv_obj_set_style_pad_all(g_game_top_bar, 0, 0);
    lv_obj_set_scrollbar_mode(g_game_top_bar, LV_SCROLLBAR_MODE_OFF);

    g_game_top_status = lv_label_create(g_game_top_bar);
    lv_label_set_text(g_game_top_status, "Game Mode");
    lv_obj_set_style_text_color(g_game_top_status, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(g_game_top_status, &lv_font_montserrat_14, 0);
    lv_obj_align(g_game_top_status, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_t *btn_exit = lv_button_create(g_game_top_bar);
    lv_obj_set_size(btn_exit, 60, 24);
    lv_obj_align(btn_exit, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(btn_exit, btn_game_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_exit_label = lv_label_create(btn_exit);
    lv_label_set_text(btn_exit_label, "Exit");
    lv_obj_center(btn_exit_label);

    /* -- Game image: 640x640, centered in top area, y offset from top bar -- */
    g_game_img = lv_image_create(game_scr);
    lv_image_set_src(g_game_img, &g_game_front_dsc);
    lv_obj_set_pos(g_game_img, 40, 32);

    /* -- HUD panel below game image (y=32+640=672, height=~608) -- */
    g_game_panel = lv_obj_create(game_scr);
    lv_obj_set_size(g_game_panel, 720, 580);
    lv_obj_set_pos(g_game_panel, 0, 676);
    lv_obj_set_style_bg_color(g_game_panel, lv_color_hex(0x0F3460), 0);
    lv_obj_set_style_border_width(g_game_panel, 1, 0);
    lv_obj_set_style_border_color(g_game_panel, lv_color_hex(0x533483), 0);
    lv_obj_set_style_pad_all(g_game_panel, 8, 0);
    lv_obj_set_scrollbar_mode(g_game_panel, LV_SCROLLBAR_MODE_OFF);

    /* HUD: use a flex layout to arrange items compactly */
    /* Row 1: position + speed */
    g_hud_pos = lv_label_create(g_game_panel);
    lv_label_set_text(g_hud_pos, "Marble X:---  Y:---");
    lv_obj_set_style_text_color(g_hud_pos, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(g_hud_pos, &lv_font_montserrat_14, 0);
    lv_obj_align(g_hud_pos, LV_ALIGN_TOP_LEFT, 4, 4);

    g_hud_speed = lv_label_create(g_game_panel);
    lv_label_set_text(g_hud_speed, "Speed: --- px/s");
    lv_obj_set_style_text_color(g_hud_speed, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(g_hud_speed, &lv_font_montserrat_14, 0);
    lv_obj_align_to(g_hud_speed, g_hud_pos, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    /* Row 2: tilt */
    g_hud_tilt = lv_label_create(g_game_panel);
    lv_label_set_text(g_hud_tilt, "Tilt: X ---  Y ---");
    lv_obj_set_style_text_color(g_hud_tilt, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(g_hud_tilt, &lv_font_montserrat_14, 0);
    lv_obj_align_to(g_hud_tilt, g_hud_speed, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    /* Row 3: bounce */
    g_hud_bounce = lv_label_create(g_game_panel);
    lv_label_set_text(g_hud_bounce, "Bounce: ---");
    lv_obj_set_style_text_color(g_hud_bounce, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(g_hud_bounce, &lv_font_montserrat_14, 0);
    lv_obj_align_to(g_hud_bounce, g_hud_tilt, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    /* Row 4: wall-pass */
    g_hud_wallpass = lv_label_create(g_game_panel);
    lv_label_set_text(g_hud_wallpass, "Wall-pass: OFF");
    lv_obj_set_style_text_color(g_hud_wallpass, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(g_hud_wallpass, &lv_font_montserrat_14, 0);
    lv_obj_align_to(g_hud_wallpass, g_hud_bounce, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    /* Row 5: objects header */
    lv_obj_t *obj_header = lv_label_create(g_game_panel);
    lv_label_set_text(obj_header, "-- Scene Objects --");
    lv_obj_set_style_text_color(obj_header, lv_color_hex(0xE94560), 0);
    lv_obj_set_style_text_font(obj_header, &lv_font_montserrat_14, 0);
    lv_obj_align_to(obj_header, g_hud_wallpass, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 8);

    /* Row 6: objects list */
    g_hud_objects = lv_label_create(g_game_panel);
    lv_label_set_text(g_hud_objects, "(none)");
    lv_obj_set_style_text_color(g_hud_objects, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(g_hud_objects, &lv_font_montserrat_14, 0);
    lv_obj_align_to(g_hud_objects, obj_header, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    /* Win/Lose overlay (centered on game image area) */
    g_hud_end_msg = lv_label_create(game_scr);
    lv_label_set_text(g_hud_end_msg, "");
    lv_obj_set_style_text_color(g_hud_end_msg, lv_color_hex(0xFFD700), 0);
    lv_obj_set_style_text_font(g_hud_end_msg, &lv_font_montserrat_14, 0);
    lv_obj_align(g_hud_end_msg, LV_ALIGN_TOP_MID, 0, 300);
    lv_obj_add_flag(g_hud_end_msg, LV_OBJ_FLAG_HIDDEN);

    /* Switch to the game screen (hides original screen automatically) */
    lv_screen_load(game_scr);

    bsp_display_unlock();

    g_game_active = true;
    ESP_LOGI(TAG, "Game mode initialized (portrait, no rotation)");
    return ESP_OK;
}

void display_exit_game_mode(void)
{
    if (!g_game_active) return;

    ESP_LOGI(TAG, "Exiting game mode...");

    bsp_display_lock(portMAX_DELAY);

    /* Get current game screen before switching away */
    lv_obj_t *game_scr = lv_screen_active();

    /* Switch back to original screen */
    if (g_orig_scr) {
        lv_screen_load(g_orig_scr);
    }

    /* Now safe to delete the game screen and all its children */
    if (game_scr && game_scr != g_orig_scr) {
        lv_obj_delete(game_scr);
    }

    g_game_top_bar   = NULL;
    g_game_top_status = NULL;
    g_game_img       = NULL;
    g_game_panel     = NULL;
    g_hud_pos        = NULL;
    g_hud_speed      = NULL;
    g_hud_tilt       = NULL;
    g_hud_bounce     = NULL;
    g_hud_wallpass   = NULL;
    g_hud_objects    = NULL;
    g_hud_end_msg    = NULL;

    bsp_display_unlock();

    /* Free game double buffers */
    if (g_game_front_buf) { heap_caps_free(g_game_front_buf); g_game_front_buf = NULL; }
    if (g_game_back_buf)  { heap_caps_free(g_game_back_buf);  g_game_back_buf  = NULL; }

    g_game_active = false;
    ESP_LOGI(TAG, "Game mode exited");
}

uint8_t *display_get_game_render_buf(int *w, int *h)
{
    if (w) *w = GAME_BUF_W;
    if (h) *h = GAME_BUF_H;
    return g_game_back_buf;
}

void display_refresh_game(void)
{
    if (!g_game_active) return;

    bsp_display_lock(portMAX_DELAY);

    /* Atomic swap: back ↔ front */
    uint8_t *tmp = g_game_front_buf;
    g_game_front_buf = g_game_back_buf;
    g_game_back_buf  = tmp;

    g_game_front_dsc.data = g_game_front_buf;
    g_game_back_dsc.data  = g_game_back_buf;

    /* Invalidate LVGL image to trigger redraw */
    if (g_game_img) {
        lv_image_set_src(g_game_img, &g_game_front_dsc);
        lv_obj_invalidate(g_game_img);
    }

    bsp_display_unlock();
}

void display_update_game_hud(float marble_x, float marble_y, float speed,
                              float tilt_x, float tilt_y,
                              int wall_pass_ms, const char *bounce_label,
                              const game_object_t *objects, int object_count)
{
    if (!g_game_active) return;

    bsp_display_lock(portMAX_DELAY);

    char buf[128];

    /* Position */
    snprintf(buf, sizeof(buf), "Marble X:%.0f  Y:%.0f",
             (double)marble_x, (double)marble_y);
    if (g_hud_pos) lv_label_set_text(g_hud_pos, buf);

    /* Speed */
    snprintf(buf, sizeof(buf), "Speed: %.0f px/s", (double)speed);
    if (g_hud_speed) lv_label_set_text(g_hud_speed, buf);

    /* Tilt */
    snprintf(buf, sizeof(buf), "Tilt: X %+.1f deg  Y %+.1f deg",
             (double)tilt_x, (double)tilt_y);
    if (g_hud_tilt) lv_label_set_text(g_hud_tilt, buf);

    /* Bounce */
    snprintf(buf, sizeof(buf), "Bounce: %s", bounce_label ? bounce_label : "---");
    if (g_hud_bounce) lv_label_set_text(g_hud_bounce, buf);

    /* Wall-pass */
    if (wall_pass_ms > 0) {
        snprintf(buf, sizeof(buf), "Wall-pass: ON (%.1fs remaining)",
                 (double)wall_pass_ms / 1000.0);
        if (g_hud_wallpass) {
            lv_label_set_text(g_hud_wallpass, buf);
            lv_obj_set_style_text_color(g_hud_wallpass, lv_color_hex(0x00FF00), 0);
        }
    } else {
        if (g_hud_wallpass) {
            lv_label_set_text(g_hud_wallpass, "Wall-pass: OFF");
            lv_obj_set_style_text_color(g_hud_wallpass, lv_color_hex(0x888888), 0);
        }
    }

    /* Object list */
    if (g_hud_objects) {
        if (object_count > 0 && objects) {
            char obj_buf[512];
            int off = 0;
            for (int i = 0; i < object_count && off < (int)sizeof(obj_buf) - 40; i++) {
                const game_object_t *obj = &objects[i];
                const char *name = pixel_world_coco_name(obj->coco_id);
                const char *status = obj->alive ? "" : " (picked)";
                const char *icon = "";
                switch (obj->type) {
                    case GAMEOBJ_FRUIT:   icon = "* "; break;
                    case GAMEOBJ_PORTAL:  icon = "O "; break;
                    case GAMEOBJ_DEATH:   icon = "X "; break;
                    case GAMEOBJ_GOAL:    icon = "! "; break;
                    case GAMEOBJ_SURFACE: icon = "~ "; break;
                    default: break;
                }
                off += snprintf(obj_buf + off, sizeof(obj_buf) - off,
                                "%s%s%s\n", icon, name, status);
            }
            lv_label_set_text(g_hud_objects, obj_buf);
        } else {
            lv_label_set_text(g_hud_objects, "(none)");
        }
    }

    bsp_display_unlock();
}

void display_show_game_end(bool is_win)
{
    if (!g_game_active || !g_hud_end_msg) return;

    bsp_display_lock(portMAX_DELAY);

    if (is_win) {
        lv_label_set_text(g_hud_end_msg, "YOU WIN!\nBottle reached!");
        lv_obj_set_style_text_color(g_hud_end_msg, lv_color_hex(0xFFD700), 0);
    } else {
        lv_label_set_text(g_hud_end_msg, "GAME OVER\nScissors touch!");
        lv_obj_set_style_text_color(g_hud_end_msg, lv_color_hex(0xFF4444), 0);
    }
    lv_obj_remove_flag(g_hud_end_msg, LV_OBJ_FLAG_HIDDEN);

    bsp_display_unlock();
}
