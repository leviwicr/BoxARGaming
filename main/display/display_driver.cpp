/**
 * 显示驱动 —— LVGL UI、预览缓冲、按键状态管理
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

static const char *TAG = "display";

/* 预览尺寸: 保持 5:4 宽高比, 宽度 640 时高度为 512 */
#define PREVIEW_W  640
#define PREVIEW_H  512

/* ---- 内部状态 ---- */
static lv_obj_t  *g_status_label = NULL;
static lv_obj_t  *g_preview_img  = NULL;
static uint8_t   *g_preview_buf  = NULL;
static lv_image_dsc_t g_preview_dsc;

static volatile bool g_trigger_detect  = false;
static volatile bool g_live_view       = false;

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

/* ---- 在预览缓冲上绘制检测框 ---- */
static void draw_boxes_on_preview(const detection_result_t *detections, int det_count)
{
    if (!detections || det_count <= 0) return;

    dl::image::img_t preview_img;
    preview_img.data     = g_preview_buf;
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

/* ---- 刷新 LVGL 预览图像 ---- */
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
    g_live_view = !g_live_view;
}

static void btn_detect_callback(lv_event_t *e)
{
    (void)e;
    g_trigger_detect = true;
}

static void btn_preproc_callback(lv_event_t *e)
{
    (void)e;
    g_preproc_idx = (g_preproc_idx + 1) % PREPROC_PRESET_COUNT;

    /* 更新按钮标签显示当前模式 */
    if (g_btn_preproc_label) {
        lv_label_set_text(g_btn_preproc_label, g_preproc_names[g_preproc_idx]);
    }
}

/* ========================================================================
 * 公开接口
 * ======================================================================== */

esp_err_t display_init(void)
{
    ESP_LOGI(TAG, "Initializing display and LVGL...");

    /* 1. 启动显示 */
    lv_display_t *disp = bsp_display_start();
    if (!disp) {
        ESP_LOGE(TAG, "Display init FAILED");
        return ESP_FAIL;
    }
    bsp_display_backlight_on();
    ESP_LOGI(TAG, "Display OK: %dx%d", BSP_LCD_H_RES, BSP_LCD_V_RES);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 2. 构建 UI */
    bsp_display_lock(portMAX_DELAY);

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101010), 0);

    /* 标题 */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Camera Preview | ESP-DL Detection");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

    /* 预览图像 (640x512 RGB565, 居中) */
    g_preview_buf = (uint8_t *)heap_caps_calloc(1, PREVIEW_W * PREVIEW_H * 2,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_preview_buf) {
        ESP_LOGE(TAG, "Failed to allocate preview buffer");
        bsp_display_unlock();
        return ESP_ERR_NO_MEM;
    }
    memset(g_preview_buf, 0x00, PREVIEW_W * PREVIEW_H * 2);

    g_preview_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    g_preview_dsc.header.w      = PREVIEW_W;
    g_preview_dsc.header.h      = PREVIEW_H;
    g_preview_dsc.header.stride = PREVIEW_W * 2;
    g_preview_dsc.data          = g_preview_buf;
    g_preview_dsc.data_size     = PREVIEW_W * PREVIEW_H * 2;

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

    /* 按钮 */
    lv_obj_t *btn_preview = lv_button_create(scr);
    lv_obj_set_size(btn_preview, 120, 40);
    lv_obj_align(btn_preview, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_obj_add_event_cb(btn_preview, btn_preview_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_preview_label = lv_label_create(btn_preview);
    lv_label_set_text(btn_preview_label, "Live View");
    lv_obj_center(btn_preview_label);

    lv_obj_t *btn_preproc = lv_button_create(scr);
    lv_obj_set_size(btn_preproc, 120, 40);
    lv_obj_align(btn_preproc, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_add_event_cb(btn_preproc, btn_preproc_callback, LV_EVENT_CLICKED, NULL);
    g_btn_preproc_label = lv_label_create(btn_preproc);
    lv_label_set_text(g_btn_preproc_label, g_preproc_names[g_preproc_idx]);
    lv_obj_center(g_btn_preproc_label);

    lv_obj_t *btn_detect = lv_button_create(scr);
    lv_obj_set_size(btn_detect, 120, 40);
    lv_obj_align(btn_detect, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
    lv_obj_add_event_cb(btn_detect, btn_detect_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_detect_label = lv_label_create(btn_detect);
    lv_label_set_text(btn_detect_label, "Detect");
    lv_obj_center(btn_detect_label);

    bsp_display_unlock();

    ESP_LOGI(TAG, "Display UI ready");
    return ESP_OK;
}

esp_err_t display_update_preview(const camera_frame_t *frame,
                                  const detection_result_t *detections,
                                  int det_count)
{
    if (!frame || !frame->buffer || !g_preview_buf) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 1. 缩放 (ImageTransformer 替代手写 scale_frame) */
    scale_frame(frame, g_preview_buf);

    /* 2. 绘制检测框 (如果有) */
    draw_boxes_on_preview(detections, det_count);

    /* 3. 刷新 LVGL */
    bsp_display_lock(portMAX_DELAY);
    update_preview_display();
    bsp_display_unlock();

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
