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
    g_live_view = !g_live_view;
    if (g_live_view) g_edge_view = false;
}

static void btn_edge_callback(lv_event_t *e)
{
    (void)e;
    g_edge_view = !g_edge_view;
    if (g_edge_view) g_live_view = false;
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

static void btn_track_capture_callback(lv_event_t *e)
{
    (void)e;
    g_track_capture = true;
    g_edge_view = true;       /* 自动进入边缘检测模式 */
    g_live_view = false;
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

    /* 标题 */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Camera Preview | ESP-DL Detection");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

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

    lv_obj_t *btn_edge = lv_button_create(scr);
    lv_obj_set_size(btn_edge, 80, 40);
    lv_obj_align(btn_edge, LV_ALIGN_BOTTOM_MID, -90, -5);
    lv_obj_add_event_cb(btn_edge, btn_edge_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_edge_label = lv_label_create(btn_edge);
    lv_label_set_text(btn_edge_label, "Edge");
    lv_obj_center(btn_edge_label);

    lv_obj_t *btn_track = lv_button_create(scr);
    lv_obj_set_size(btn_track, 80, 40);
    lv_obj_align(btn_track, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_add_event_cb(btn_track, btn_track_capture_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_track_label = lv_label_create(btn_track);
    lv_label_set_text(btn_track_label, "Track");
    lv_obj_center(btn_track_label);

    lv_obj_t *btn_preproc = lv_button_create(scr);
    lv_obj_set_size(btn_preproc, 80, 40);
    lv_obj_align(btn_preproc, LV_ALIGN_BOTTOM_MID, 90, -5);
    lv_obj_add_event_cb(btn_preproc, btn_preproc_callback, LV_EVENT_CLICKED, NULL);
    g_btn_preproc_label = lv_label_create(btn_preproc);
    lv_label_set_text(g_btn_preproc_label, g_preproc_names[g_preproc_idx]);
    lv_obj_center(g_btn_preproc_label);

    lv_obj_t *btn_detect = lv_button_create(scr);
    lv_obj_set_size(btn_detect, 80, 40);
    lv_obj_align(btn_detect, LV_ALIGN_BOTTOM_MID, 180, -5);
    lv_obj_add_event_cb(btn_detect, btn_detect_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_detect_label = lv_label_create(btn_detect);
    lv_label_set_text(btn_detect_label, "Detect");
    lv_obj_center(btn_detect_label);

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
