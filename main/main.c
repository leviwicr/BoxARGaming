/**
 * 智绘灵境 —— AR Interactive Sandbox
 * 阶段2: 目标检测 (ESP-DL YOLO11n)
 *
 * 当前: 摄像头画面预览 + 按键触发检测
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "config.h"
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"

static const char *TAG = "main";

#define PREVIEW_W  640
#define PREVIEW_H  512   /* 800:640 = 5:4, 640/800*640=512 */

static volatile bool g_trigger_detect  = false;
static volatile bool g_trigger_preview = false;
static volatile bool g_live_view       = false;
static lv_obj_t  *g_status_label = NULL;
static lv_obj_t  *g_preview_img  = NULL;
static uint8_t   *g_preview_buf  = NULL;  /* 缩放后的预览数据 */
static lv_image_dsc_t g_preview_dsc;

/* ---- 缩放: 800x640 → PREVIEW_W×PREVIEW_H (最近邻) ---- */
static void scale_frame(const camera_frame_t *frame, uint8_t *dst_buf)
{
    uint16_t *src = (uint16_t *)frame->buffer;
    uint16_t *dst = (uint16_t *)dst_buf;

    for (int y = 0; y < PREVIEW_H; y++) {
        int src_y = y * CAMERA_V_RES / PREVIEW_H;
        uint16_t *src_row = src + src_y * CAMERA_H_RES;
        uint16_t *dst_row = dst + y * PREVIEW_W;
        for (int x = 0; x < PREVIEW_W; x++) {
            int src_x = x * CAMERA_H_RES / PREVIEW_W;
            dst_row[x] = src_row[src_x];
        }
    }
}

/* ---- 更新预览图像 ---- */
static void update_preview_display(void)
{
    if (!g_preview_img) return;
    lv_image_set_src(g_preview_img, &g_preview_dsc);
    lv_obj_invalidate(g_preview_img);
}

/* ---- 按键回调 ---- */
static void btn_preview_callback(lv_event_t *e)
{
    g_live_view = !g_live_view;
}

static void btn_detect_callback(lv_event_t *e)
{
    g_trigger_detect = true;
}

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  AR Sandbox - Phase 2: Detection");
    ESP_LOGI(TAG, "==============================================");

    /* ---- 1. 初始化显示 ---- */
    ESP_LOGI(TAG, "Initializing display and LVGL...");
    lv_display_t *disp = bsp_display_start();
    if (!disp) {
        ESP_LOGE(TAG, "Display init FAILED");
        return;
    }
    bsp_display_backlight_on();
    ESP_LOGI(TAG, "Display OK: %dx%d", BSP_LCD_H_RES, BSP_LCD_V_RES);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* ---- 2. 构建 UI ---- */
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
    g_preview_buf = heap_caps_calloc(1, PREVIEW_W * PREVIEW_H * 2,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (g_preview_buf) {
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
    } else {
        ESP_LOGE(TAG, "Failed to allocate preview buffer");
    }

    /* 状态标签 (在预览下方) */
    g_status_label = lv_label_create(scr);
    lv_label_set_text(g_status_label, "Status: Init camera...");
    lv_obj_set_style_text_color(g_status_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(g_status_label, &lv_font_montserrat_14, 0);
    lv_obj_align(g_status_label, LV_ALIGN_BOTTOM_MID, 0, -55);

    /* 按钮 */
    lv_obj_t *btn_preview = lv_button_create(scr);
    lv_obj_set_size(btn_preview, 150, 45);
    lv_obj_align(btn_preview, LV_ALIGN_BOTTOM_LEFT, 15, -5);
    lv_obj_add_event_cb(btn_preview, btn_preview_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_preview_label = lv_label_create(btn_preview);
    lv_label_set_text(btn_preview_label, "Live View");
    lv_obj_center(btn_preview_label);

    lv_obj_t *btn_detect = lv_button_create(scr);
    lv_obj_set_size(btn_detect, 150, 45);
    lv_obj_align(btn_detect, LV_ALIGN_BOTTOM_RIGHT, -15, -5);
    lv_obj_add_event_cb(btn_detect, btn_detect_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_detect_label = lv_label_create(btn_detect);
    lv_label_set_text(btn_detect_label, "Detect");
    lv_obj_center(btn_detect_label);

    bsp_display_unlock();

    /* ---- 3. 初始化相机 ---- */
    ESP_LOGI(TAG, "--- Camera Init Start ---");
    bsp_display_lock(portMAX_DELAY);
    lv_label_set_text(g_status_label, "Status: Init camera...");
    bsp_display_unlock();

    esp_err_t ret = camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: %s (0x%X)", esp_err_to_name(ret), ret);
        bsp_display_lock(portMAX_DELAY);
        lv_label_set_text_fmt(g_status_label, "Camera init FAILED!\n%s", esp_err_to_name(ret));
        lv_obj_set_style_text_color(g_status_label, lv_color_hex(0xFF0000), 0);
        bsp_display_unlock();
        return;
    }

    /* ---- 4. 相机预热 ---- */
    bsp_display_lock(portMAX_DELAY);
    lv_label_set_text(g_status_label, "Status: Warming up...");
    bsp_display_unlock();
    camera_warmup(10);

    bsp_display_lock(portMAX_DELAY);
    lv_label_set_text(g_status_label, "Ready! Press LIVE VIEW or DETECT");
    lv_obj_set_style_text_color(g_status_label, lv_color_hex(0x00FF00), 0);
    bsp_display_unlock();

    ESP_LOGI(TAG, "Camera ready");

    /* ---- 5. 主循环 ---- */
    bool was_live_view = false;
    while (1) {
        bool detect_now = false;

        if (g_trigger_detect) {
            g_trigger_detect = false;
            detect_now = true;
        }

        /* 实时预览模式 */
        if (g_live_view) {
            if (!was_live_view) {
                ESP_LOGI(TAG, "Live view ON");
                bsp_display_lock(portMAX_DELAY);
                lv_label_set_text(g_status_label, "Live View ON  (press again to stop)");
                lv_obj_set_style_text_color(g_status_label, lv_color_hex(0x00AAFF), 0);
                bsp_display_unlock();
                was_live_view = true;
            }

            camera_frame_t frame;
            ret = camera_capture_frame(&frame, 2000);
            if (ret == ESP_OK && g_preview_buf) {
                scale_frame(&frame, g_preview_buf);
                bsp_display_lock(portMAX_DELAY);
                update_preview_display();
                bsp_display_unlock();
            }
        } else {
            if (was_live_view) {
                ESP_LOGI(TAG, "Live view OFF");
                bsp_display_lock(portMAX_DELAY);
                lv_label_set_text(g_status_label, "Live View OFF  (press DETECT)");
                lv_obj_set_style_text_color(g_status_label, lv_color_hex(0x00FF00), 0);
                bsp_display_unlock();
                was_live_view = false;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        /* 按键触发检测 */
        if (detect_now) {
            ESP_LOGI(TAG, "--- Detection triggered ---");

            bsp_display_lock(portMAX_DELAY);
            lv_label_set_text(g_status_label, "Status: Capturing...");
            bsp_display_unlock();

            camera_frame_t frame;
            ret = camera_capture_frame(&frame, 2000);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Capture FAILED: %s", esp_err_to_name(ret));
                bsp_display_lock(portMAX_DELAY);
                lv_label_set_text_fmt(g_status_label, "Capture FAILED!\n%s", esp_err_to_name(ret));
                lv_obj_set_style_text_color(g_status_label, lv_color_hex(0xFF0000), 0);
                bsp_display_unlock();
                goto detect_done;
            }

            /* 更新预览 */
            if (g_preview_buf) {
                scale_frame(&frame, g_preview_buf);
                bsp_display_lock(portMAX_DELAY);
                update_preview_display();
                bsp_display_unlock();
            }

            /* 像素诊断 */
            ESP_LOGI(TAG, "Frame: %dx%d", frame.width, frame.height);
            ESP_LOGI(TAG, "  First 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                     frame.buffer[0], frame.buffer[1], frame.buffer[2], frame.buffer[3],
                     frame.buffer[4], frame.buffer[5], frame.buffer[6], frame.buffer[7]);

            /* 目标检测 */
            bsp_display_lock(portMAX_DELAY);
            lv_label_set_text(g_status_label, "Status: Running detection...");
            bsp_display_unlock();

            detection_result_t detections[DETECTION_MAX_OBJECTS];
            int det_count = DETECTION_MAX_OBJECTS;
            ret = detection_run(&frame, detections, &det_count);

            bsp_display_lock(portMAX_DELAY);
            if (ret != ESP_OK) {
                lv_label_set_text_fmt(g_status_label, "Detection FAILED!\n%s", esp_err_to_name(ret));
                lv_obj_set_style_text_color(g_status_label, lv_color_hex(0xFF0000), 0);
                ESP_LOGE(TAG, "Detection failed: %s", esp_err_to_name(ret));
            } else if (det_count > 0) {
                char result_str[256];
                int off = snprintf(result_str, sizeof(result_str), "Found %d objects:", det_count);
                for (int i = 0; i < det_count && off < (int)sizeof(result_str) - 30; i++) {
                    off += snprintf(result_str + off, sizeof(result_str) - off,
                                    "\n%s (%.0f%%)", detections[i].label, detections[i].score * 100);
                }
                lv_label_set_text(g_status_label, result_str);
                lv_obj_set_style_text_color(g_status_label, lv_color_hex(0x00FF00), 0);
                ESP_LOGI(TAG, "Detected %d objects", det_count);
            } else {
                lv_label_set_text(g_status_label, "No objects detected\nCheck objects / lighting");
                lv_obj_set_style_text_color(g_status_label, lv_color_hex(0xFFCC00), 0);
                ESP_LOGW(TAG, "No objects detected");
            }
            bsp_display_unlock();
        }

detect_done:
        ;
    }
}
