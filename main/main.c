/**
 * 智绘灵境 —— AR Interactive Sandbox
 * 阶段2: 目标检测 (ESP-DL YOLO11n)
 *
 * 当前: 摄像头画面预览 + 按键触发检测
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "config.h"
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"
#include "display/display_driver.h"
#include "image_processing/image_processing.hpp"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  AR Sandbox - Phase 2: Detection");
    ESP_LOGI(TAG, "==============================================");

    /* ---- 1. 初始化显示 ---- */
    esp_err_t ret = display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Display init FAILED");
        return;
    }

    /* ---- 2. 初始化相机 ---- */
    ESP_LOGI(TAG, "--- Camera Init Start ---");
    display_set_status("Status: Init camera...", 0x00FF00);

    ret = camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: %s (0x%X)", esp_err_to_name(ret), ret);
        display_set_status("Camera init FAILED!", 0xFF0000);
        return;
    }

    /* ---- 3. 初始化预处理模块 ---- */
    ret = preprocessing_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Preprocessing init failed, continuing without it");
    }

    /* ---- 4. 相机预热 ---- */
    display_set_status("Status: Warming up...", 0x00FF00);
    camera_warmup(10);

    display_set_status("Ready! Press LIVE VIEW or DETECT", 0x00FF00);
    ESP_LOGI(TAG, "Camera ready");

    /* ---- 5. 主循环 ---- */
    bool was_live_view = false;

    while (1) {
        /* 处理检测触发 */
        if (display_detect_triggered()) {
            ESP_LOGI(TAG, "--- Detection triggered ---");

            display_set_status("Status: Capturing...", 0x00FF00);

            camera_frame_t frame;
            ret = camera_capture_frame(&frame, 2000);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Capture FAILED: %s", esp_err_to_name(ret));
                display_set_status("Capture FAILED!", 0xFF0000);
                goto detect_done;
            }

            /* 像素诊断 */
            ESP_LOGI(TAG, "Frame: %dx%d", frame.width, frame.height);
            ESP_LOGI(TAG, "  First 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                     frame.buffer[0], frame.buffer[1], frame.buffer[2], frame.buffer[3],
                     frame.buffer[4], frame.buffer[5], frame.buffer[6], frame.buffer[7]);

            /* 预处理 (仅一次, 检测和预览共用) */
            camera_frame_t display_frame = frame;
            uint8_t *proc_buf = NULL;
            uint32_t preproc_flags = display_get_preproc_flags();

            if (preproc_flags != PREPROC_FLAG_NONE) {
                proc_buf = preprocess_frame_rgb565(
                    frame.buffer, frame.width, frame.height,
                    preproc_flags, NULL);
                if (proc_buf) {
                    display_frame.buffer  = proc_buf;
                    display_frame.buf_len = frame.width * frame.height * 2;
                    ESP_LOGI(TAG, "Preprocessing applied, flags=0x%02X",
                             (unsigned)preproc_flags);
                }
            }

            /* 目标检测 */
            display_set_status("Status: Running detection...", 0x00FF00);

            detection_result_t detections[DETECTION_MAX_OBJECTS];
            int det_count = DETECTION_MAX_OBJECTS;
            ret = detection_run(&display_frame, detections, &det_count, PREPROC_FLAG_NONE);

            /* 更新预览 (预处理后的画面 + 检测框) */
            display_update_preview(&display_frame, detections, det_count);

            /* 释放预处理缓冲区 */
            if (proc_buf) {
                heap_caps_free(proc_buf);
                proc_buf = NULL;
            }

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Detection failed: %s", esp_err_to_name(ret));
                display_set_status("Detection FAILED!", 0xFF0000);
            } else if (det_count > 0) {
                char result_str[256];
                int off = snprintf(result_str, sizeof(result_str), "Found %d objects:", det_count);
                for (int i = 0; i < det_count && off < (int)sizeof(result_str) - 30; i++) {
                    off += snprintf(result_str + off, sizeof(result_str) - off,
                                    "\n%s (%.0f%%)", detections[i].label, detections[i].score * 100);
                }
                display_set_status(result_str, 0x00FF00);
                ESP_LOGI(TAG, "Detected %d objects", det_count);
            } else {
                display_set_status("No objects detected\nCheck objects / lighting", 0xFFCC00);
                ESP_LOGW(TAG, "No objects detected");
            }
        }

    detect_done:

        /* 实时预览模式 */
        if (display_is_live_view()) {
            if (!was_live_view) {
                ESP_LOGI(TAG, "Live view ON");
                display_set_status("Live View ON  (press again to stop)", 0x00AAFF);
                was_live_view = true;
            }

            camera_frame_t frame;
            ret = camera_capture_frame(&frame, 2000);
            if (ret == ESP_OK) {
                display_update_preview(&frame, NULL, 0);
            }
        } else {
            if (was_live_view) {
                ESP_LOGI(TAG, "Live view OFF");
                display_set_status("Live View OFF  (press DETECT)", 0x00FF00);
                was_live_view = false;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
