/**
 * 目标检测驱动 —— ESP-DL COCODetect 封装
 *
 * C++ 实现，通过 extern "C" 接口供 main.c 调用。
 */

#include <stdio.h>
#include <list>
#include <cstring>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "coco_detect.hpp"
#include "dl_image_define.hpp"
#include "dl_image_process.hpp"
#include "config.h"
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"
#include "image_processing/image_processing.hpp"

static const char *TAG = "detection";

static COCODetect *g_detect = NULL;

/* COCO 类别名称 (80 classes) */
static const char *COCO_CLASSES[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
};

/* 桌面物体类别白名单 (COCO 索引) */
static bool is_desktop_object(int category)
{
    switch (category) {
    case 39:  /* bottle */
    case 41:  /* cup */
    case 44:  /* spoon */
    case 64:  /* mouse */
    case 66:  /* keyboard */
    case 67:  /* cell phone */
    case 73:  /* book */
    case 76:  /* scissors */
        return true;
    default:
        return false;
    }
}

esp_err_t detection_init(void)
{
    if (g_detect) return ESP_OK;

    ESP_LOGI(TAG, "Initializing COCODetect (lazy load on first run)");
    g_detect = new COCODetect(COCODetect::YOLO11N_320_S8_V1);
    if (!g_detect) {
        ESP_LOGE(TAG, "Failed to create COCODetect");
        return ESP_ERR_NO_MEM;
    }

    /* 设置阈值：置信度 >= 0.3, NMS IoU <= 0.7 */
    g_detect->set_score_thr(0.15f);
    g_detect->set_nms_thr(0.7f);

    ESP_LOGI(TAG, "COCODetect ready");
    return ESP_OK;
}

esp_err_t detection_run(const camera_frame_t *frame,
                        detection_result_t *results,
                        int *count,
                        uint32_t preprocess_flags)
{
    if (!frame || !frame->buffer || !results || !count || *count <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_detect) {
        esp_err_t ret = detection_init();
        if (ret != ESP_OK) return ret;
    }

    /* 构建 ESP-DL 图像结构 */
    dl::image::img_t img;
    img.data     = frame->buffer;
    img.width    = frame->width;
    img.height   = frame->height;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

    /* 可选: 预处理管线 (RGB565 → RGB888 → preprocess → RGB565) */
    dl::image::img_t proc_img = img;
    bool preprocessed = false;

    if (preprocess_flags != 0) {
        uint8_t *temp_buf = preprocessing_get_temp_buffer();
        if (!temp_buf) {
            ESP_LOGW(TAG, "Preprocessing temp buffer not available, skipping");
        } else {
            /* 分配处理后的 RGB565 缓冲区 (与帧相同尺寸) */
            size_t rgb565_size = frame->width * frame->height * 2;
            uint8_t *proc_buf = (uint8_t *)heap_caps_calloc(1, rgb565_size,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!proc_buf) {
                ESP_LOGW(TAG, "Failed to alloc preprocessing buffer, skipping");
            } else {
                ESP_LOGI(TAG, "Applying preprocessing flags=0x%02X", (unsigned)preprocess_flags);

                /* Step 1: RGB565 → RGB888 (使用 ImageTransformer) */
                dl::image::img_t src_rgb888;
                src_rgb888.data     = temp_buf;
                src_rgb888.width    = frame->width;
                src_rgb888.height   = frame->height;
                src_rgb888.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

                dl::image::ImageTransformer()
                    .set_src_img(img)
                    .set_dst_img(src_rgb888)
                    .transform();

                /* Step 2: 预处理 (原地修改 RGB888) */
                preprocess(src_rgb888, preprocess_flags);

                /* Step 3: RGB888 → RGB565 (使用 ImageTransformer) */
                dl::image::img_t dst_rgb565;
                dst_rgb565.data     = proc_buf;
                dst_rgb565.width    = frame->width;
                dst_rgb565.height   = frame->height;
                dst_rgb565.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

                dl::image::ImageTransformer()
                    .set_src_img(src_rgb888)
                    .set_dst_img(dst_rgb565)
                    .transform();

                proc_img = dst_rgb565;
                preprocessed = true;
            }
        }
    }

    /* 运行检测 (预处理 + 推理 + 后处理 自动完成) */
    ESP_LOGI(TAG, "Running detection on %dx%d frame...",
             proc_img.width, proc_img.height);

    /* 像素采样诊断: 打印前 8 个像素验证数据 */
    {
        uint16_t *px = (uint16_t *)proc_img.data;
        ESP_LOGI(TAG, "Pixel[0..7]: %04X %04X %04X %04X %04X %04X %04X %04X",
                 px[0], px[1], px[2], px[3], px[4], px[5], px[6], px[7]);
    }

    std::list<dl::detect::result_t> &detect_results = g_detect->run(proc_img);

    /* 释放预处理缓冲区 */
    if (preprocessed) {
        heap_caps_free(proc_img.data);
    }

    /* 诊断: 输出原始检测数量 */
    int raw_count = 0;
    for (const auto &res : detect_results) {
        raw_count++;
        ESP_LOGI(TAG, "  RAW[%d] cat=%d score=%.3f box=[%d,%d,%d,%d]",
                 raw_count, res.category, res.score,
                 res.box[0], res.box[1], res.box[2], res.box[3]);
    }
    ESP_LOGI(TAG, "Raw detections: %d total", raw_count);

    /* 坐标转换: 相机 (800x640) → 地图 (640x640) */
    const float scale_x = (float)MAP_SIZE / (float)CAMERA_H_RES;
    const float scale_y = (float)MAP_SIZE / (float)CAMERA_V_RES;

    int result_idx = 0;
    int max_count = *count;
    *count = 0;

    for (const auto &res : detect_results) {
        if (result_idx >= max_count) break;

        /* 白名单过滤 */
        if (!is_desktop_object(res.category)) continue;

        /* 存储相机空间原始坐标 (裁剪到有效范围) */
        int cx1 = res.box[0], cy1 = res.box[1];
        int cx2 = res.box[2], cy2 = res.box[3];
        if (cx1 < 0) cx1 = 0;
        if (cy1 < 0) cy1 = 0;
        if (cx2 >= CAMERA_H_RES) cx2 = CAMERA_H_RES - 1;
        if (cy2 >= CAMERA_V_RES) cy2 = CAMERA_V_RES - 1;

        /* 跳过无效框 */
        if (cx2 <= cx1 || cy2 <= cy1) continue;

        /* 转换为地图坐标 */
        int x1 = (int)(cx1 * scale_x);
        int y1 = (int)(cy1 * scale_y);
        int x2 = (int)(cx2 * scale_x);
        int y2 = (int)(cy2 * scale_y);

        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x2 >= MAP_SIZE) x2 = MAP_SIZE - 1;
        if (y2 >= MAP_SIZE) y2 = MAP_SIZE - 1;

        results[result_idx].category      = res.category;
        results[result_idx].score         = res.score;
        results[result_idx].box[0]        = x1;
        results[result_idx].box[1]        = y1;
        results[result_idx].box[2]        = x2;
        results[result_idx].box[3]        = y2;
        results[result_idx].box_camera[0] = cx1;
        results[result_idx].box_camera[1] = cy1;
        results[result_idx].box_camera[2] = cx2;
        results[result_idx].box_camera[3] = cy2;
        results[result_idx].label         = (res.category >= 0 && res.category < 80)
                                            ? COCO_CLASSES[res.category] : "unknown";

        ESP_LOGI(TAG, "[%d] %s score=%.2f box=[%d,%d,%d,%d] map=[%d,%d,%d,%d]",
                 result_idx,
                 results[result_idx].label,
                 results[result_idx].score,
                 cx1, cy1, cx2, cy2,
                 x1, y1, x2, y2);

        result_idx++;
    }

    *count = result_idx;
    ESP_LOGI(TAG, "Detection done: %d objects found", result_idx);

    return ESP_OK;
}

void detection_deinit(void)
{
    if (g_detect) {
        delete g_detect;
        g_detect = NULL;
        ESP_LOGI(TAG, "COCODetect deinitialized");
    }
}
