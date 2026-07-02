/**
 * 目标检测驱动 —— ESP-DL COCODetect 封装
 *
 * C++ 实现，通过 extern "C" 接口供 main.c 调用。
 */

#include <stdio.h>
#include <list>
#include "esp_log.h"
#include "coco_detect.hpp"
#include "dl_image_define.hpp"
#include "config.h"
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"

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
                        int *count)
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
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;//确认缓冲区的图像数据是RGB565大端还是小端！！

    /* 运行检测 (预处理 + 推理 + 后处理 自动完成) */
    ESP_LOGI(TAG, "Running detection on %dx%d frame...",
             frame->width, frame->height);

    /* 像素采样诊断: 打印前 16 个像素验证数据 */
    {
        uint16_t *px = (uint16_t *)frame->buffer;
        ESP_LOGI(TAG, "Pixel[0..7]: %04X %04X %04X %04X %04X %04X %04X %04X",
                 px[0], px[1], px[2], px[3], px[4], px[5], px[6], px[7]);
    }

    std::list<dl::detect::result_t> &detect_results = g_detect->run(img);

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

        /* 转换坐标并裁剪到地图范围 */
        int x1 = (int)(res.box[0] * scale_x);
        int y1 = (int)(res.box[1] * scale_y);
        int x2 = (int)(res.box[2] * scale_x);
        int y2 = (int)(res.box[3] * scale_y);

        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x2 >= MAP_SIZE) x2 = MAP_SIZE - 1;
        if (y2 >= MAP_SIZE) y2 = MAP_SIZE - 1;

        /* 跳过无效框 */
        if (x2 <= x1 || y2 <= y1) continue;

        results[result_idx].category = res.category;
        results[result_idx].score    = res.score;
        results[result_idx].box[0]   = x1;
        results[result_idx].box[1]   = y1;
        results[result_idx].box[2]   = x2;
        results[result_idx].box[3]   = y2;
        results[result_idx].label    = (res.category >= 0 && res.category < 80)
                                       ? COCO_CLASSES[res.category] : "unknown";

        ESP_LOGI(TAG, "[%d] %s score=%.2f box=[%d,%d,%d,%d] map=[%d,%d,%d,%d]",
                 result_idx,
                 results[result_idx].label,
                 results[result_idx].score,
                 res.box[0], res.box[1], res.box[2], res.box[3],
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
