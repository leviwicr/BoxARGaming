#pragma once

#include "esp_err.h"
#include <stdint.h>
#include "camera/camera_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DETECTION_MAX_RESULTS 10

typedef struct {
    int         category;       /* COCO 类别索引 (0-79) */
    float       score;          /* 置信度 (0.0-1.0) */
    int         box[4];         /* 边界框 [x1, y1, x2, y2]，已转换到地图坐标 */
    const char *label;          /* 类别名称 */
} detection_result_t;

/**
 * @brief 初始化目标检测器
 *
 * 首次调用 detection_run() 时会延迟加载模型。
 *
 * @return ESP_OK 成功
 */
esp_err_t detection_init(void);

/**
 * @brief 对相机帧运行目标检测
 *
 * @param frame     相机帧数据 (RGB565)
 * @param results   输出检测结果数组
 * @param count     输入/输出: 传入最大结果数, 传出实际结果数
 * @return ESP_OK 成功
 */
esp_err_t detection_run(const camera_frame_t *frame,
                        detection_result_t *results,
                        int *count);

/**
 * @brief 反初始化检测器，释放资源
 */
void detection_deinit(void);

#ifdef __cplusplus
}
#endif
