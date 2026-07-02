#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 相机帧数据
 */
typedef struct {
    uint8_t  *buffer;       /* 帧数据缓冲区 (RGB565, V4L2 mmap) */
    size_t    buf_len;      /* 缓冲区大小（字节） */
    uint16_t  width;        /* 帧宽度（像素） */
    uint16_t  height;       /* 帧高度（像素） */
} camera_frame_t;

/**
 * @brief 初始化相机 (OV5647 + MIPI-CSI + ISP via esp_video)
 *
 * 使用 esp_video V4L2 框架，通过 /dev/video0 访问 CSI 摄像头。
 * ISP 管线由 esp_video 自动管理 (RAW8→RGB565)。
 *
 * @return ESP_OK 成功，否则失败
 */
esp_err_t camera_init(void);

/**
 * @brief 捕获一帧图像
 *
 * @param out_frame 输出帧数据，buffer 指向内部 mmap 缓冲区
 * @param timeout_ms 超时时间（毫秒）
 * @return ESP_OK 成功
 */
esp_err_t camera_capture_frame(camera_frame_t *out_frame, uint32_t timeout_ms);

/**
 * @brief 反初始化相机，释放资源
 * @return ESP_OK 成功
 */
esp_err_t camera_deinit(void);

/**
 * @brief 相机预热：捕获并丢弃前 N 帧以稳定 AWB/AE
 * @param frames 要丢弃的帧数 (建议 5-10)
 * @return ESP_OK 成功
 */
esp_err_t camera_warmup(int frames);

/**
 * @brief 启用/禁用传感器测试图案 (用于诊断)
 * @param enable 1=启用, 0=禁用
 * @return ESP_OK 成功, ESP_ERR_NOT_SUPPORTED=传感器不支持
 */
esp_err_t camera_test_pattern(int enable);

#ifdef __cplusplus
}
#endif
