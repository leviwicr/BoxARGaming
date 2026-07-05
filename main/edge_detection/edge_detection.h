#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize edge detection module (allocate PSRAM working buffers)
 * @return ESP_OK on success
 */
esp_err_t edge_detect_init(void);

/**
 * @brief Deinitialize edge detection module (free working buffers)
 */
void edge_detect_deinit(void);

/**
 * @brief Get downscale working buffer (400x320 RGB565, PSRAM)
 *
 * Caller fills this buffer via edge_downscale_half() before calling
 * edge_detect_run().
 *
 * @return Pointer to downscale buffer (EDGE_DOWNSCALE_W * EDGE_DOWNSCALE_H * 2 bytes)
 */
uint8_t *edge_get_downscale_buf(void);

/**
 * @brief Get edge map working buffer (EDGE_DOWNSCALE_W * EDGE_DOWNSCALE_H uint8, PSRAM)
 * @return Pointer to edge map buffer
 */
uint8_t *edge_get_edge_map_buf(void);

/**
 * @brief Downscale RGB565 image by 2x using area averaging
 *
 * Averages each 2x2 block into one output pixel (component-wise).
 * Assumes dst is at least (sw/2) * (sh/2) * 2 bytes.
 *
 * @param src Source RGB565 buffer
 * @param sw  Source width (pixels)
 * @param sh  Source height (pixels)
 * @param dst Destination RGB565 buffer
 */
void edge_downscale_half(const uint8_t *src, int sw, int sh, uint8_t *dst);

/**
 * @brief Run Canny edge detection on an RGB565 image
 *
 * Full 5-stage pipeline: RGB565→Gray → Gaussian Blur → Sobel → NMS → Hysteresis.
 * All processing uses fixed-point integer math (no floating point in inner loops).
 *
 * @param rgb565       Input RGB565 image buffer
 * @param w            Image width (pixels)
 * @param h            Image height (pixels)
 * @param edge_map_out Output edge map (caller-allocated, w*h uint8, 0=non-edge, 255=edge)
 * @param low_thresh   Low threshold for hysteresis (0-255)
 * @param high_thresh  High threshold for hysteresis (0-255)
 * @return ESP_OK on success
 */
esp_err_t edge_detect_run(const uint8_t *rgb565, int w, int h,
                          uint8_t *edge_map_out,
                          int low_thresh, int high_thresh);

#ifdef __cplusplus
}
#endif
