#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize display, LVGL UI, and preview buffer
 * @return ESP_OK on success
 */
esp_err_t display_init(void);

/**
 * @brief Update preview image from camera frame (scales 800x640 → 640x512)
 *
 * Uses ESP-DL ImageTransformer (SIMD-accelerated NN resize).
 * Optionally draws detection bounding boxes on the preview.
 *
 * @param frame       Camera frame to display
 * @param detections  Detection results (NULL or count=0 to skip box drawing)
 * @param det_count   Number of valid detection results
 * @return ESP_OK on success
 */
esp_err_t display_update_preview(const camera_frame_t *frame,
                                  const detection_result_t *detections,
                                  int det_count);

/**
 * @brief Set status label text and color
 * @param text  Status text (will be copied)
 * @param color RGB888 color (e.g. 0x00FF00)
 */
void display_set_status(const char *text, uint32_t color);

/**
 * @brief Check if live view mode is active
 * @return true if live view is ON
 */
bool display_is_live_view(void);

/**
 * @brief Check and atomically clear the detect trigger flag
 * @return true if detect button was pressed since last check
 */
bool display_detect_triggered(void);

/**
 * @brief Get current preprocessing flags (runtime-switchable)
 * @return bitmask of PREPROC_FLAG_*
 */
uint32_t display_get_preproc_flags(void);

#ifdef __cplusplus
}
#endif
