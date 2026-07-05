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

/**
 * @brief Check if edge detection view mode is active
 * @return true if edge view is ON
 */
bool display_is_edge_view(void);

/**
 * @brief Check and atomically clear the track capture trigger flag
 * @return true if "Capture Track" button was pressed since last check
 */
bool display_track_capture_triggered(void);

/**
 * @brief Prepare edge detection preview in render buffer (no LVGL refresh)
 *
 * Converts scaled preview to grayscale and overlays colored edge lines.
 * Writes to the back buffer. Call display_refresh_preview() afterwards
 * to atomically swap and display.
 *
 * @param frame    Camera frame to display
 * @param edge_map Edge detection binary map (0=non-edge, 255=edge)
 * @param ew       Edge map width (pixels)
 * @param eh       Edge map height (pixels)
 * @return ESP_OK on success
 */
esp_err_t display_update_edge_preview(const camera_frame_t *frame,
                                       const uint8_t *edge_map,
                                       int ew, int eh);

/**
 * @brief Get direct pointer to preview buffer for external rendering
 * @param w  Output: preview buffer width
 * @param h  Output: preview buffer height
 * @return pointer to preview buffer (640x512 RGB565)
 */
uint8_t *display_get_preview_buf(int *w, int *h);

/**
 * @brief Get pointer to render buffer for external drawing (marble, etc.)
 *
 * This is the back buffer — safe to write outside the BSP display lock.
 * On display_refresh_preview() the back buffer is atomically swapped to front.
 *
 * @param w  Output: buffer width (640)
 * @param h  Output: buffer height (512)
 * @return pointer to render buffer (640x512 RGB565, PSRAM)
 */
uint8_t *display_get_render_buf(int *w, int *h);

/**
 * @brief Prepare preview (scale + draw boxes) without LVGL refresh
 *
 * Use this with display_get_render_buf() + display_refresh_preview()
 * to inject custom rendering (e.g. marble) between prepare and refresh.
 *
 * @param frame       Camera frame to display
 * @param detections  Detection results (NULL or count=0 to skip box drawing)
 * @param det_count   Number of valid detection results
 * @return ESP_OK on success
 */
esp_err_t display_prepare_preview(const camera_frame_t *frame,
                                   const detection_result_t *detections,
                                   int det_count);

/**
 * @brief Atomically swap render buffer ↔ preview buffer and refresh LVGL
 *
 * Must be called under BSP display lock. After the swap, the previous front
 * buffer becomes the new back buffer (available for the next frame's rendering).
 */
void display_refresh_preview(void);

#ifdef __cplusplus
}
#endif
