#pragma once

#include <stdint.h>
#include "detection/detection_driver.h"
#include "camera/camera_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTOUR_MAX_DIM  64   /* 每个物体轮廓掩码的最大尺寸 */

/**
 * @brief 单个物体的轮廓数据
 *
 * box_game: 游戏地图坐标 [x1,y1,x2,y2] (640×640 空间), 用于在画布上定位
 * mask:     CONTOUR_MAX_DIM² 的二值边缘图 (0=空, 255=轮廓)
 * mask_w/h: 掩码实际宽高 (≤ CONTOUR_MAX_DIM)
 */
typedef struct {
    int     box_game[4];
    uint8_t mask[CONTOUR_MAX_DIM * CONTOUR_MAX_DIM];
    int     mask_w, mask_h;
    bool    valid;
} object_contour_t;

/**
 * @brief Render one game frame onto the given RGB565 buffer.
 *
 * Pipeline: background → grid → track walls → marble → object contours.
 */
void game_render_frame(uint16_t *buf, int w, int h);

/**
 * @brief 从相机帧中提取每个检测物体的轮廓线
 *
 * 对每个检测到的物体:
 *   裁剪 ROI → 下采样 → 灰度化 → Sobel 边缘检测 → 存入轮廓掩码
 *
 * @param frame       相机帧 (RGB565, 800×640)
 * @param detections  检测结果数组
 * @param count       检测数量
 */
void game_extract_contours(const camera_frame_t *frame,
                           const detection_result_t *detections, int count);

/**
 * @brief Render pixel game world to a 640×640 RGB565 buffer.
 *
 * Pipeline: floor background → tilemap walls → game objects (sprites) → marble.
 * Called at ~30fps during PLAYING state. Coordinates are 1:1 (buffer = game map).
 *
 * @param buf  Target buffer (640×640 RGB565)
 * @param w    Buffer width (640)
 * @param h    Buffer height (640)
 */
void game_render_pixel_frame(uint16_t *buf, int w, int h);

#ifdef __cplusplus
}
#endif
