#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize track collision module (allocate collision map)
 * @return ESP_OK on success
 */
esp_err_t track_collision_init(void);

/**
 * @brief Build collision map from Canny edge detection result
 *
 * Coordinates: edge map (400x320) is a 2x downscale of the 800x640 camera frame.
 * The game map is the center 640x640 crop of the camera, which corresponds to
 * a 320x320 region in edge map space (x∈[40,360), y∈[0,320)).
 * This is upscaled 2x to a 640x640 collision bitmap.
 *
 * @param edge_map  Input edge map (ew × eh uint8, 0=free, 255=edge)
 * @param ew, eh    Edge map width/height (400×320)
 * @return ESP_OK on success
 */
esp_err_t track_build_from_edges(const uint8_t *edge_map, int ew, int eh);

/**
 * @brief Check if a game coordinate (in MAP_SIZE 640×640) is a wall
 * @return true if wall, false if free space
 */
bool track_is_wall(int game_x, int game_y);

/**
 * @brief Get wall normal at a game coordinate (from Sobel gradient of collision map)
 * @param game_x, game_y  Game coordinate (0..639)
 * @param nx, ny          Output: wall normal (normalized, points toward free space)
 * @return true if wall normal is valid, false if not near a wall
 */
bool track_get_normal(int game_x, int game_y, float *nx, float *ny);

/**
 * @brief Render track walls onto a display buffer
 *
 * Renders the collision map as colored wall lines by downscaling the 640×640
 * game map to the display area (512×512 at offset 64,0 in the 640×512 preview).
 *
 * @param buf    Target RGB565 buffer (preview buffer)
 * @param buf_w  Buffer width (640)
 * @param buf_h  Buffer height (512)
 * @param wall_color  RGB565 color for wall pixels
 */
void track_render(uint16_t *buf, int buf_w, int buf_h, uint16_t wall_color);

/**
 * @brief Check if collision map has been built (non-empty)
 */
bool track_is_built(void);

/**
 * @brief Mark a rectangular region as wall in the collision map.
 *
 * Used to add book walls and other game-tilemap walls that aren't from
 * Canny edges. Must be called after track_build_from_edges().
 *
 * @param x0, y0  Top-left corner in game map coords (0..639)
 * @param x1, y1  Bottom-right corner (inclusive)
 */
void track_set_wall_rect(int x0, int y0, int x1, int y1);

/**
 * @brief Clear a rectangular region in the collision map (make passable).
 *
 * Used when a destructible book wall is destroyed.
 *
 * @param x0, y0  Top-left corner in game map coords (0..639)
 * @param x1, y1  Bottom-right corner (inclusive)
 */
void track_clear_wall_rect(int x0, int y0, int x1, int y1);

/**
 * @brief Deinitialize and free collision map
 */
void track_collision_deinit(void);

#ifdef __cplusplus
}
#endif
