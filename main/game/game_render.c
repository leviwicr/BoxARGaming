/**
 * 游戏渲染模块 — 纯画布渲染 (无相机输入)
 *
 * 渲染管线: 背景 → 网格 → 赛道墙壁 → 弹珠 → 检测物体幽灵虚影
 * 用于赛道捕获后的游戏界面, 类似电子游戏的独立画面。
 */

#include "game_render.h"
#include "track/track_collision.h"
#include "physics/marble_physics.h"
#include "pixel_game/pixel_world.h"
#include "pixel_game/pixel_sprite.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "game";

/* Trophy zoom animation state */
#define GOAL_ANIM_DURATION_US  1200000  /* 1.2 seconds */
static uint64_t g_goal_anim_start_us = 0;
static bool     g_goal_anim_started  = false;

/* 桌面背景色 (深灰绿, 模拟游戏桌布) */
#define GAME_BG_R   (uint16_t)(((30 >> 3) << 11) | ((40 >> 2) << 5) | (28 >> 3))
#define GAME_GRID_R (uint16_t)(((38 >> 3) << 11) | ((48 >> 2) << 5) | (34 >> 3))

/* 幽灵轮廓渲染参数 (0-255 色彩空间) */
#define GHOST_COLOR_R   40
#define GHOST_COLOR_G   180
#define GHOST_COLOR_B   220
#define GHOST_ALPHA_EDGE   140   /* 边缘半透明度 */
#define GHOST_ALPHA_FILL   55    /* 内部填充半透明度 (更淡) */
#define SOBEL_THRESH       50    /* Sobel 边缘阈值 */

/* 掩码编码 */
#define MASK_EMPTY  0
#define MASK_EDGE   255
#define MASK_FILLED 128

/* 存储的物体轮廓 (Track 捕获时提取, 游戏渲染时使用) */
static object_contour_t g_contours[DETECTION_MAX_RESULTS];
static int              g_contour_count = 0;

/* ========================================================================
 * RGB565 alpha 混合
 * ======================================================================== */
static inline uint16_t blend_rgb565(uint16_t bg, int r, int g, int b, int alpha)
{
    int ia = 255 - alpha;
    int br = ((bg >> 11) & 0x1F) * 255 / 31;
    int bg_r = ((bg >> 5)  & 0x3F) * 255 / 63;
    int bb = ( bg        & 0x1F) * 255 / 31;
    int out_r = (r * alpha + br * ia) / 255;
    int out_g = (g * alpha + bg_r * ia) / 255;
    int out_b = (b * alpha + bb * ia) / 255;
    return (uint16_t)(((out_r >> 3) << 11) | ((out_g >> 2) << 5) | (out_b >> 3));
}

static inline uint16_t bg_color_at(int x, int y)
{
    int on_grid = ((y & 31) == 0);
    int px_on = on_grid || ((x & 31) == 0);
    return px_on ? GAME_GRID_R : GAME_BG_R;
}

/* ========================================================================
 * 坐标映射
 * ======================================================================== */
static inline int map_to_buf_x(int map_x) { return map_x * 512 / 640 + 64; }
static inline int map_to_buf_y(int map_y) { return map_y * 512 / 640; }

/* ========================================================================
 * RGB565 单像素 → 灰度
 * ======================================================================== */
static inline uint8_t rgb565_to_gray(uint16_t p)
{
    int r5 = (p >> 11) & 0x1F;
    int g6 = (p >> 5)  & 0x3F;
    int b5 =  p        & 0x1F;
    return (uint8_t)(((r5 * 77 * 255 / 31) + (g6 * 150 * 255 / 63) + (b5 * 29 * 255 / 31)) >> 8);
}

/* ========================================================================
 * 3×3 Sobel 边缘检测 (定点)
 * ======================================================================== */
static void sobel_edges(const uint8_t *gray, int w, int h,
                        uint8_t *edge_out, int threshold)
{
    memset(edge_out, 0, (size_t)w * h);

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int p00 = gray[(y - 1) * w + (x - 1)];
            int p01 = gray[(y - 1) * w + x];
            int p02 = gray[(y - 1) * w + (x + 1)];
            int p10 = gray[y * w + (x - 1)];
            int p12 = gray[y * w + (x + 1)];
            int p20 = gray[(y + 1) * w + (x - 1)];
            int p21 = gray[(y + 1) * w + x];
            int p22 = gray[(y + 1) * w + (x + 1)];

            int gx = -p00 + p02 - (p10 << 1) + (p12 << 1) - p20 + p22;
            int gy = -p00 - (p01 << 1) - p02 + p20 + (p21 << 1) + p22;
            int mag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);

            if (mag > threshold) {
                edge_out[y * w + x] = MASK_EDGE;
            }
        }
    }
}

/* ========================================================================
 * 形态学闭运算 (dilate → erode): 闭合边缘小缺口
 * 在独立的临时缓冲区上操作, 结果写回 mask
 * ======================================================================== */
static void morph_close(uint8_t *mask, int w, int h)
{
    uint8_t tmp[CONTOUR_MAX_DIM * CONTOUR_MAX_DIM];

    /* 1. Dilate: 若 8 邻域有边缘, 则设为边缘 */
    memcpy(tmp, mask, (size_t)w * h);
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            if (tmp[y * w + x] == MASK_EDGE) continue;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (tmp[(y + dy) * w + (x + dx)] == MASK_EDGE) {
                        mask[y * w + x] = MASK_EDGE;
                        goto next_dilate;
                    }
                }
            }
        next_dilate:;
        }
    }

    /* 2. Erode: 若 8 邻域有空隙, 则清除边缘 */
    memcpy(tmp, mask, (size_t)w * h);
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            if (tmp[y * w + x] != MASK_EDGE) continue;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (tmp[(y + dy) * w + (x + dx)] == MASK_EMPTY) {
                        mask[y * w + x] = MASK_EMPTY;
                        break;
                    }
                }
            }
        }
    }
}

/* ========================================================================
 * BFS 泛洪填充: 从掩码中心向外填充, 遇到边缘停止
 * 若填充触及掩码边界 → 轮廓不闭合 → 返回 false
 * ======================================================================== */
typedef struct { int16_t x, y; } bfs_point_t;
#define BFS_QUEUE_SZ  (CONTOUR_MAX_DIM * CONTOUR_MAX_DIM)

static bool flood_fill_interior(uint8_t *mask, int w, int h)
{
    static bfs_point_t queue[BFS_QUEUE_SZ];
    int head = 0, tail = 0;

    int cx = w / 2, cy = h / 2;

    /* 起点如果是边缘, 稍微偏移找内部 */
    if (mask[cy * w + cx] == MASK_EDGE) {
        int offsets[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
        bool found = false;
        for (int r = 2; r < w / 2 && !found; r++) {
            for (int d = 0; d < 4 && !found; d++) {
                int nx = cx + offsets[d][0] * r;
                int ny = cy + offsets[d][1] * r;
                if (nx >= 0 && nx < w && ny >= 0 && ny < h && mask[ny * w + nx] != MASK_EDGE) {
                    cx = nx; cy = ny; found = true;
                }
            }
        }
        if (!found) return false;  /* 全是边缘, 无法填充 */
    }

    queue[tail].x = (int16_t)cx;
    queue[tail].y = (int16_t)cy;
    tail++;

    bool closed = true;

    while (head < tail) {
        int16_t x = queue[head].x, y = queue[head].y;
        head++;

        if (x < 0 || x >= w || y < 0 || y >= h) {
            closed = false;    /* 泄漏到边界外 */
            continue;
        }
        uint8_t *p = mask + y * w + x;
        if (*p == MASK_EDGE || *p == MASK_FILLED) continue;

        *p = MASK_FILLED;

        if (tail + 4 > BFS_QUEUE_SZ) { closed = false; break; }

        queue[tail  ].x = (int16_t)(x + 1); queue[tail  ].y = y; tail++;
        queue[tail  ].x = (int16_t)(x - 1); queue[tail  ].y = y; tail++;
        queue[tail  ].x = x;                queue[tail  ].y = (int16_t)(y + 1); tail++;
        queue[tail  ].x = x;                queue[tail  ].y = (int16_t)(y - 1); tail++;
    }

    return closed;
}

/* ========================================================================
 * 从相机帧裁剪 ROI → 下采样 → 灰度 → Sobel → 闭运算 → 泛洪填充 → 存储
 * ======================================================================== */
static bool extract_single_contour(const camera_frame_t *frame,
                                   const detection_result_t *det,
                                   object_contour_t *contour)
{
    /* 1. 相机坐标下的 ROI */
    int cx1 = det->box_camera[0], cy1 = det->box_camera[1];
    int cx2 = det->box_camera[2], cy2 = det->box_camera[3];

    if (cx1 < 0) cx1 = 0;
    if (cy1 < 0) cy1 = 0;
    if (cx2 >= frame->width)  cx2 = frame->width  - 1;
    if (cy2 >= frame->height) cy2 = frame->height - 1;

    int roi_w = cx2 - cx1, roi_h = cy2 - cy1;
    if (roi_w < 8 || roi_h < 8) return false;

    /* 2. 掩码尺寸: 下采样 ~4×, 限制在 CONTOUR_MAX_DIM 内 */
    int mw = roi_w / 4, mh = roi_h / 4;
    if (mw < 8)  mw = 8;
    if (mh < 8)  mh = 8;
    if (mw > CONTOUR_MAX_DIM) mw = CONTOUR_MAX_DIM;
    if (mh > CONTOUR_MAX_DIM) mh = CONTOUR_MAX_DIM;

    contour->mask_w = mw;
    contour->mask_h = mh;

    /* 3. 下采样 + 灰度化: 相机帧 → 灰度掩码 */
    const uint16_t *src = (const uint16_t *)frame->buffer;
    int fw = frame->width;
    uint8_t gray_buf[CONTOUR_MAX_DIM * CONTOUR_MAX_DIM];
    uint32_t sx = ((uint32_t)roi_w << 16) / (uint32_t)mw;
    uint32_t sy = ((uint32_t)roi_h << 16) / (uint32_t)mh;

    for (int my = 0; my < mh; my++) {
        int cam_y = cy1 + ((my * sy) >> 16);
        if (cam_y >= frame->height) cam_y = frame->height - 1;
        const uint16_t *row = src + cam_y * fw;
        for (int mx = 0; mx < mw; mx++) {
            int cam_x = cx1 + ((mx * sx) >> 16);
            if (cam_x >= frame->width) cam_x = frame->width - 1;
            gray_buf[my * mw + mx] = rgb565_to_gray(row[cam_x]);
        }
    }

    /* 4. Sobel 边缘检测 */
    memset(contour->mask, 0, sizeof(contour->mask));
    sobel_edges(gray_buf, mw, mh, contour->mask, SOBEL_THRESH);

    /* 5. 闭运算: 填补边缘小缺口 */
    morph_close(contour->mask, mw, mh);

    /* 6. BFS 泛洪填充内部 */
    bool closed = flood_fill_interior(contour->mask, mw, mh);

    /* 7. 保存游戏坐标 */
    memcpy(contour->box_game, det->box, sizeof(contour->box_game));
    contour->valid = true;

    ESP_LOGI(TAG, "  Object contour: %dx%d mask, %s",
             mw, mh, closed ? "closed+filled" : "open (edge-only)");
    return true;
}

/* ========================================================================
 * 公开 API
 * ======================================================================== */

void game_extract_contours(const camera_frame_t *frame,
                           const detection_result_t *detections, int count)
{
    g_contour_count = 0;

    if (!frame || !detections || count <= 0) return;
    if (count > DETECTION_MAX_RESULTS) count = DETECTION_MAX_RESULTS;

    int extracted = 0;
    for (int i = 0; i < count; i++) {
        if (extract_single_contour(frame, &detections[i], &g_contours[extracted])) {
            extracted++;
        }
    }
    g_contour_count = extracted;
    ESP_LOGI(TAG, "Extracted %d object contours", extracted);
}

void game_render_frame(uint16_t *buf, int w, int h)
{
    /* 1. 填充背景 + 棋盘格纹理 */
    for (int y = 0; y < h; y++) {
        int on_grid = ((y & 31) == 0);
        for (int x = 0; x < w; x++) {
            int px_on = on_grid || ((x & 31) == 0);
            buf[y * w + x] = px_on ? GAME_GRID_R : GAME_BG_R;
        }
    }

    /* 2. 赛道墙壁 (红色) */
    if (track_is_built()) {
        track_render(buf, w, h, 0xF800);
    }

    /* 3. 弹珠 (3D 金属球) */
    if (track_is_built()) {
        marble_draw(buf, w, h);
    }

    /* 4. 检测物体 — 半透明幽灵虚影 (内部淡 + 边缘深) */
    for (int i = 0; i < g_contour_count; i++) {
        object_contour_t *c = &g_contours[i];
        if (!c->valid) continue;

        int bx1 = map_to_buf_x(c->box_game[0]);
        int by1 = map_to_buf_y(c->box_game[1]);
        int bx2 = map_to_buf_x(c->box_game[2]);
        int by2 = map_to_buf_y(c->box_game[3]);
        int bw  = bx2 - bx1, bh = by2 - by1;
        if (bw < 1 || bh < 1) continue;

        uint32_t sx = ((uint32_t)c->mask_w << 16) / (uint32_t)bw;
        uint32_t sy = ((uint32_t)c->mask_h << 16) / (uint32_t)bh;

        for (int py = 0; py < bh; py++) {
            int my = (py * sy) >> 16;
            if (my >= c->mask_h) my = c->mask_h - 1;
            int buf_y = by1 + py;
            if (buf_y < 0 || buf_y >= h) continue;

            uint8_t *mask_row = c->mask + my * c->mask_w;

            for (int px = 0; px < bw; px++) {
                int mx = (px * sx) >> 16;
                if (mx >= c->mask_w) mx = c->mask_w - 1;
                int buf_x = bx1 + px;
                if (buf_x < 0 || buf_x >= w) continue;

                uint8_t mval = mask_row[mx];
                if (mval == MASK_EDGE) {
                    /* 轮廓线: 与背景色混合, 覆盖下方赛道墙壁 */
                    buf[buf_y * w + buf_x] = blend_rgb565(
                        bg_color_at(buf_x, buf_y),
                        GHOST_COLOR_R, GHOST_COLOR_G, GHOST_COLOR_B,
                        GHOST_ALPHA_EDGE);
                } else if (mval == MASK_FILLED) {
                    /* 内部: 更淡的填充, 同样覆盖下方赛道墙壁 */
                    buf[buf_y * w + buf_x] = blend_rgb565(
                        bg_color_at(buf_x, buf_y),
                        GHOST_COLOR_R, GHOST_COLOR_G, GHOST_COLOR_B,
                        GHOST_ALPHA_FILL);
                }
            }
        }
    }
}

/* ========================================================================
 * Aim arrow helpers (cup capture mechanic)
 * ======================================================================== */
#define AIM_ARROW_COLOR  0xFFE0   /* bright yellow */
#define AIM_ARROW_LEN    45       /* arrow shaft length in pixels */
#define AIM_ARROW_HEAD   10       /* arrowhead wing length */

static inline void draw_pixel_safe(uint16_t *buf, int w, int h, int x, int y, uint16_t c)
{
    if (x >= 0 && x < w && y >= 0 && y < h) {
        buf[y * w + x] = c;
    }
}

static void draw_aim_arrow(uint16_t *buf, int w, int h,
                           int cx, int cy, float angle)
{
    /* shaft endpoint */
    int ex = cx + (int)(cosf(angle) * AIM_ARROW_LEN);
    int ey = cy + (int)(sinf(angle) * AIM_ARROW_LEN);

    /* Bresenham line for shaft */
    int dx = abs(ex - cx), dy = -abs(ey - cy);
    int sx = (cx < ex) ? 1 : -1;
    int sy = (cy < ey) ? 1 : -1;
    int err = dx + dy, e2;
    int x = cx, y = cy;
    while (1) {
        draw_pixel_safe(buf, w, h, x, y, AIM_ARROW_COLOR);
        if (x == ex && y == ey) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    /* arrowhead wings at ±150° from shaft direction */
    float wing_a1 = angle + 2.618f;  /* +150° */
    float wing_a2 = angle - 2.618f;  /* -150° */
    int w1x = ex + (int)(cosf(wing_a1) * AIM_ARROW_HEAD);
    int w1y = ey + (int)(sinf(wing_a1) * AIM_ARROW_HEAD);
    int w2x = ex + (int)(cosf(wing_a2) * AIM_ARROW_HEAD);
    int w2y = ey + (int)(sinf(wing_a2) * AIM_ARROW_HEAD);

    /* draw arrowhead wings */
    dx = abs(w1x - ex); dy = -abs(w1y - ey);
    sx = (ex < w1x) ? 1 : -1; sy = (ey < w1y) ? 1 : -1;
    err = dx + dy; x = ex; y = ey;
    while (1) {
        draw_pixel_safe(buf, w, h, x, y, AIM_ARROW_COLOR);
        if (x == w1x && y == w1y) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    dx = abs(w2x - ex); dy = -abs(w2y - ey);
    sx = (ex < w2x) ? 1 : -1; sy = (ey < w2y) ? 1 : -1;
    err = dx + dy; x = ex; y = ey;
    while (1) {
        draw_pixel_safe(buf, w, h, x, y, AIM_ARROW_COLOR);
        if (x == w2x && y == w2y) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

/* ========================================================================
 * Pixel Game Rendering (Terraria-style)
 *
 * Renders the full pixel game world to a 640x640 RGB565 buffer.
 * Pipeline: floor → tilemap → objects → aim arrow → marble.
 * ======================================================================== */

/* Floor colors (Terraria-style dirt/grass) */
#define FLOOR_COLOR1  ((uint16_t)(((34 >> 3) << 11) | ((50 >> 2) << 5) | (18 >> 3)))
#define FLOOR_COLOR2  ((uint16_t)(((40 >> 3) << 11) | ((55 >> 2) << 5) | (22 >> 3)))
#define FLOOR_COLOR3  ((uint16_t)(((28 >> 3) << 11) | ((42 >> 2) << 5) | (15 >> 3)))

static inline uint32_t floor_hash(unsigned int x, unsigned int y)
{
    uint32_t h = x * 374761393u + y * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

static uint16_t floor_color_at(int px, int py)
{
    int tile_x = px / 16;
    int tile_y = py / 16;
    uint32_t h = floor_hash((unsigned int)tile_x, (unsigned int)tile_y);

    uint16_t base;
    switch (h % 5) {
        case 0: base = FLOOR_COLOR1; break;
        case 1: base = FLOOR_COLOR2; break;
        case 2: base = FLOOR_COLOR1; break;
        case 3: base = FLOOR_COLOR3; break;
        default: base = FLOOR_COLOR2; break;
    }

    uint32_t hp = floor_hash((unsigned int)px, (unsigned int)py);
    int r = (int)(((base >> 11) & 0x1F) * 255 / 31);
    int g = (int)(((base >> 5)  & 0x3F) * 255 / 63);
    int b = (int)(( base        & 0x1F) * 255 / 31);

    int noise = (int)(hp % 13) - 6;
    r += noise; g += noise; b += noise;
    if (r < 0) { r = 0; } else if (r > 255) { r = 255; }
    if (g < 0) { g = 0; } else if (g > 255) { g = 255; }
    if (b < 0) { b = 0; } else if (b > 255) { b = 255; }

    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

void game_render_pixel_frame(uint16_t *buf, int w, int h)
{
    if (!buf || w < 640 || h < 640) return;

    pixel_world_t *world = pixel_world_get();

    /* 1. Fill floor background */
    for (int y = 0; y < GAME_MAP_PIXELS; y++) {
        for (int x = 0; x < GAME_MAP_PIXELS; x++) {
            buf[y * w + x] = floor_color_at(x, y);
        }
    }

    /* 2. Draw tilemap (walls, broken tiles) */
    if (world && pixel_world_is_built()) {
        for (int ty = 0; ty < GAME_MAP_TILES; ty++) {
            for (int tx = 0; tx < GAME_MAP_TILES; tx++) {
                tile_type_t t = (tile_type_t)world->tilemap[ty][tx];
                if (t == TILE_EMPTY) continue;

                const sprite_t *sp = sprite_get_tile(t);
                if (sp) {
                    sprite_blit(buf, w, sp, tx * GAME_TILE_SIZE, ty * GAME_TILE_SIZE);
                }
            }
        }

        /* 3. Draw game objects */
        for (int i = 0; i < world->object_count; i++) {
            game_object_t *obj = &world->objects[i];
            if (!obj->alive) continue;

            const sprite_t *sp = sprite_get_by_coco(obj->coco_id);
            if (!sp) {
                sp = sprite_get_obj(obj->type);
            }
            if (!sp) continue;

            int sx = obj->pixel_x - sp->w / 2;
            int sy = obj->pixel_y - sp->h / 2;
            sprite_blit_keyed_edgeblend(buf, w, sp, sx, sy, 0xF81F);
        }
    }

    /* 3b. Draw cup aim arrow */
    if (world && world->cup_aiming) {
        draw_aim_arrow(buf, w, h,
                       world->cup_aim_cx, world->cup_aim_cy,
                       world->cup_aim_angle);
    }

    /* 4. Draw marble (3D metal ball, 1:1 scale) */
    marble_draw_game(buf, w, h);

    /* 5. Goal reached: animate trophy scaling up */
    if (world && world->goal_reached) {
        const sprite_t *bottle = sprite_get_by_coco(39);
        if (bottle && bottle->pixels) {
            uint64_t now = esp_timer_get_time();
            if (!g_goal_anim_started) {
                g_goal_anim_start_us = now;
                g_goal_anim_started = true;
            }

            int64_t elapsed = (int64_t)(now - g_goal_anim_start_us);
            float t = (float)elapsed / (float)GOAL_ANIM_DURATION_US;
            if (t > 1.0f) t = 1.0f;

            /* Ease-out: 1 - (1-t)^2 */
            float ease = 1.0f - (1.0f - t) * (1.0f - t);

            /* Darken screen: 100% → 40% brightness */
            int dark = (int)(255.0f - 153.0f * ease);  /* 255→102 */
            for (int i = 0; i < GAME_MAP_PIXELS * GAME_MAP_PIXELS; i++) {
                uint16_t p = buf[i];
                int r = ((p >> 11) & 0x1F) * dark / 255;
                int g = ((p >> 5)  & 0x3F) * dark / 255;
                int b = ( p        & 0x1F) * dark / 255;
                buf[i] = (uint16_t)((r << 11) | (g << 5) | b);
            }

            /* Trophy scales from tiny to 160x240 */
            int big_w = (int)(160.0f * ease);
            int big_h = (int)(240.0f * ease);
            if (big_w < 6) big_w = 6;
            if (big_h < 9) big_h = 9;
            int cx = (GAME_MAP_PIXELS - big_w) / 2;
            int cy = (GAME_MAP_PIXELS - big_h) / 2;
            sprite_blit_keyed_scaled(buf, w, bottle, cx, cy, big_w, big_h, 0xF81F);
        }
    } else {
        g_goal_anim_started = false;
    }
}
