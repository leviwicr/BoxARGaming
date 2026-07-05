/**
 * 轨道碰撞模块 — 从 Canny 边缘检测结果构建游戏赛道
 *
 * 坐标映射:
 *   相机帧 800×640 → 边缘图 400×320 (÷2)
 *   游戏地图: 相机中心 640×640 裁剪 → 边缘图 x∈[40,360), y∈[0,320)
 *   碰撞图: 320×320 边缘区域 → 最近邻上采样 → 640×640
 *
 * 碰撞检测: 采样弹珠圆周 8 点 + 中心点, 命中墙壁则反射
 * 法线估计: Sobel 3×3 梯度 (预计算, 碰撞图构建时生成)
 */

#include "track_collision.h"
#include "config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "track";

/* ---- 碰撞图: 640×640 uint8, 0=free, 1=wall, 2=wall_dilated ---- */
static uint8_t *g_collision_map = NULL;
static int8_t  *g_normal_x = NULL;    /* Sobel Gx, -128..127 */
static int8_t  *g_normal_y = NULL;    /* Sobel Gy, -128..127 */
static bool     g_built = false;

#define COL_MAP_W  640
#define COL_MAP_H  640

/* ---- 游戏区域在边缘图中的位置 ---- */
/* 相机 800×640 → 边缘图 400×320, 游戏地图=中心 640×640
 * 边缘图中的游戏区域: x∈[40, 360), y∈[0, 320), 大小 320×320 */
#define EDGE_GAME_X0    40   /* 边缘图中游戏区域左上角X */
#define EDGE_GAME_Y0    0    /* 边缘图中游戏区域左上角Y */
#define EDGE_GAME_W     320  /* 游戏区域在边缘图中的宽度 */
#define EDGE_GAME_H     320  /* 游戏区域在边缘图中的高度 */

/* ---- 碰撞检测用 ---- */
#define SAMPLE_POINTS   8     /* 圆周采样点数 */
#define WALL_SEARCH_R   2     /* 法线搜索半径 */

esp_err_t track_collision_init(void)
{
    if (g_collision_map) return ESP_OK;  /* 已初始化 */

    size_t map_bytes = COL_MAP_W * COL_MAP_H;  /* 640*640 = 409600 */
    g_collision_map = (uint8_t *)heap_caps_calloc(map_bytes, 1,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_normal_x = (int8_t *)heap_caps_calloc(map_bytes, 1,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_normal_y = (int8_t *)heap_caps_calloc(map_bytes, 1,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!g_collision_map || !g_normal_x || !g_normal_y) {
        ESP_LOGE(TAG, "PSRAM alloc failed");
        track_collision_deinit();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Init OK (%d KB for collision map + normals)",
             (int)(map_bytes * 3 / 1024));
    return ESP_OK;
}

void track_collision_deinit(void)
{
    if (g_collision_map) { heap_caps_free(g_collision_map); g_collision_map = NULL; }
    if (g_normal_x)     { heap_caps_free(g_normal_x);      g_normal_x = NULL; }
    if (g_normal_y)     { heap_caps_free(g_normal_y);      g_normal_y = NULL; }
    g_built = false;
}

/* ---- 形态学膨胀 (3×3, 使墙壁变厚, 用于碰撞) ---- */
static void dilate_walls(uint8_t *map, int w, int h)
{
    uint8_t *tmp = (uint8_t *)heap_caps_malloc(w * h,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tmp) return;

    memcpy(tmp, map, w * h);

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            if (tmp[y * w + x]) continue;
            /* 如果 8 邻域有墙壁, 填充 3×3 区域 */
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (tmp[(y + dy) * w + (x + dx)]) {
                        map[y * w + x] = 2;  /* 膨胀填充, 标记为 dilated */
                        goto next_pixel;
                    }
                }
            }
        next_pixel:;
        }
    }

    heap_caps_free(tmp);
}

/* ---- Sobel 法线预计算 ---- */
static void compute_normals(uint8_t *map, int w, int h)
{
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int idx = y * w + x;

            if (!map[idx]) {
                g_normal_x[idx] = 0;
                g_normal_y[idx] = 0;
                continue;
            }

            /* Sobel: 用 map 值 (0/1/2) 计算梯度, 法线指向低值侧 (free space) */
            int gx = -map[idx - w - 1] + map[idx - w + 1]
                     - (map[idx - 1]   << 1) + (map[idx + 1]   << 1)
                     - map[idx + w - 1] + map[idx + w + 1];

            int gy = -map[idx - w - 1] - (map[idx - w] << 1) - map[idx - w + 1]
                     + map[idx + w - 1] + (map[idx + w] << 1) + map[idx + w + 1];

            /* 存储有符号梯度, 法线方向 = -G (指向低值=自由空间) */
            int8_t nx = (int8_t)(-gx / 2);  /* scale down to fit int8 */
            int8_t ny = (int8_t)(-gy / 2);

            g_normal_x[idx] = nx;
            g_normal_y[idx] = ny;
        }
    }
}

esp_err_t track_build_from_edges(const uint8_t *edge_map, int ew, int eh)
{
    if (!g_collision_map || !edge_map) return ESP_ERR_INVALID_STATE;
    if (ew < EDGE_GAME_X0 + EDGE_GAME_W || eh < EDGE_GAME_Y0 + EDGE_GAME_H) {
        ESP_LOGE(TAG, "Edge map too small: %dx%d (need >= %dx%d)",
                 ew, eh, EDGE_GAME_X0 + EDGE_GAME_W, EDGE_GAME_Y0 + EDGE_GAME_H);
        return ESP_ERR_INVALID_ARG;
    }

    /* 清零碰撞图 */
    memset(g_collision_map, 0, COL_MAP_W * COL_MAP_H);
    memset(g_normal_x, 0, COL_MAP_W * COL_MAP_H);
    memset(g_normal_y, 0, COL_MAP_W * COL_MAP_H);

    /* 从边缘图中裁剪游戏区域 (320×320) 并最近邻上采样 ×2 → 640×640 */
    for (int cy = 0; cy < COL_MAP_H; cy++) {
        int ey = EDGE_GAME_Y0 + cy / 2;
        if (ey >= eh) ey = eh - 1;

        for (int cx = 0; cx < COL_MAP_W; cx++) {
            int ex = EDGE_GAME_X0 + cx / 2;
            if (ex >= ew) ex = ew - 1;

            if (edge_map[ey * ew + ex] == 255) {
                g_collision_map[cy * COL_MAP_W + cx] = 1;
            }
        }
    }

    /* 膨胀: 让墙壁线变厚, 提高碰撞检测可靠性 */
    dilate_walls(g_collision_map, COL_MAP_W, COL_MAP_H);

    /* 预计算法线 */
    compute_normals(g_collision_map, COL_MAP_W, COL_MAP_H);

    g_built = true;

    /* 统计 */
    int wall_count = 0;
    for (int i = 0; i < COL_MAP_W * COL_MAP_H; i++) {
        if (g_collision_map[i]) wall_count++;
    }
    ESP_LOGI(TAG, "Track built: %d wall pixels (%.1f%%)",
             wall_count, 100.0f * wall_count / (COL_MAP_W * COL_MAP_H));

    return ESP_OK;
}

bool track_is_built(void)
{
    return g_built;
}

bool track_is_wall(int game_x, int game_y)
{
    if (!g_built || !g_collision_map) return false;
    if (game_x < 0 || game_x >= COL_MAP_W) return true;  /* 地图外=墙壁 */
    if (game_y < 0 || game_y >= COL_MAP_H) return true;
    return g_collision_map[game_y * COL_MAP_W + game_x] != 0;
}

bool track_get_normal(int game_x, int game_y, float *nx, float *ny)
{
    if (!g_built || !g_normal_x || !g_normal_y) {
        *nx = 0; *ny = 0;
        return false;
    }

    /* 钳到范围内 */
    if (game_x < 1) game_x = 1;
    if (game_x >= COL_MAP_W - 1) game_x = COL_MAP_W - 2;
    if (game_y < 1) game_y = 1;
    if (game_y >= COL_MAP_H - 1) game_y = COL_MAP_H - 2;

    int idx = game_y * COL_MAP_W + game_x;

    /* 搜索附近墙壁像素的法线 (WALL_SEARCH_R 半径内) */
    float sum_nx = 0, sum_ny = 0;
    int count = 0;

    for (int dy = -WALL_SEARCH_R; dy <= WALL_SEARCH_R; dy++) {
        for (int dx = -WALL_SEARCH_R; dx <= WALL_SEARCH_R; dx++) {
            int si = (game_y + dy) * COL_MAP_W + (game_x + dx);
            if (si < 0 || si >= COL_MAP_W * COL_MAP_H) continue;
            if (g_collision_map[si]) {
                sum_nx += g_normal_x[si];
                sum_ny += g_normal_y[si];
                count++;
            }
        }
    }

    if (count == 0) {
        *nx = 0; *ny = 0;
        return false;
    }

    /* 归一化 */
    float len = sqrtf(sum_nx * sum_nx + sum_ny * sum_ny);
    if (len < 1.0f) {
        *nx = 0; *ny = 0;
        return false;
    }

    *nx = sum_nx / len;
    *ny = sum_ny / len;
    return true;
}

void track_render(uint16_t *buf, int buf_w, int buf_h, uint16_t wall_color)
{
    if (!g_built || !g_collision_map || !buf) return;

    /* 碰撞图 640×640 → 预览游戏区 512×512 (offset 64,0 in 640×512)
     * scale = 512/640 = 0.8
     * buf_x = col_x * 512/640 + 64
     * buf_y = col_y * 512/640
     */
    for (int cy = 1; cy < COL_MAP_H - 1; cy++) {
        int buf_y = cy * 512 / COL_MAP_H;
        if (buf_y < 0 || buf_y >= buf_h) continue;

        for (int cx = 1; cx < COL_MAP_W - 1; cx++) {
            if (!g_collision_map[cy * COL_MAP_W + cx]) continue;

            int buf_x = cx * 512 / COL_MAP_W + 64;
            if (buf_x < 0 || buf_x >= buf_w) continue;

            /* 画加粗线: 渲染碰撞像素及其直接邻居 */
            for (int dy = -1; dy <= 1; dy++) {
                int by = buf_y + dy;
                if (by < 0 || by >= buf_h) continue;
                for (int dx = -1; dx <= 1; dx++) {
                    int bx = buf_x + dx;
                    if (bx < 0 || bx >= buf_w) continue;
                    buf[by * buf_w + bx] = wall_color;
                }
            }
        }
    }
}
