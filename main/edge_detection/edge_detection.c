/**
 * Canny Edge Detection — 纯 C 实现, 定点整数数学
 *
 * 5 阶段流水线:
 *   1. RGB565 → 灰度 (LUT 加速)
 *   2. 高斯模糊 3x3 (可分离 1D 卷积, 核 [1,2,1]/4)
 *   3. Sobel 梯度 (Gx, Gy) + 幅值 |Gx|+|Gy| + 方向量化 (4扇区)
 *   4. 非极大值抑制 (NMS)
 *   5. 双阈值 + 滞后边缘跟踪
 *
 * 全部使用整数算术, 内层循环无浮点运算, 无 sqrt, 无 atan2。
 */

#include "edge_detection.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "canny";

/* ---- 工作缓冲区 (PSRAM, 一次性分配, 复用) ---- */
static uint8_t  *g_gray     = NULL;  /* 灰度图       w*h       */
static uint8_t  *g_temp     = NULL;  /* 高斯/通用暂存 w*h       */
static uint16_t *g_mag      = NULL;  /* 梯度幅值     w*h*2     */
static uint8_t  *g_dir      = NULL;  /* 方向扇区     w*h       */
static uint8_t  *g_ds_buf   = NULL;  /* 下采样缓冲   400x320x2 */
static uint8_t  *g_edge_map = NULL;  /* 边缘图输出   400x320   */
static int       g_buf_w    = 0;
static int       g_buf_h    = 0;

/* ---- RGB565→Gray LUT ---- */
static uint8_t r_lut[32];
static uint8_t g_lut[64];
static uint8_t b_lut[32];
static int    g_lut_ready = 0;

#define STRONG  255
#define WEAK    128
#define ABS(x)  ((x) < 0 ? -(x) : (x))

/* ========================================================================
 * LUT 初始化
 * ======================================================================== */
static void build_gray_luts(void)
{
    if (g_lut_ready) return;
    /* Y = 0.299*R + 0.587*G + 0.114*B, 整数: (R8*77+G8*150+B8*29) >> 8 */
    for (int i = 0; i < 32; i++) {
        uint8_t r8 = (i * 255 + 15) / 31;
        r_lut[i] = (uint8_t)(((uint16_t)r8 * 77) >> 8);
    }
    for (int i = 0; i < 64; i++) {
        uint8_t g8 = (i * 255 + 31) / 63;
        g_lut[i] = (uint8_t)(((uint16_t)g8 * 150) >> 8);
    }
    for (int i = 0; i < 32; i++) {
        uint8_t b8 = (i * 255 + 15) / 31;
        b_lut[i] = (uint8_t)(((uint16_t)b8 * 29) >> 8);
    }
    g_lut_ready = 1;
}

/* ========================================================================
 * Stage 1: RGB565 → 灰度
 * ======================================================================== */
static void rgb565_to_gray(const uint8_t *rgb565, int w, int h)
{
    const uint16_t *src = (const uint16_t *)rgb565;
    for (int i = 0; i < w * h; i++) {
        uint16_t p = src[i];
        uint8_t r5 = (p >> 11) & 0x1F;
        uint8_t g6 = (p >> 5)  & 0x3F;
        uint8_t b5 =  p        & 0x1F;
        g_gray[i] = r_lut[r5] + g_lut[g6] + b_lut[b5];
    }
}

/* ========================================================================
 * Stage 2: 高斯模糊 3x3 (可分离 1D 卷积)
 *
 * 核 [1, 2, 1] / 4, 先水平再垂直。
 * 边界处理: 复制边缘像素。
 * ======================================================================== */
#if 0  /* 保留备用: 原始分辨率 Canny 时启用 */
static void gaussian_blur(int w, int h)
{
    /* ---- 水平方向 ---- */
    for (int y = 0; y < h; y++) {
        const uint8_t *row = g_gray + y * w;
        uint8_t *dst = g_temp + y * w;

        /* 左边界 */
        dst[0] = (uint8_t)((row[0] * 3 + row[1]) >> 2);
        /* 内部 */
        for (int x = 1; x < w - 1; x++) {
            dst[x] = (uint8_t)((row[x-1] + row[x] * 2 + row[x+1]) >> 2);
        }
        /* 右边界 */
        dst[w-1] = (uint8_t)((row[w-2] + row[w-1] * 3) >> 2);
    }

    /* ---- 垂直方向 ---- */
    for (int x = 0; x < w; x++) {
        /* 上边界 */
        g_gray[x] = (uint8_t)((g_temp[x] * 3 + g_temp[w + x]) >> 2);
        /* 内部 */
        for (int y = 1; y < h - 1; y++) {
            int idx = y * w + x;
            g_gray[idx] = (uint8_t)((g_temp[idx - w] + g_temp[idx] * 2 + g_temp[idx + w]) >> 2);
        }
        /* 下边界 */
        int idx = (h - 1) * w + x;
        g_gray[idx] = (uint8_t)((g_temp[idx - w] + g_temp[idx] * 3) >> 2);
    }
}
#endif  /* gaussian_blur */

/* ========================================================================
 * Stage 3: Sobel 梯度 + 幅值 + 方向量化
 *
 * Gx kernel: [-1  0  1;  -2  0  2;  -1  0  1]
 * Gy kernel: [-1 -2 -1;   0  0  0;   1  2  1]
 *
 * 幅值 = |Gx| + |Gy|  (快速近似, 误差 < 3%)
 * 方向: 4 扇区量化, 无 atan2
 *
 * 扇区 0 (≈0°/180°): 水平梯度 → 比较左右邻居
 * 扇区 1 (≈45°/225°): 对角 → 比较 (x-1,y+1), (x+1,y-1)
 * 扇区 2 (≈90°/270°): 垂直梯度 → 比较上下邻居
 * 扇区 3 (≈135°/315°): 对角 → 比较 (x-1,y-1), (x+1,y+1)
 * ======================================================================== */
static void sobel_gradient(int w, int h)
{
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int idx = y * w + x;

            /* 获取 3x3 邻域灰度值 */
            int p00 = g_gray[idx - w - 1];
            int p01 = g_gray[idx - w];
            int p02 = g_gray[idx - w + 1];
            int p10 = g_gray[idx - 1];
            int p12 = g_gray[idx + 1];
            int p20 = g_gray[idx + w - 1];
            int p21 = g_gray[idx + w];
            int p22 = g_gray[idx + w + 1];

            /* Sobel Gx, Gy */
            int gx = -p00 + p02 - (p10 << 1) + (p12 << 1) - p20 + p22;
            int gy = -p00 - (p01 << 1) - p02 + p20 + (p21 << 1) + p22;

            /* 幅值 = |gx| + |gy|, 限制到 0-1023 */
            int ax = ABS(gx);
            int ay = ABS(gy);
            int mag = ax + ay;
            if (mag > 1023) mag = 1023;
            g_mag[idx] = (uint16_t)mag;

            /* 方向量化 (4 扇区) */
            uint8_t sector;
            if (ax > ay) {
                if (ax > (ay << 1))
                    sector = 0;   /* 近似 0° 或 180° */
                else
                    sector = 1;   /* 近似 45° 或 225° */
            } else {
                if (ay > (ax << 1))
                    sector = 2;   /* 近似 90° 或 270° */
                else
                    sector = 3;   /* 近似 135° 或 315° */
            }
            g_dir[idx] = sector;
        }
    }

    /* 边界清零 */
    for (int y = 0; y < h; y++) {
        g_mag[y * w] = 0;
        g_mag[y * w + w - 1] = 0;
        g_dir[y * w] = 0;
        g_dir[y * w + w - 1] = 0;
    }
    for (int x = 1; x < w - 1; x++) {
        g_mag[x] = 0;
        g_mag[(h - 1) * w + x] = 0;
        g_dir[x] = 0;
        g_dir[(h - 1) * w + x] = 0;
    }
}

/* ========================================================================
 * Stage 4: 非极大值抑制 (NMS)
 *
 * 沿梯度方向的两个邻居比较幅值,
 * 如当前像素不是局部最大值则抑制 (设为 0)。
 * ======================================================================== */
static void non_max_suppression(uint8_t *edge_map, int w, int h, int low_thresh)
{
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int idx = y * w + x;
            uint16_t mag = g_mag[idx];

            if (mag < (uint16_t)(low_thresh << 2)) {  /* 缩放到 mag 域 (0-1023) */
                edge_map[idx] = 0;
                continue;
            }

            int n1, n2;
            switch (g_dir[idx]) {
            case 0:  /* ← → */
                n1 = g_mag[idx - 1];
                n2 = g_mag[idx + 1];
                break;
            case 1:  /* ↖ ↘: (x-1,y+1), (x+1,y-1) */
                n1 = g_mag[idx - 1 + w];
                n2 = g_mag[idx + 1 - w];
                break;
            case 2:  /* ↑ ↓ */
                n1 = g_mag[idx - w];
                n2 = g_mag[idx + w];
                break;
            case 3:  /* ↗ ↙: (x+1,y+1), (x-1,y-1) */
            default:
                n1 = g_mag[idx + 1 + w];
                n2 = g_mag[idx - 1 - w];
                break;
            }

            if (mag >= n1 && mag >= n2) {
                edge_map[idx] = (uint8_t)(mag >> 2);  /* 缩放到 0-255 用于阈值判断 */
            } else {
                edge_map[idx] = 0;
            }
        }
    }
}

/* ========================================================================
 * Stage 5: 双阈值 + 滞后边缘跟踪
 *
 * Pass 1: mag > high → STRONG(255), mag > low → WEAK(128), else 0
 * Pass 2-N: 迭代传播 — 与 STRONG 相邻的 WEAK 升级为 STRONG
 * 最终: WEAK → 0 (抑制孤立弱边缘)
 * ======================================================================== */
static void hysteresis(uint8_t *edge_map, int w, int h,
                       int low_thresh, int high_thresh)
{
    /* Pass 1: 双阈值分类 */
    for (int i = 0; i < w * h; i++) {
        uint8_t v = edge_map[i];
        if (v == 0) continue;
        if (v >= (uint8_t)high_thresh) {
            edge_map[i] = STRONG;
        } else if (v >= (uint8_t)low_thresh) {
            edge_map[i] = WEAK;
        } else {
            edge_map[i] = 0;
        }
    }

    /* Pass 2-N: 迭代传播 */
    int changed;
    do {
        changed = 0;
        for (int y = 1; y < h - 1; y++) {
            for (int x = 1; x < w - 1; x++) {
                int idx = y * w + x;
                if (edge_map[idx] != WEAK) continue;

                /* 检查 8 邻域是否有 STRONG */
                if (edge_map[idx - w - 1] == STRONG ||
                    edge_map[idx - w]     == STRONG ||
                    edge_map[idx - w + 1] == STRONG ||
                    edge_map[idx - 1]     == STRONG ||
                    edge_map[idx + 1]     == STRONG ||
                    edge_map[idx + w - 1] == STRONG ||
                    edge_map[idx + w]     == STRONG ||
                    edge_map[idx + w + 1] == STRONG) {
                    edge_map[idx] = STRONG;
                    changed = 1;
                }
            }
        }
    } while (changed);

    /* 清除剩余 WEAK 像素 */
    for (int i = 0; i < w * h; i++) {
        if (edge_map[i] == WEAK) {
            edge_map[i] = 0;
        }
    }
}

/* ========================================================================
 * 公开接口
 * ======================================================================== */

esp_err_t edge_detect_init(void)
{
    build_gray_luts();

    /* 按最大预期分辨率 (400x320) 分配, 实际使用按入参裁剪 */
    int max_w = 400, max_h = 320;
    size_t npixels = (size_t)max_w * max_h;

    if (g_gray == NULL) {
        g_gray = (uint8_t *)heap_caps_calloc(npixels, 1,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (g_temp == NULL) {
        g_temp = (uint8_t *)heap_caps_calloc(npixels, 1,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (g_mag == NULL) {
        g_mag = (uint16_t *)heap_caps_calloc(npixels, 2,
                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (g_dir == NULL) {
        g_dir = (uint8_t *)heap_caps_calloc(npixels, 1,
                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (g_ds_buf == NULL) {
        g_ds_buf = (uint8_t *)heap_caps_calloc(max_w * max_h, 2,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (g_edge_map == NULL) {
        g_edge_map = (uint8_t *)heap_caps_calloc(npixels, 1,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }

    if (!g_gray || !g_temp || !g_mag || !g_dir || !g_ds_buf || !g_edge_map) {
        ESP_LOGE(TAG, "PSRAM allocation FAILED");
        edge_detect_deinit();
        return ESP_ERR_NO_MEM;
    }

    g_buf_w = max_w;
    g_buf_h = max_h;

    ESP_LOGI(TAG, "Init OK (max %dx%d, buffers %zu KB)",
             max_w, max_h,
             (npixels * (1 + 1 + 2 + 1)) / 1024);
    return ESP_OK;
}

void edge_detect_deinit(void)
{
    if (g_gray)  { heap_caps_free(g_gray);  g_gray  = NULL; }
    if (g_temp)  { heap_caps_free(g_temp);  g_temp  = NULL; }
    if (g_mag)   { heap_caps_free(g_mag);   g_mag   = NULL; }
    if (g_dir)   { heap_caps_free(g_dir);   g_dir   = NULL; }
    if (g_ds_buf)  { heap_caps_free(g_ds_buf);  g_ds_buf   = NULL; }
    if (g_edge_map){ heap_caps_free(g_edge_map);g_edge_map = NULL; }
    g_buf_w = g_buf_h = 0;
}

uint8_t *edge_get_downscale_buf(void)
{
    return g_ds_buf;
}

uint8_t *edge_get_edge_map_buf(void)
{
    return g_edge_map;
}

void edge_downscale_half(const uint8_t *src, int sw, int sh, uint8_t *dst)
{
    const uint16_t *s = (const uint16_t *)src;
    uint16_t *d = (uint16_t *)dst;
    int dw = sw / 2, dh = sh / 2;
    for (int y = 0; y < dh; y++) {
        int sy = y * 2;
        for (int x = 0; x < dw; x++) {
            int sx = x * 2;
            uint16_t p00 = s[sy * sw + sx];
            uint16_t p01 = s[sy * sw + sx + 1];
            uint16_t p10 = s[(sy + 1) * sw + sx];
            uint16_t p11 = s[(sy + 1) * sw + sx + 1];
            /* 分量平均: R(5bit), G(6bit), B(5bit) */
            int r = ((p00 >> 11) & 0x1F) + ((p01 >> 11) & 0x1F) +
                    ((p10 >> 11) & 0x1F) + ((p11 >> 11) & 0x1F);
            int g = ((p00 >> 5) & 0x3F) + ((p01 >> 5) & 0x3F) +
                    ((p10 >> 5) & 0x3F) + ((p11 >> 5) & 0x3F);
            int b = (p00 & 0x1F) + (p01 & 0x1F) + (p10 & 0x1F) + (p11 & 0x1F);
            d[y * dw + x] = (uint16_t)(((r >> 2) << 11) | ((g >> 2) << 5) | (b >> 2));
        }
    }
}

esp_err_t edge_detect_run(const uint8_t *rgb565, int w, int h,
                          uint8_t *edge_map_out,
                          int low_thresh, int high_thresh)
{
    if (!rgb565 || !edge_map_out || w < 3 || h < 3) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_gray || w > g_buf_w || h > g_buf_h) {
        ESP_LOGE(TAG, "Buffer overflow: %dx%d > %dx%d", w, h, g_buf_w, g_buf_h);
        return ESP_ERR_NO_MEM;
    }

    if (!g_lut_ready) build_gray_luts();

    /* 清除上一帧的边缘图 (防止边界残留) */
    memset(edge_map_out, 0, (size_t)w * h);

    /* ---- Stage 1: RGB565 → Gray ---- */
    rgb565_to_gray(rgb565, w, h);

    /* NOTE: 跳过 Gaussian Blur — 2x 下采样已提供足够的抗锯齿平滑。
     * 如需在原始分辨率下使用 Canny, 可取消注释下面一行: */
    /* gaussian_blur(w, h); */

    /* ---- Stage 2 & 3: Sobel + Magnitude + Direction ---- */
    sobel_gradient(w, h);

    /* ---- Stage 4: NMS ---- */
    non_max_suppression(edge_map_out, w, h, low_thresh);

    /* ---- Stage 5: Hysteresis ---- */
    hysteresis(edge_map_out, w, h, low_thresh, high_thresh);

    /* 诊断: 统计边缘像素数量 */
    int edge_count = 0;
    for (int i = 0; i < w * h; i++) {
        if (edge_map_out[i] == STRONG) edge_count++;
    }
    ESP_LOGI(TAG, "Edges found: %d pixels (%.1f%%)", edge_count,
             100.0f * edge_count / (w * h));

    return ESP_OK;
}
