/**
 * 图像预处理模块 —— 直方图均衡化 / 锐化 / 对比度拉伸 / 去噪
 *
 * 所有算法均操作 RGB888 格式, 使用 PSRAM 共享暂存缓冲区。
 */

#include "image_processing.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dl_image_process.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

static const char *TAG = "imgproc";

/* 共享暂存缓冲区: 800x640 RGB888 = 1,536,000 bytes (~1.46 MB PSRAM) */
static uint8_t *g_temp_buf = NULL;
static size_t   g_temp_size = 0;

/* ========================================================================
 * 生命周期
 * ======================================================================== */

esp_err_t preprocessing_init(void)
{
    if (g_temp_buf) return ESP_OK;

    g_temp_size = 800 * 640 * 3;  /* 最大帧尺寸 RGB888 */
    g_temp_buf = (uint8_t *)heap_caps_calloc(1, g_temp_size,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_temp_buf) {
        ESP_LOGE(TAG, "Failed to allocate temp buffer (%u bytes)", (unsigned)g_temp_size);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Temp buffer allocated: %u bytes in PSRAM", (unsigned)g_temp_size);
    return ESP_OK;
}

void preprocessing_deinit(void)
{
    if (g_temp_buf) {
        heap_caps_free(g_temp_buf);
        g_temp_buf = NULL;
        g_temp_size = 0;
        ESP_LOGI(TAG, "Temp buffer freed");
    }
}

uint8_t *preprocessing_get_temp_buffer(void)
{
    return g_temp_buf;
}

/* ========================================================================
 * 工具: RGB888 像素分量提取/写入
 * ======================================================================== */

static inline uint8_t *pixel_ptr(uint8_t *img, int x, int y, int row_step)
{
    return img + y * row_step + x * 3;
}

static inline void get_rgb(const uint8_t *p, uint8_t &r, uint8_t &g, uint8_t &b)
{
    r = p[0]; g = p[1]; b = p[2];
}

static inline void set_rgb(uint8_t *p, uint8_t r, uint8_t g, uint8_t b)
{
    p[0] = r; p[1] = g; p[2] = b;
}

static inline uint8_t rgb_to_y(uint8_t r, uint8_t g, uint8_t b)
{
    /* ITU-R BT.601: Y = 0.299R + 0.587G + 0.114B */
    return (uint8_t)((299 * r + 587 * g + 114 * b) / 1000);
}

/* ========================================================================
 * 1. 直方图均衡化 (Y 通道)
 * ======================================================================== */

esp_err_t histogram_equalize(dl::image::img_t &img)
{
    if (img.pix_type != dl::image::DL_IMAGE_PIX_TYPE_RGB888) {
        ESP_LOGE(TAG, "hist_eq: requires RGB888 input");
        return ESP_ERR_INVALID_ARG;
    }

    int w = img.width, h = img.height;
    int row_step = img.row_step();
    uint8_t *data = (uint8_t *)img.data;
    int total = w * h;

    /* 计算 Y 通道直方图 */
    uint32_t hist[256] = {0};
    for (int y = 0; y < h; y++) {
        uint8_t *row = data + y * row_step;
        for (int x = 0; x < w; x++) {
            uint8_t *p = row + x * 3;
            uint8_t Y = rgb_to_y(p[0], p[1], p[2]);
            hist[Y]++;
        }
    }

    /* 跳过纯色图像 (min == max) */
    int y_min = 0, y_max = 255;
    while (y_min < 255 && hist[y_min] == 0) y_min++;
    while (y_max > 0 && hist[y_max] == 0) y_max--;
    if (y_min >= y_max) return ESP_OK;

    /* CDF → LUT: 均衡化映射 */
    uint8_t lut[256];
    uint32_t cdf = 0;
    float scale = 255.0f / (float)total;
    for (int i = 0; i < 256; i++) {
        cdf += hist[i];
        lut[i] = (uint8_t)((float)cdf * scale + 0.5f);
    }

    /* 应用: 每个像素按 Y 比值缩放 RGB */
    for (int y = 0; y < h; y++) {
        uint8_t *row = data + y * row_step;
        for (int x = 0; x < w; x++) {
            uint8_t *p = row + x * 3;
            uint8_t Y_old = rgb_to_y(p[0], p[1], p[2]);
            uint8_t Y_new = lut[Y_old];
            if (Y_old == 0) {
                /* 纯黑 → 直接映射到均衡化后的黑色电平 */
                uint8_t v = Y_new;
                p[0] = v; p[1] = v; p[2] = v;
            } else {
                float ratio = (float)Y_new / (float)Y_old;
                int r = (int)(p[0] * ratio); if (r > 255) r = 255;
                int g = (int)(p[1] * ratio); if (g > 255) g = 255;
                int b = (int)(p[2] * ratio); if (b > 255) b = 255;
                p[0] = (uint8_t)r;
                p[1] = (uint8_t)g;
                p[2] = (uint8_t)b;
            }
        }
    }

    ESP_LOGI(TAG, "hist_eq done: Y range [%d, %d]", y_min, y_max);
    return ESP_OK;
}

/* ========================================================================
 * 2. Gamma 校正 (LUT, γ > 1 提亮暗区)
 * ======================================================================== */

esp_err_t gamma_correct(dl::image::img_t &img)
{
    if (img.pix_type != dl::image::DL_IMAGE_PIX_TYPE_RGB888) {
        ESP_LOGE(TAG, "gamma: requires RGB888 input");
        return ESP_ERR_INVALID_ARG;
    }

    int w = img.width, h = img.height;
    int row_step = img.row_step();
    uint8_t *data = (uint8_t *)img.data;

    /* 预计算 LUT: out = 255 * (in/255)^(1/γ) */
    uint8_t lut[256];
    float inv_gamma = 1.0f / PREPROC_GAMMA_VALUE;
    for (int i = 0; i < 256; i++) {
        float v = powf((float)i / 255.0f, inv_gamma) * 255.0f;
        if (v > 255.0f) v = 255.0f;
        lut[i] = (uint8_t)(v + 0.5f);
    }

    /* 逐像素查表 (每通道独立) */
    for (int y = 0; y < h; y++) {
        uint8_t *row = data + y * row_step;
        for (int x = 0; x < w; x++) {
            uint8_t *p = row + x * 3;
            p[0] = lut[p[0]];
            p[1] = lut[p[1]];
            p[2] = lut[p[2]];
        }
    }

    ESP_LOGI(TAG, "gamma_correct done: gamma=%.1f", (double)PREPROC_GAMMA_VALUE);
    return ESP_OK;
}

/* ========================================================================
 * 3. 锐化 (3x3 Laplacian)
 * ======================================================================== */

esp_err_t sharpen(dl::image::img_t &img)
{
    if (img.pix_type != dl::image::DL_IMAGE_PIX_TYPE_RGB888) {
        ESP_LOGE(TAG, "sharpen: requires RGB888 input");
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_temp_buf) {
        ESP_LOGE(TAG, "sharpen: temp buffer not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int w = img.width, h = img.height;
    int row_step = img.row_step();
    size_t buf_size = h * row_step;
    if (buf_size > g_temp_size) {
        ESP_LOGE(TAG, "sharpen: image too large for temp buffer");
        return ESP_ERR_NO_MEM;
    }

    uint8_t *data = (uint8_t *)img.data;

    /* 拷贝原图到暂存区 */
    memcpy(g_temp_buf, data, buf_size);

    /* 3x3 Laplacian 核: [[0,-1,0],[-1,5,-1],[0,-1,0]] */
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            uint8_t *p = pixel_ptr(data, x, y, row_step);

            for (int c = 0; c < 3; c++) {
                int val = 0;
                val += -1 * (int)pixel_ptr(g_temp_buf, x,     y - 1, row_step)[c];
                val += -1 * (int)pixel_ptr(g_temp_buf, x - 1, y,     row_step)[c];
                val +=  5 * (int)pixel_ptr(g_temp_buf, x,     y,     row_step)[c];
                val += -1 * (int)pixel_ptr(g_temp_buf, x + 1, y,     row_step)[c];
                val += -1 * (int)pixel_ptr(g_temp_buf, x,     y + 1, row_step)[c];

                if (val < 0) val = 0;
                if (val > 255) val = 255;
                p[c] = (uint8_t)val;
            }
        }
    }

    return ESP_OK;
}

/* ========================================================================
 * 4. 对比度拉伸 (2%-98% 百分位)
 * ======================================================================== */

esp_err_t contrast_stretch(dl::image::img_t &img)
{
    if (img.pix_type != dl::image::DL_IMAGE_PIX_TYPE_RGB888) {
        ESP_LOGE(TAG, "contrast: requires RGB888 input");
        return ESP_ERR_INVALID_ARG;
    }

    int w = img.width, h = img.height;
    int row_step = img.row_step();
    uint8_t *data = (uint8_t *)img.data;
    int total = w * h;
    int p_low = total * 2 / 100;   /* 2%  */
    int p_high = total * 98 / 100; /* 98% */

    /* 每通道计算直方图并找百分位边界 */
    for (int c = 0; c < 3; c++) {
        uint32_t hist[256] = {0};
        for (int y = 0; y < h; y++) {
            uint8_t *row = data + y * row_step;
            for (int x = 0; x < w; x++) {
                hist[row[x * 3 + c]]++;
            }
        }

        uint8_t low = 0, high = 255;
        uint32_t cdf = 0;
        for (int i = 0; i < 256; i++) {
            cdf += hist[i];
            if (cdf >= (uint32_t)p_low) { low = i; break; }
        }
        cdf = 0;
        for (int i = 255; i >= 0; i--) {
            cdf += hist[i];
            if (cdf >= (uint32_t)(total - p_high)) { high = i; break; }
        }

        if (high <= low) continue;  /* 范围太窄, 跳过此通道 */

        /* 线性拉伸: new = (old - low) * 255 / (high - low) */
        float scale = 255.0f / (float)(high - low);
        for (int y = 0; y < h; y++) {
            uint8_t *row = data + y * row_step;
            for (int x = 0; x < w; x++) {
                uint8_t *p = row + x * 3 + c;
                int val = (int)((*p - low) * scale + 0.5f);
                if (val < 0) val = 0;
                if (val > 255) val = 255;
                *p = (uint8_t)val;
            }
        }
    }

    return ESP_OK;
}

/* ========================================================================
 * 5. 去噪 (3x3 中值滤波)
 * ======================================================================== */

/* 9 元素排序网络 (取中位数 = 第5个) */
static inline uint8_t median_of_9(uint8_t a[9])
{
    /* 使用 std::nth_element 或手动偏序 — 此处用手动冒泡4轮取第5小 */
    uint8_t v[9];
    memcpy(v, a, 9);
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 9; j++) {
            if (v[j] < v[i]) { uint8_t t = v[i]; v[i] = v[j]; v[j] = t; }
        }
    }
    /* v[4] 现在是第5小的元素 (中位数) */
    for (int i = 4; i < 9; i++) {
        for (int j = i + 1; j < 9; j++) {
            if (v[j] < v[i]) { uint8_t t = v[i]; v[i] = v[j]; v[j] = t; }
        }
    }
    return v[4];
}

esp_err_t denoise_median(dl::image::img_t &img)
{
    if (img.pix_type != dl::image::DL_IMAGE_PIX_TYPE_RGB888) {
        ESP_LOGE(TAG, "denoise: requires RGB888 input");
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_temp_buf) {
        ESP_LOGE(TAG, "denoise: temp buffer not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int w = img.width, h = img.height;
    int row_step = img.row_step();
    size_t buf_size = h * row_step;
    if (buf_size > g_temp_size) {
        ESP_LOGE(TAG, "denoise: image too large for temp buffer");
        return ESP_ERR_NO_MEM;
    }

    uint8_t *data = (uint8_t *)img.data;
    memcpy(g_temp_buf, data, buf_size);

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            uint8_t *p = pixel_ptr(data, x, y, row_step);

            for (int c = 0; c < 3; c++) {
                uint8_t nb[9];
                nb[0] = pixel_ptr(g_temp_buf, x - 1, y - 1, row_step)[c];
                nb[1] = pixel_ptr(g_temp_buf, x,     y - 1, row_step)[c];
                nb[2] = pixel_ptr(g_temp_buf, x + 1, y - 1, row_step)[c];
                nb[3] = pixel_ptr(g_temp_buf, x - 1, y,     row_step)[c];
                nb[4] = pixel_ptr(g_temp_buf, x,     y,     row_step)[c];
                nb[5] = pixel_ptr(g_temp_buf, x + 1, y,     row_step)[c];
                nb[6] = pixel_ptr(g_temp_buf, x - 1, y + 1, row_step)[c];
                nb[7] = pixel_ptr(g_temp_buf, x,     y + 1, row_step)[c];
                nb[8] = pixel_ptr(g_temp_buf, x + 1, y + 1, row_step)[c];
                p[c] = median_of_9(nb);
            }
        }
    }

    return ESP_OK;
}

/* ========================================================================
 * 预处理管线 (按顺序应用)
 * ======================================================================== */

esp_err_t preprocess(dl::image::img_t &img, uint32_t flags)
{
    if (flags == 0) return ESP_OK;
    if (img.pix_type != dl::image::DL_IMAGE_PIX_TYPE_RGB888) {
        ESP_LOGE(TAG, "preprocess: requires RGB888 input");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    if (flags & PREPROC_FLAG_DENOISE) {
        ret = denoise_median(img);
        if (ret != ESP_OK) return ret;
    }
    if (flags & PREPROC_FLAG_GAMMA) {
        ret = gamma_correct(img);
        if (ret != ESP_OK) return ret;
    }
    if (flags & PREPROC_FLAG_HIST_EQ) {
        ret = histogram_equalize(img);
        if (ret != ESP_OK) return ret;
    }
    if (flags & PREPROC_FLAG_SHARPEN) {
        ret = sharpen(img);
        if (ret != ESP_OK) return ret;
    }
    if (flags & PREPROC_FLAG_CONTRAST) {
        ret = contrast_stretch(img);
        if (ret != ESP_OK) return ret;
    }

    return ESP_OK;
}

/* ========================================================================
 * C 可调用: 对 RGB565 帧做预处理, 返回新 RGB565 缓冲区
 * ======================================================================== */

uint8_t *preprocess_frame_rgb565(const uint8_t *src_buf,
                                  int width, int height,
                                  uint32_t flags, size_t *out_len)
{
    if (!src_buf || width <= 0 || height <= 0 || flags == 0 || !g_temp_buf) {
        return NULL;
    }

    size_t rgb565_size = width * height * 2;
    uint8_t *dst_buf = (uint8_t *)heap_caps_calloc(1, rgb565_size,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!dst_buf) {
        ESP_LOGE(TAG, "preprocess_frame: failed to alloc output buffer");
        return NULL;
    }

    /* Step 1: RGB565 → RGB888 (到共享暂存区) */
    dl::image::img_t src;
    src.data     = (void *)src_buf;
    src.width    = (uint16_t)width;
    src.height   = (uint16_t)height;
    src.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

    dl::image::img_t rgb888;
    rgb888.data     = g_temp_buf;
    rgb888.width    = (uint16_t)width;
    rgb888.height   = (uint16_t)height;
    rgb888.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

    dl::image::ImageTransformer()
        .set_src_img(src)
        .set_dst_img(rgb888)
        .transform();

    /* Step 2: 预处理 (原地修改 RGB888) */
    esp_err_t ret = preprocess(rgb888, flags);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "preprocess_frame: preprocess failed");
        heap_caps_free(dst_buf);
        return NULL;
    }

    /* Step 3: RGB888 → RGB565 */
    dl::image::img_t dst;
    dst.data     = dst_buf;
    dst.width    = (uint16_t)width;
    dst.height   = (uint16_t)height;
    dst.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

    dl::image::ImageTransformer()
        .set_src_img(rgb888)
        .set_dst_img(dst)
        .transform();

    if (out_len) *out_len = rgb565_size;
    return dst_buf;
}
