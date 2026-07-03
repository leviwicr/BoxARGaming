#pragma once

#include "esp_err.h"
#include "config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化预处理模块 (分配共享 RGB888 暂存缓冲区)
 */
esp_err_t preprocessing_init(void);

/**
 * @brief 反初始化预处理模块 (释放暂存缓冲区)
 */
void preprocessing_deinit(void);

/**
 * @brief 获取共享暂存缓冲区指针 (800x640x3, PSRAM)
 */
uint8_t *preprocessing_get_temp_buffer(void);

/**
 * @brief 对 RGB565 帧应用预处理, 返回新的 RGB565 缓冲区
 *
 * 内部流程: RGB565→RGB888→preprocess→RGB565
 * 调用者负责 heap_caps_free() 释放返回的缓冲区。
 *
 * @param src_buf 源 RGB565 缓冲区
 * @param width   图像宽度 (px)
 * @param height  图像高度 (px)
 * @param flags   预处理标志位 (PREPROC_FLAG_* 按位或)
 * @param out_len 输出: 新缓冲区字节长度 (可为 NULL)
 * @return 新的 RGB565 缓冲区 (PSRAM), 或 NULL 表示失败
 */
uint8_t *preprocess_frame_rgb565(const uint8_t *src_buf,
                                  int width, int height,
                                  uint32_t flags, size_t *out_len);

#ifdef __cplusplus
}
#endif

/* ---- 以下仅供 C++ 编译单元使用 ---- */
#ifdef __cplusplus
#include "dl_image_define.hpp"

/**
 * @brief 对 RGB888 图像应用预处理管线
 *
 * 按顺序应用: 直方图均衡化 → 锐化 → 对比度拉伸 → 去噪
 *
 * @param img   输入图像 (RGB888 格式, 原地修改)
 * @param flags 预处理标志位组合 (PREPROC_FLAG_* 按位或)
 * @return ESP_OK 成功
 */
esp_err_t preprocess(dl::image::img_t &img, uint32_t flags);

/**
 * @brief Y通道直方图均衡化 (RGB888, 原地)
 */
esp_err_t histogram_equalize(dl::image::img_t &img);

/**
 * @brief Gamma 校正 (RGB888, 原地, γ = PREPROC_GAMMA_VALUE)
 *
 * 逐像素 LUT 映射: out = 255 * (in/255)^(1/γ)
 * γ > 1 提亮暗区, 不依赖全局统计, 不易放大噪声。
 */
esp_err_t gamma_correct(dl::image::img_t &img);

/**
 * @brief 3x3 Laplacian 锐化 (RGB888, 原地)
 */
esp_err_t sharpen(dl::image::img_t &img);

/**
 * @brief 对比度拉伸 2%-98% 百分位 (RGB888, 原地)
 */
esp_err_t contrast_stretch(dl::image::img_t &img);

/**
 * @brief 3x3 中值滤波去噪 (RGB888, 原地)
 */
esp_err_t denoise_median(dl::image::img_t &img);
#endif
