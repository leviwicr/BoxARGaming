/**
 * Audio Driver — ES8311 Codec + I2S 初始化与播放控制
 *
 * 封装 Waveshare BSP 音频函数, 提供初始化、音量控制、音频数据输出接口。
 * 用于芯片音乐合成器的底层输出。
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 音频参数 ---- */
#define AUDIO_SAMPLE_RATE     22050
#define AUDIO_BIT_DEPTH       16
#define AUDIO_CHANNELS        1
#define AUDIO_BLOCK_SAMPLES   512    /* ~23.2ms per block */
#define AUDIO_BLOCK_BYTES     (AUDIO_BLOCK_SAMPLES * (AUDIO_BIT_DEPTH / 8))

/**
 * @brief Initialize audio hardware (I2S + ES8311 Codec + PA)
 *
 * Must be called after I2C bus is initialized (display_init does this).
 * Safe to call multiple times — returns ESP_OK if already initialized.
 *
 * @return ESP_OK on success
 */
esp_err_t audio_driver_init(void);

/**
 * @brief Start audio output (open codec device)
 * @return ESP_OK on success
 */
esp_err_t audio_driver_start(void);

/**
 * @brief Stop audio output (close codec device)
 * @return ESP_OK on success
 */
esp_err_t audio_driver_stop(void);

/**
 * @brief Set master volume
 * @param vol 0-100 (percent)
 * @return ESP_OK on success
 */
esp_err_t audio_driver_set_volume(int vol);

/**
 * @brief Mute audio (set hardware volume to 0)
 * @return ESP_OK on success
 */
esp_err_t audio_driver_mute(void);

/**
 * @brief Unmute audio (restore previous volume)
 * @return ESP_OK on success
 */
esp_err_t audio_driver_unmute(void);

/**
 * @brief Write PCM audio samples to codec (blocking)
 *
 * Internally calls esp_codec_dev_write(). May block if hardware
 * playback FIFO is full, providing natural flow control.
 *
 * @param data   PCM samples (16-bit signed, mono, interleaved if stereo)
 * @param len    Length in bytes
 * @return number of bytes written, or negative on error
 */
int audio_driver_write(const uint8_t *data, int len);

/**
 * @brief Deinitialize audio driver
 */
void audio_driver_deinit(void);

/**
 * @brief Check if audio driver is initialized
 * @return true if initialized and ready to play
 */
bool audio_driver_is_ready(void);

#ifdef __cplusplus
}
#endif
