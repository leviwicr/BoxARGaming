/**
 * Audio Driver — ES8311 Codec + I2S 实现
 *
 * 调用 Waveshare BSP 函数管理 ES8311 音频编解码器:
 *   bsp_audio_init()           → I2S + Codec 初始化 (默认 22050Hz/Mono/16bit)
 *   bsp_audio_codec_speaker_init() → 获取 esp_codec_dev 播放句柄
 *   esp_codec_dev_write()      → 输出 PCM 数据
 */

#include "audio_driver.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "esp_codec_dev.h"

static const char *TAG = "audio_drv";

static esp_codec_dev_handle_t g_codec_dev = NULL;
static int  g_volume = 60;          /* 0-100, default 60% */
static bool g_muted  = false;
static bool g_ready  = false;

/* ========================================================================
 * Public API
 * ======================================================================== */

esp_err_t audio_driver_init(void)
{
    if (g_codec_dev) {
        /* Already initialized */
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing audio: ES8311 + I2S @ %dHz/%dbit/Mono",
             AUDIO_SAMPLE_RATE, AUDIO_BIT_DEPTH);

    /* Step 1: Init I2C (safe to call even if already initialized by display) */
    esp_err_t ret = bsp_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Step 2: Init I2S + Codec (full-duplex, mono, 22050Hz, 16bit) */
    ret = bsp_audio_init(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Step 3: Get speaker codec device handle */
    g_codec_dev = bsp_audio_codec_speaker_init();
    if (!g_codec_dev) {
        ESP_LOGE(TAG, "Speaker codec init FAILED");
        return ESP_FAIL;
    }

    /* Step 4: Set default volume */
    esp_codec_dev_set_out_vol(g_codec_dev, (float)g_volume);

    /* Step 5: Disable ES8311 ADC sidetone & mute ADC.
     * BSP writes REG44=0x58 (dac2adc loopback) and leaves ADC powered.
     * Fix: REG44=0x08 (disable loopback), REG17=0x00 (mute ADC volume) */
    int reg_ret44 = esp_codec_dev_write_reg(g_codec_dev, 0x44, 0x08);
    int reg_ret17 = esp_codec_dev_write_reg(g_codec_dev, 0x17, 0x00);
    ESP_LOGI(TAG, "ES8311 sidetone fix: REG44=%s REG17=%s",
             reg_ret44 == 0 ? "OK" : "FAIL",
             reg_ret17 == 0 ? "OK" : "FAIL");

    g_ready = true;
    ESP_LOGI(TAG, "Audio driver initialized successfully");
    return ESP_OK;
}

esp_err_t audio_driver_start(void)
{
    if (!g_codec_dev) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = AUDIO_SAMPLE_RATE,
        .channel         = AUDIO_CHANNELS,
        .bits_per_sample = AUDIO_BIT_DEPTH,
    };

    esp_err_t ret = esp_codec_dev_open(g_codec_dev, &fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Codec open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Disable ES8311 sidetone (ADC→DAC loopback) and mute ADC.
     * BSP writes REG44=0x58 which routes mic input to speaker output.
     * REG44=0x08 disables dac2adc mixing; REG17=0x00 mutes ADC volume. */
    esp_codec_dev_write_reg(g_codec_dev, 0x44, 0x08);
    esp_codec_dev_write_reg(g_codec_dev, 0x17, 0x00);
    ESP_LOGI(TAG, "ES8311 sidetone re-check: REG44=0x08 REG17=0x00");

    ESP_LOGI(TAG, "Audio playback started");
    return ESP_OK;
}

esp_err_t audio_driver_stop(void)
{
    if (!g_codec_dev) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = esp_codec_dev_close(g_codec_dev);
    ESP_LOGW(TAG, "Audio playback stopped (ret=%d)", ret);
    return ret;
}

esp_err_t audio_driver_set_volume(int vol)
{
    if (vol < 0)   vol = 0;
    if (vol > 100) vol = 100;

    g_volume = vol;

    if (g_codec_dev && !g_muted) {
        esp_codec_dev_set_out_vol(g_codec_dev, (float)vol);
    }

    ESP_LOGI(TAG, "Volume: %d%%", vol);
    return ESP_OK;
}

esp_err_t audio_driver_mute(void)
{
    if (!g_codec_dev) return ESP_ERR_INVALID_STATE;
    if (g_muted)      return ESP_OK;

    esp_codec_dev_set_out_vol(g_codec_dev, 0.0f);
    g_muted = true;
    ESP_LOGI(TAG, "Muted");
    return ESP_OK;
}

esp_err_t audio_driver_unmute(void)
{
    if (!g_codec_dev) return ESP_ERR_INVALID_STATE;
    if (!g_muted)     return ESP_OK;

    esp_codec_dev_set_out_vol(g_codec_dev, (float)g_volume);
    g_muted = false;
    ESP_LOGI(TAG, "Unmuted (vol=%d)", g_volume);
    return ESP_OK;
}

int audio_driver_write(const uint8_t *data, int len)
{
    if (!g_codec_dev) {
        return -1;
    }

    int ret = esp_codec_dev_write(g_codec_dev, (void *)data, len);
    return ret;
}

void audio_driver_deinit(void)
{
    if (g_codec_dev) {
        esp_codec_dev_close(g_codec_dev);
        esp_codec_dev_delete(g_codec_dev);
        g_codec_dev = NULL;
    }
    g_ready = false;
    ESP_LOGI(TAG, "Audio driver deinitialized");
}

bool audio_driver_is_ready(void)
{
    return g_ready;
}
