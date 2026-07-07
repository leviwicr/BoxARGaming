/**
 * Audio Task — 实时音频合成与输出
 *
 * 驱动流程:
 *   1. 初始化 Audio Driver (I2S + ES8311)
 *   2. 初始化 PSG Synthesizer (4ch chip music engine)
 *   3. 主循环: 接收命令 → 渲染样本 → 输出到 Codec
 *
 * 线程安全: 仅在 Audio Task 中调用 synth 函数, 无需互斥锁。
 * 其他任务通过 g_audio_cmd_q 发送命令, 队列深度 16 保证不丢事件。
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "audio/audio_driver.h"
#include "audio/audio_synth.h"
#include "ipc/ipc.h"
#include "tasks/audio_task.h"

static const char *TAG = "audio_task";

void audio_task(void *pvParams)
{
    (void)pvParams;
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  Audio Task — Chip Music Synthesizer");
    ESP_LOGI(TAG, "==============================================");

    /* ---- 1. Init audio hardware ---- */
    esp_err_t ret = audio_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio driver init FAILED: %s", esp_err_to_name(ret));
        /* Non-fatal: continue without audio */
        vTaskDelete(NULL);
        return;
    }

    ret = audio_driver_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio start FAILED: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    /* ---- 2. Init synth ---- */
    audio_synth_init();
    audio_driver_set_volume(60);

    ESP_LOGI(TAG, "Audio task ready — waiting for commands");

    /* ---- 3. Main loop ---- */
    int16_t render_buf[AUDIO_BLOCK_SAMPLES];

    while (1) {
        /* Check for incoming commands (non-blocking) */
        audio_cmd_t cmd;
        while (xQueueReceive(g_audio_cmd_q, &cmd, 0) == pdTRUE) {
            switch (cmd.cmd) {
            case AUDIO_CMD_PLAY_SFX:
                ESP_LOGD(TAG, "CMD: Play SFX %d", cmd.sfx);
                audio_synth_trigger_sfx(cmd.sfx);
                break;

            case AUDIO_CMD_BGM_START:
                ESP_LOGI(TAG, "CMD: BGM Start");
                audio_synth_bgm_start();
                break;

            case AUDIO_CMD_BGM_STOP:
                ESP_LOGI(TAG, "CMD: BGM Stop");
                audio_synth_bgm_stop();
                break;

            case AUDIO_CMD_BGM_PAUSE:
                ESP_LOGI(TAG, "CMD: BGM Pause");
                audio_synth_bgm_pause();
                break;

            case AUDIO_CMD_SET_MASTER_VOL:
                audio_synth_set_master_vol(cmd.value);
                audio_driver_set_volume(cmd.value);
                break;

            case AUDIO_CMD_MUTE:
                ESP_LOGI(TAG, "CMD: Mute");
                audio_driver_mute();
                break;

            case AUDIO_CMD_UNMUTE:
                ESP_LOGI(TAG, "CMD: Unmute");
                audio_driver_unmute();
                break;

            default:
                break;
            }
        }

        /* Render one block of audio */
        audio_synth_render(render_buf, AUDIO_BLOCK_SAMPLES);

        /* Output to codec (blocking — natural flow control) */
        audio_driver_write((const uint8_t *)render_buf, AUDIO_BLOCK_BYTES);
    }
}
