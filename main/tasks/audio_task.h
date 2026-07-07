#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Audio Task 入口
 *
 * 职责:
 *   - 接收音频命令 (SFX 触发, BGM 控制, 音量)
 *   - 驱动 PSG 合成器实时渲染
 *   - 输出 PCM 数据到 ES8311 Codec
 *
 * 运行于 Core 0 (外设 I/O 核), 优先级 3。
 */
void audio_task(void *pvParams);

#ifdef __cplusplus
}
#endif
