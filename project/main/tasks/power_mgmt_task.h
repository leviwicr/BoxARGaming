#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Power Management Task entry point
 *
 * Phase 2: 仅监控空闲时间、记录状态转换日志。
 * Phase 3 将扩展: 背光控制、相机启停、IMU 降速。
 */
void power_mgmt_task(void *pvParams);

/**
 * @brief Notify power management that user activity occurred.
 *
 * Called from button callbacks and main control state transitions.
 * Resets the idle timer.
 */
void notify_user_activity(void);

/**
 * @brief Get idle time in seconds since last user activity.
 */
uint32_t pm_get_idle_seconds(void);

/**
 * @brief Resume all peripherals from power-save state.
 *
 * Restores backlight to 100%, restarts camera stream if stopped,
 * resets IMU to full 200Hz polling. Safe to call when already active
 * (returns immediately).
 *
 * Called from:
 *   - Power Task when idle drops below 5s
 *   - Main Control Task at game/detect entry (fast-path)
 */
void pm_resume_all(void);

#ifdef __cplusplus
}
#endif
