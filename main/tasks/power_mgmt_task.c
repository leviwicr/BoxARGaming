/**
 * Power Management Task — 空闲检测 (Phase 2)
 *
 * 当前职责:
 *   - 监控用户空闲时间
 *   - 记录阶段日志 (30s / 60s / 120s)
 *
 * 注: SMP FreeRTOS (ESP-IDF v5.5) 不支持 esp_pm_configure(),
 *     因此 Phase 2 仅做空闲跟踪, 不涉及 CPU/DFS/Light-sleep。
 *     实际的省电动作在 Phase 3: 背光、相机流、IMU 采样率控制。
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "camera/camera_driver.h"
#include "ipc/ipc.h"
#include "tasks/power_mgmt_task.h"

static const char *TAG = "power_mgmt";

/* ---- 省电状态机 ---- */
typedef enum {
    PM_STATE_ACTIVE = 0,   /* idle < 30s, 全速运行 */
    PM_STATE_DIM,           /* 30s <= idle < 60s, 背光降至20% */
    PM_STATE_DEEP_SLEEP,    /* idle >= 60s, 背光关闭 + 相机停流 + IMU降速 */
} pm_state_t;

static pm_state_t g_pm_state = PM_STATE_ACTIVE;

/* ---- 空闲跟踪 ---- */
static volatile uint32_t g_last_activity_tick = 0;

/* ---- 日志节流 ---- */
static uint32_t g_last_idle_logged = 0;

void notify_user_activity(void)
{
    g_last_activity_tick = xTaskGetTickCount();
}

uint32_t pm_get_idle_seconds(void)
{
    if (g_last_activity_tick == 0) {
        return 0;
    }
    return (xTaskGetTickCount() - g_last_activity_tick) * portTICK_PERIOD_MS / 1000;
}

void pm_resume_all(void)
{
    if (g_pm_state == PM_STATE_ACTIVE) {
        return;  /* 无需恢复 */
    }

    ESP_LOGI(TAG, "Resuming all peripherals from state=%d...", (int)g_pm_state);

    if (g_pm_state == PM_STATE_DEEP_SLEEP) {
        /* 恢复相机流 */
        esp_err_t ret = camera_stream_start();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Camera stream resume failed: %s", esp_err_to_name(ret));
        }
        g_camera_paused = false;

        /* 恢复 IMU 全速 */
        g_imu_period_ms = 5;
    }

    /* 恢复背光 (DIM 和 DEEP_SLEEP 都需要) */
    bsp_display_brightness_set(100);

    g_pm_state = PM_STATE_ACTIVE;
    ESP_LOGI(TAG, "All peripherals resumed");
}

void power_mgmt_task(void *pvParams)
{
    (void)pvParams;
    ESP_LOGI(TAG, "Power management task started, monitoring idle @ 10Hz");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  /* 10Hz */

    notify_user_activity();

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        uint32_t idle_s = pm_get_idle_seconds();

        /* ================================================================
         * Phase 3: 分层省电动作
         * ================================================================ */

        /* 恢复: 用户活动后回到 ACTIVE */
        if (idle_s < 5 && g_pm_state != PM_STATE_ACTIVE) {
            pm_resume_all();
        }

        /* 30s: 背光降至 20% */
        if (idle_s >= 30 && g_pm_state == PM_STATE_ACTIVE) {
            ESP_LOGI(TAG, "System idle 30s — dimming backlight to 20%%");
            bsp_display_brightness_set(20);
            g_pm_state = PM_STATE_DIM;
        }

        /* 60s: 背光关闭 + 相机停流 + IMU 降至 10Hz */
        if (idle_s >= 60 && g_pm_state == PM_STATE_DIM) {
            ESP_LOGI(TAG, "System idle 60s — deep sleep (backlight OFF + camera STOP + IMU 10Hz)");

            /* 背光关闭 */
            bsp_display_brightness_set(0);

            /* 相机停流: 先设标志拒绝新请求, 等待进行中的 DQBUF 完成, 再 STREAMOFF */
            g_camera_paused = true;
            vTaskDelay(pdMS_TO_TICKS(100));  /* 等待进行中的 DQBUF (~33ms @30fps) 完成 */

            esp_err_t ret = camera_stream_stop();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Camera stream stop failed: %s", esp_err_to_name(ret));
            }

            /* IMU 降至 10Hz (100ms 周期) */
            g_imu_period_ms = 100;

            g_pm_state = PM_STATE_DEEP_SLEEP;
        }

        /* 阶段性日志 */
        if (idle_s >= 30 && g_last_idle_logged < 30) {
            g_last_idle_logged = 30;
            ESP_LOGI(TAG, "System idle for 30s");
        } else if (idle_s >= 60 && g_last_idle_logged < 60) {
            g_last_idle_logged = 60;
            ESP_LOGI(TAG, "System idle for 60s");
        } else if (idle_s >= 120 && g_last_idle_logged < 120) {
            g_last_idle_logged = 120;
            ESP_LOGI(TAG, "System idle for 120s");
        } else if (idle_s < 5) {
            g_last_idle_logged = 0;
        }
    }
}
