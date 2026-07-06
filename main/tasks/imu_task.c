/**
 * IMU Task — 姿态采样与发布
 *
 * IMU 初始化在 app_main 中完成 (与原有流程一致)。
 * 此任务仅负责:
 *   1. 以 200Hz 固定频率读取 IMU 传感器
 *   2. 运行卡尔曼滤波姿态解算
 *   3. 将最新姿态发布到 g_imu_attitude_q (overwrite 模式)
 *
 * 消费者 (Main Control Task) 通过 xQueuePeek 非阻塞获取最新姿态。
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "imu/imu_driver.h"
#include "ipc/ipc.h"

static const char *TAG = "imu_task";

void imu_task(void *pvParams)
{
    (void)pvParams;
    ESP_LOGI(TAG, "IMU task started, polling @ 200Hz");

    TickType_t last_wake = xTaskGetTickCount();
    int last_period_ms = g_imu_period_ms;  /* 跟踪周期变化 */

    /* 通知 app_main: IMU 已就绪 */
    xEventGroupSetBits(g_sys_events, SYS_EVT_IMU_READY);

    while (1) {
        /* 检测采样周期是否变化 (省电模式切换时由 Power Task 修改) */
        if (g_imu_period_ms != last_period_ms) {
            last_period_ms = g_imu_period_ms;
            last_wake = xTaskGetTickCount();  /* 重置基准, 避免追赶延迟 */
            ESP_LOGI(TAG, "IMU period changed to %d ms (%d Hz)",
                     last_period_ms, 1000 / last_period_ms);
        }

        TickType_t period = pdMS_TO_TICKS(last_period_ms);
        vTaskDelayUntil(&last_wake, period);

        imu_attitude_t att;
        if (imu_get_attitude(&att) == ESP_OK) {
            /* 发布最新姿态 (非阻塞, 覆盖旧数据) */
            xQueueOverwrite(g_imu_attitude_q, &att);
        }
    }
}
