/**
 * Display / LVGL Task — LVGL 心跳
 *
 * 显示初始化在 app_main 中完成 (与原有流程一致)。
 * 此任务仅负责周期性调用 lv_timer_handler(), 驱动:
 *   - LVGL 定时器 (动画等)
 *   - LVGL 显示刷新
 *   - 触摸输入处理 (GT911)
 *
 * 所有 display_*() 函数通过 bsp_display_lock/unlock 保护,
 * 可以从其他任务安全调用。
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "ipc/ipc.h"

static const char *TAG = "display_task";

void display_task(void *pvParams)
{
    (void)pvParams;
    ESP_LOGI(TAG, "Display task started, LVGL heartbeat @ 30fps");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  /* ~30fps */

    /* 通知 app_main: 显示已就绪 */
    xEventGroupSetBits(g_sys_events, SYS_EVT_DISPLAY_READY);

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        bsp_display_lock(portMAX_DELAY);
        lv_timer_handler();
        bsp_display_unlock();
    }
}
