#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Display / LVGL Task 入口
 *
 * 职责: 周期性调用 lv_timer_handler() (~30fps), 驱动 LVGL 定时器和动画。
 * 显示初始化在 app_main 中完成 (与原有流程一致), 此任务仅负责 LVGL 心跳。
 */
void display_task(void *pvParams);

#ifdef __cplusplus
}
#endif
