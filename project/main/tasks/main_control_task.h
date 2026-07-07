#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Main Control Task 入口
 *
 * 职责: 游戏状态机、用户交互响应、任务协调。
 * 运行在 Core 1 (计算核), 与 Core 0 (外设 I/O 核) 并行。
 */
void main_control_task(void *pvParams);

#ifdef __cplusplus
}
#endif
