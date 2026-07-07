#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IMU Task 入口
 *
 * 职责: 以固定频率(200Hz)轮询 IMU, 运行卡尔曼滤波, 将姿态发布到 g_imu_attitude_q。
 * IMU 初始化在 app_main 中完成 (与原有流程一致), 此任务仅负责运行时数据采集。
 */
void imu_task(void *pvParams);

#ifdef __cplusplus
}
#endif
