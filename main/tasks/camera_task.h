#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Camera Task 入口
 *
 * 职责: 等待捕获请求, 执行 V4L2 DQBUF, 将帧发布到 g_frame_response_q。
 * 相机初始化在 app_main 中完成 (与原有流程一致), 此任务仅负责运行时的帧捕获。
 */
void camera_task(void *pvParams);

#ifdef __cplusplus
}
#endif
