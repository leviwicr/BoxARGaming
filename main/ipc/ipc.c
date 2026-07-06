/**
 * IPC — 进程间通信实现
 *
 * 定义全局 IPC 句柄, 以及便捷函数 camera_capture_via_task() /
 * imu_get_attitude_via_task() 的实现。
 */

#include "ipc.h"

/* ---- 全局IPC句柄定义 ---- */
QueueHandle_t      g_event_q           = NULL;
QueueHandle_t      g_frame_response_q  = NULL;
SemaphoreHandle_t  g_capture_request   = NULL;
QueueHandle_t      g_imu_attitude_q    = NULL;
EventGroupHandle_t g_sys_events        = NULL;

/* ---- 外设省电控制标志 (Phase 3) ---- */
volatile bool g_camera_paused = false;
volatile bool g_imu_active    = true;
volatile int  g_imu_period_ms = 5;   /* 默认 200Hz */

/* ---- 通过 Camera Task 捕获一帧 (阻塞) ---- */
esp_err_t camera_capture_via_task(camera_frame_t *frame, uint32_t timeout_ms)
{
    if (!g_capture_request || !g_frame_response_q || !frame) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 相机流已暂停 (省电模式), 拒绝请求 */
    if (g_camera_paused) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 发送捕获请求信号量 */
    xSemaphoreGive(g_capture_request);

    /* 阻塞等待帧返回 */
    if (xQueueReceive(g_frame_response_q, frame, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return (frame->buffer != NULL) ? ESP_OK : ESP_ERR_TIMEOUT;
    }
    return ESP_ERR_TIMEOUT;
}

/* ---- 获取最新IMU姿态 (非阻塞) ---- */
esp_err_t imu_get_attitude_via_task(imu_attitude_t *att)
{
    if (!g_imu_attitude_q || !att) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueuePeek(g_imu_attitude_q, att, 0) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_FAIL;
}
