#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "camera/camera_driver.h"
#include "imu/imu_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Inter-Process Communication — 全局句柄与事件定义
 * ======================================================================== */

/* ---- 事件类型 ---- */
typedef enum {
    EVT_BTN_LIVE_VIEW = 1,
    EVT_BTN_EDGE_VIEW,
    EVT_BTN_GAME,
    EVT_BTN_DETECT,
    EVT_BTN_PREPROC,
    EVT_TOUCH_SCREEN,
    EVT_GAME_CAPTURE_DONE,
    EVT_GAME_WIN,
    EVT_GAME_LOSE,
    EVT_GAME_EXIT,
    EVT_DETECTION_COMPLETE,
    EVT_DETECTION_FAILED,
    EVT_SYSTEM_READY,
    EVT_DISPLAY_READY,
    EVT_CAMERA_READY,
    EVT_IMU_READY,
    EVT_USER_ACTIVITY,
} event_type_t;

typedef struct {
    event_type_t type;
    void        *data;
    uint32_t     timestamp;
} event_t;

/* ---- 相机任务命令 ---- */
typedef enum {
    CAMERA_CMD_NONE = 0,
} camera_cmd_t;

/* ---- 全局IPC句柄 ---- */
extern QueueHandle_t      g_event_q;
extern QueueHandle_t      g_frame_response_q;
extern SemaphoreHandle_t  g_capture_request;
extern QueueHandle_t      g_imu_attitude_q;
extern EventGroupHandle_t g_sys_events;

/* ---- 外设省电控制标志 (Phase 3) ---- */
extern volatile bool g_camera_paused;   // 相机流暂停标志, 为真时 camera_capture_via_task 拒绝请求
extern volatile bool g_imu_active;      // IMU 全速标志 (保留, 供未来扩展)
extern volatile int  g_imu_period_ms;   // IMU 采样周期 (ms), 默认 5 (200Hz)

/* 事件组位 */
#define SYS_EVT_DISPLAY_READY  (1 << 0)
#define SYS_EVT_CAMERA_READY   (1 << 1)
#define SYS_EVT_IMU_READY      (1 << 2)
#define SYS_EVT_GAME_ACTIVE    (1 << 3)
#define SYS_EVT_ALL_READY      (SYS_EVT_DISPLAY_READY | SYS_EVT_CAMERA_READY | SYS_EVT_IMU_READY)

/* ---- 便捷函数 ---- */

/**
 * @brief 通过 Camera Task 捕获一帧 (替代直接调用 camera_capture_frame)
 *
 * 发送捕获请求给 Camera Task, 阻塞等待帧就绪。
 * 行为与 camera_capture_frame() 相同, 只是通过任务间通信完成。
 *
 * @param frame      输出帧数据
 * @param timeout_ms 超时时间(毫秒)
 * @return ESP_OK 成功, ESP_ERR_TIMEOUT 超时
 */
esp_err_t camera_capture_via_task(camera_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief 获取最新IMU姿态 (非阻塞, 从IMU Task队列读取)
 * @param att 输出姿态数据
 * @return ESP_OK 有数据, ESP_FAIL 队列为空
 */
esp_err_t imu_get_attitude_via_task(imu_attitude_t *att);

#ifdef __cplusplus
}
#endif
