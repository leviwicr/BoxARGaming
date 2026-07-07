/**
 * 智绘灵境 —— AR Interactive Sandbox
 *
 * app_main: 硬件初始化 + 创建 FreeRTOS 任务 + 接管主控循环
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "config.h"
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"
#include "display/display_driver.h"
#include "image_processing/image_processing.hpp"
#include "imu/imu_driver.h"
#include "physics/marble_physics.h"
#include "ipc/ipc.h"
#include "tasks/camera_task.h"
#include "tasks/display_task.h"
#include "tasks/imu_task.h"
#include "tasks/main_control_task.h"
#include "tasks/power_mgmt_task.h"
#include "tasks/audio_task.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  AR Sandbox - Multi-Task Architecture");
    ESP_LOGI(TAG, "==============================================");

    /* ====================================================================
     * 1. 创建 IPC 对象
     * ==================================================================== */
    g_event_q          = xQueueCreate(20, sizeof(event_t));
    g_frame_response_q = xQueueCreate(2, sizeof(camera_frame_t));
    g_capture_request  = xSemaphoreCreateBinary();
    g_imu_attitude_q   = xQueueCreate(1, sizeof(imu_attitude_t));
    g_audio_cmd_q      = xQueueCreate(16, sizeof(audio_cmd_t));
    g_sys_events       = xEventGroupCreate();

    if (!g_event_q || !g_frame_response_q || !g_capture_request ||
        !g_imu_attitude_q || !g_audio_cmd_q || !g_sys_events) {
        ESP_LOGE(TAG, "IPC creation FAILED");
        return;
    }
    ESP_LOGI(TAG, "IPC objects created");

    /* ====================================================================
     * 2. 初始化显示 — 必须先于其他, 提供用户反馈
     * ==================================================================== */
    esp_err_t ret = display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Display init FAILED");
        return;
    }

    /* ====================================================================
     * 3. 初始化相机
     * ==================================================================== */
    ESP_LOGI(TAG, "--- Camera Init Start ---");
    display_set_status("Status: Init camera...", 0x00FF00);

    ret = camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: %s (0x%X)", esp_err_to_name(ret), ret);
        display_set_status("Camera init FAILED!", 0xFF0000);
        return;
    }

    /* ====================================================================
     * 4. 初始化预处理模块
     * ==================================================================== */
    ret = preprocessing_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Preprocessing init failed, continuing without it");
    }

    /* ====================================================================
     * 5. 相机预热
     * ==================================================================== */
    display_set_status("Status: Warming up...", 0x00FF00);
    camera_warmup(10);

    /* ====================================================================
     * 6. 初始化 IMU + 弹珠物理 (相机预热后, I2C总线已稳定)
     * ==================================================================== */
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed (non-fatal): %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "IMU ready");
        marble_physics_init();
    }

    /* ====================================================================
     * 7. 创建 FreeRTOS 任务
     *    — Camera Task:  prio 5 (实时帧捕获)
     *    — Display Task: prio 4 (LVGL 心跳)
     *    — IMU Task:     prio 4 (200Hz 姿态采样)
     *    — Main Control: prio 2 → app_main 自身演化为本任务
     * ==================================================================== */
    ESP_LOGI(TAG, "Creating tasks...");

    xTaskCreatePinnedToCore(camera_task, "camera", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(display_task, "display", 8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, 4, NULL, 0);

    /* Power Management Task (prio 1 = lowest, monitors idle) */
    xTaskCreatePinnedToCore(power_mgmt_task, "power_mgmt", 2048, NULL, 1, NULL, 0);
    /* Audio Task (prio 3 = between IMU/Display and Power, Core 0) */
    xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, 3, NULL, 0);

    /* 等待 IMU 和 Display 就绪 (Camera Task 无需等待, 按需驱动) */
    EventBits_t ready = xEventGroupWaitBits(g_sys_events,
                                            SYS_EVT_IMU_READY | SYS_EVT_DISPLAY_READY,
                                            pdFALSE, pdTRUE, pdMS_TO_TICKS(3000));
    if ((ready & SYS_EVT_DISPLAY_READY) == 0) {
        ESP_LOGW(TAG, "Display task not ready within timeout");
    }
    if ((ready & SYS_EVT_IMU_READY) == 0) {
        ESP_LOGW(TAG, "IMU task not ready within timeout (non-fatal)");
    }

    display_set_status("Ready! Press LIVE VIEW or DETECT", 0x00FF00);
    ESP_LOGI(TAG, "All tasks started — entering main control loop");

    /* 初始化空闲计时器基线 */
    notify_user_activity();

    /* ====================================================================
     * 8. 创建 Main Control Task 到 Core 1 (计算密集型)
     *
     * Core 1 专用于: 游戏渲染 + 物理模拟 + 目标检测 + 边缘检测。
     * Core 0 保留: 相机帧捕获 (V4L2 DMA)、IMU 采样 (I2C)、
     *              LVGL 显示刷新 (SPI/DSI)、电源管理。
     * ==================================================================== */
    xTaskCreatePinnedToCore(main_control_task, "main_ctrl", 8192, NULL, 2, NULL, 1);
    ESP_LOGI(TAG, "Main control task created on Core 1");
    ESP_LOGI(TAG, "Dual-core layout: Core0=[Camera,Display,IMU,Audio,Power] Core1=[MainCtrl,Physics]");
}
