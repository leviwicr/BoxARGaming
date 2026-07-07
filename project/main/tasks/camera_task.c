/**
 * Camera Task — V4L2 帧捕获
 *
 * 相机初始化在 app_main 中完成 (与原有流程一致)。
 * 此任务仅负责运行时的按需帧捕获:
 *   1. 阻塞等待 g_capture_request 信号量
 *   2. 调用 camera_capture_frame() (DQBUF 阻塞)
 *   3. 将帧发布到 g_frame_response_q
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "camera/camera_driver.h"
#include "ipc/ipc.h"

static const char *TAG = "camera_task";

void camera_task(void *pvParams)
{
    (void)pvParams;
    ESP_LOGI(TAG, "Camera task started, waiting for capture requests...");

    while (1) {
        /* 阻塞等待捕获请求 */
        if (xSemaphoreTake(g_capture_request, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 捕获一帧 (DQBUF 阻塞直到帧就绪或超时) */
        camera_frame_t frame;
        memset(&frame, 0, sizeof(frame));
        esp_err_t ret = camera_capture_frame(&frame, 2000);

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Capture failed: %s (0x%X)", esp_err_to_name(ret), ret);
            /* 仍然发送空帧通知调用者失败 */
        }

        /* 发布帧到 Main Control Task (阻塞等待直到队列有空间) */
        if (xQueueSend(g_frame_response_q, &frame, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Frame response queue full, dropping frame");
        }
    }
}
