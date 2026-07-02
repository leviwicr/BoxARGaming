/**
 * 相机驱动 —— esp_video (V4L2) + OV5647 + MIPI CSI
 *
 * 使用 esp_video 高级框架, 通过标准 V4L2 API 访问摄像头。
 * 传感器通过 BSP 共享 I2C 总线进行 SCCB 通信。
 * esp_video 内部自动管理: CSI 控制器、ISP 管线、传感器初始化。
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "linux/videodev2.h"

/* BSP — 获取已有 I2C 总线 */
#include "bsp/esp-bsp.h"

#include "config.h"
#include "camera/camera_driver.h"

static const char *TAG = "camera";

/* ========================================================================
 * 静态变量
 * ======================================================================== */

static int      g_fd           = -1;
static void    *g_buf_addr[2]  = {NULL, NULL};
static size_t   g_buf_len[2]   = {0};
static int      g_num_bufs     = 0;
static int      g_dqbuf_idx    = -1;  /* 当前已出队的缓冲区索引, -1=无 */

/* ========================================================================
 * 内部辅助
 * ======================================================================== */

/* 归还上一次出队的缓冲区 */
static void return_buffer(void)
{
    if (g_dqbuf_idx < 0) return;

    struct v4l2_buffer buf = {0};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = g_dqbuf_idx;
    if (ioctl(g_fd, VIDIOC_QBUF, &buf) != 0) {
        ESP_LOGW(TAG, "VIDIOC_QBUF(%d) failed: %d", g_dqbuf_idx, errno);
    }
    g_dqbuf_idx = -1;
}

/* ========================================================================
 * 公开接口
 * ======================================================================== */

esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "=== Camera Init Start (esp_video V4L2) ===");

    /* 1. 配置 esp_video
     *    - SCCB: 复用 BSP 已有的 I2C 总线 (与显示/触摸/音频共享)
     *    - LDO: 不初始化 (显示 BSP 已为 MIPI PHY 上电 LDO ch3)
     *    - 传感器: 类型和默认格式由 Kconfig 选择 (OV5647, RAW8 800x640)
     */
    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = {
            .init_sccb = false,            /* 使用已有 I2C 总线 */
            .i2c_handle = bsp_i2c_get_handle(),
            .freq = 100000,
        },
        .reset_pin = -1,                   /* OV5647 无外部复位 */
        .pwdn_pin  = -1,                   /* OV5647 无外部掉电 */
        .dont_init_ldo = true,             /* 显示 BSP 已处理 MIPI PHY LDO */
    };

    esp_video_init_config_t cam_config = {
        .csi = &csi_cfg,
    };

    esp_err_t ret = esp_video_init(&cam_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed: %s (0x%X)", esp_err_to_name(ret), ret);
        return ret;
    }
    ESP_LOGI(TAG, "esp_video subsystem initialized");

    /* 2. 打开 MIPI-CSI 视频设备 */
    g_fd = open("/dev/video0", O_RDWR);
    if (g_fd < 0) {
        ESP_LOGE(TAG, "Failed to open /dev/video0 (errno=%d)", errno);
        esp_video_deinit();
        return ESP_FAIL;
    }

    /* 3. 设置输出格式: RGB565 800x640
     *    OV5647 输出 RAW8, ISP 自动转换为 RGB565
     */
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAMERA_H_RES;
    fmt.fmt.pix.height      = CAMERA_V_RES;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    if (ioctl(g_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT failed (errno=%d)", errno);
        close(g_fd);
        g_fd = -1;
        esp_video_deinit();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Format: %dx%d %.4s, bytesperline=%d",
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             (char *)&fmt.fmt.pix.pixelformat,
             fmt.fmt.pix.bytesperline);

    /* 4. 请求双缓冲 */
    struct v4l2_requestbuffers req = {0};
    req.count  = 2;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(g_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed (errno=%d)", errno);
        close(g_fd);
        g_fd = -1;
        esp_video_deinit();
        return ESP_FAIL;
    }
    g_num_bufs = req.count;
    ESP_LOGI(TAG, "Buffers: %d (V4L2_MMAP)", g_num_bufs);

    /* 5. mmap 并入队所有缓冲区 */
    for (int i = 0; i < g_num_bufs; i++) {
        struct v4l2_buffer buf = {0};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(g_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF[%d] failed", i);
            goto fail;
        }

        g_buf_addr[i] = mmap(NULL, buf.length,
                             PROT_READ | PROT_WRITE,
                             MAP_SHARED, g_fd, buf.m.offset);
        g_buf_len[i] = buf.length;

        if (g_buf_addr[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap[%d] failed", i);
            g_buf_addr[i] = NULL;
            goto fail;
        }

        if (ioctl(g_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF[%d] failed", i);
            goto fail;
        }
    }

    /* 6. 开启视频流 (传感器 → CSI → ISP → V4L2 缓冲队列 全链路启动) */
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(g_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed");
        goto fail;
    }

    /* 7. 传感器级亮度/增益控制 (尽力而为, 不影响 ISP IPA 管线) */
    struct v4l2_control ctrl = {0};
    ctrl.id = V4L2_CID_AUTOGAIN;
    ctrl.value = 1;
    if (ioctl(g_fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        ESP_LOGW(TAG, "V4L2_CID_AUTOGAIN not supported");
    }

    ctrl.id = V4L2_CID_BRIGHTNESS;
    ctrl.value = 128;
    if (ioctl(g_fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        ESP_LOGW(TAG, "V4L2_CID_BRIGHTNESS not supported");
    }

    ESP_LOGI(TAG, "=== Camera Init Complete (streaming) ===");
    return ESP_OK;

fail:
    for (int i = 0; i < g_num_bufs; i++) {
        if (g_buf_addr[i]) {
            munmap(g_buf_addr[i], g_buf_len[i]);
            g_buf_addr[i] = NULL;
        }
    }
    close(g_fd);
    g_fd = -1;
    esp_video_deinit();
    return ESP_FAIL;
}

esp_err_t camera_capture_frame(camera_frame_t *out_frame, uint32_t timeout_ms)
{
    if (g_fd < 0) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (!out_frame) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 归还上一次出队的缓冲区 (使 CSI 可以继续写入) */
    return_buffer();

    /* 设置 DQBUF 超时 */
    struct timeval tv = {
        .tv_sec  = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    ioctl(g_fd, VIDIOC_S_DQBUF_TIMEOUT, &tv);

    /* 出队一帧 (阻塞等待直到帧就绪或超时) */
    struct v4l2_buffer buf = {0};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(g_fd, VIDIOC_DQBUF, &buf) != 0) {
        ESP_LOGE(TAG, "DQBUF timeout/failed (errno=%d)", errno);
        return ESP_ERR_TIMEOUT;
    }

    g_dqbuf_idx = buf.index;

    out_frame->buffer  = g_buf_addr[buf.index];
    out_frame->buf_len = buf.bytesused;
    out_frame->width   = CAMERA_H_RES;
    out_frame->height  = CAMERA_V_RES;

    return ESP_OK;
}

esp_err_t camera_warmup(int frames)
{
    if (g_fd < 0) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Camera warmup: %d frames...", frames);

    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    ioctl(g_fd, VIDIOC_S_DQBUF_TIMEOUT, &tv);

    for (int i = 0; i < frames; i++) {
        return_buffer();

        struct v4l2_buffer buf = {0};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(g_fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGW(TAG, "Warmup frame %d/%d timeout", i + 1, frames);
            break;
        }
        g_dqbuf_idx = buf.index;
    }

    ESP_LOGI(TAG, "Camera warmup done");
    return ESP_OK;
}

esp_err_t camera_test_pattern(int enable)
{
    if (g_fd < 0) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    struct v4l2_ext_control ctrl = {
        .id    = V4L2_CID_TEST_PATTERN,
        .value = enable ? 1 : 0,
    };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count      = 1,
        .controls   = &ctrl,
    };

    if (ioctl(g_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
        ESP_LOGW(TAG, "Test pattern control not supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Sensor test pattern: %s", enable ? "ON" : "OFF");
    return ESP_OK;
}

esp_err_t camera_deinit(void)
{
    ESP_LOGI(TAG, "Camera deinit");

    if (g_fd >= 0) {
        return_buffer();

        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(g_fd, VIDIOC_STREAMOFF, &type);

        for (int i = 0; i < g_num_bufs; i++) {
            if (g_buf_addr[i]) {
                munmap(g_buf_addr[i], g_buf_len[i]);
                g_buf_addr[i] = NULL;
            }
        }

        close(g_fd);
        g_fd = -1;
    }

    esp_video_deinit();
    return ESP_OK;
}
