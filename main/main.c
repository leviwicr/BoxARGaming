/**
 * 智绘灵境 —— AR Interactive Sandbox
 * 阶段2: 目标检测 (ESP-DL YOLO11n)
 *
 * 当前: 摄像头画面预览 + 按键触发检测
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "config.h"
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"
#include "display/display_driver.h"
#include "image_processing/image_processing.hpp"
#include "edge_detection/edge_detection.h"
#include "esp_timer.h"
#include "imu/imu_driver.h"
#include "physics/marble_physics.h"
#include "track/track_collision.h"
#include "game/game_render.h"
#include "pixel_game/pixel_world.h"
#include "pixel_game/pixel_physics.h"
#include "pixel_game/pixel_sprite.h"

static const char *TAG = "main";

/* ---- Pixel game state machine ---- */
typedef enum {
    GAME_STATE_IDLE = 0,
    GAME_STATE_CAPTURING,
    GAME_STATE_PLAYING,
    GAME_STATE_WIN,
    GAME_STATE_LOSE,
} game_state_t;

static game_state_t g_game_state = GAME_STATE_IDLE;
static uint64_t     g_game_end_ticks = 0;

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  AR Sandbox - Phase 2: Detection");
    ESP_LOGI(TAG, "==============================================");

    /* ---- 1. 初始化显示 ---- */
    esp_err_t ret = display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Display init FAILED");
        return;
    }

    /* ---- 2. 初始化相机 ---- */
    ESP_LOGI(TAG, "--- Camera Init Start ---");
    display_set_status("Status: Init camera...", 0x00FF00);

    ret = camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: %s (0x%X)", esp_err_to_name(ret), ret);
        display_set_status("Camera init FAILED!", 0xFF0000);
        return;
    }

    /* ---- 3. 初始化预处理模块 ---- */
    ret = preprocessing_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Preprocessing init failed, continuing without it");
    }

    /* 注: edge_detect 和 track_collision 延迟到首次按 Track 时再初始化,
     *     避免启动时占用 PSRAM 导致 YOLO 模型无法加载。 */

    /* ---- 4. 相机预热 ---- */
    display_set_status("Status: Warming up...", 0x00FF00);
    camera_warmup(10);

    /* ---- 5. 初始化IMU + 弹珠物理 (相机预热后, I2C总线已稳定) ---- */
    vTaskDelay(pdMS_TO_TICKS(100));   /* 等 I2C 总线彻底释放 */
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed (non-fatal): %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "IMU ready");
        marble_physics_init();
    }

    display_set_status("Ready! Press LIVE VIEW or DETECT", 0x00FF00);
    ESP_LOGI(TAG, "Camera ready");

    /* ---- 6. 主循环 ---- */
    bool was_live_view = false;
    bool was_edge_view = false;
    uint64_t last_status_log = 0;

    while (1) {
        /* ================================================================
         * Pixel Game State Machine
         * ================================================================ */
        /* Check game capture trigger (Game button from IDLE) */
        if (display_game_capture_triggered() && g_game_state == GAME_STATE_IDLE) {
            g_game_state = GAME_STATE_CAPTURING;
            ESP_LOGI(TAG, "Game capture triggered!");
        }

        /* Check game exit (Game/Exit button during PLAYING) */
        if (display_game_exit_triggered()) {
            if (g_game_state == GAME_STATE_PLAYING ||
                g_game_state == GAME_STATE_WIN ||
                g_game_state == GAME_STATE_LOSE) {
                ESP_LOGI(TAG, "Game exit requested");
                pixel_physics_stop();
                display_exit_game_mode();
                pixel_world_destroy();
                g_game_state = GAME_STATE_IDLE;
                display_set_status("Ready! Press DETECT or GAME", 0x00FF00);
                continue;
            }
        }

        switch (g_game_state) {

        case GAME_STATE_CAPTURING: {
            ESP_LOGI(TAG, "--- Game: Capturing frame ---");
            display_set_status("Game: Capturing...", 0x00FF00);

            camera_frame_t frame;
            ret = camera_capture_frame(&frame, 2000);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Game capture FAILED");
                display_set_status("Game capture FAILED!", 0xFF0000);
                g_game_state = GAME_STATE_IDLE;
                break;
            }

            /* Edge detection */
            if (!edge_get_downscale_buf()) {
                edge_detect_init();
            }
            uint8_t *ds_buf = edge_get_downscale_buf();
            uint8_t *emap   = edge_get_edge_map_buf();
            edge_downscale_half(frame.buffer, frame.width, frame.height, ds_buf);
            edge_detect_run(ds_buf, EDGE_DOWNSCALE_W, EDGE_DOWNSCALE_H,
                           emap, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD);

            /* COCO detection */
            detection_result_t detections[DETECTION_MAX_OBJECTS];
            int det_count = DETECTION_MAX_OBJECTS;
            ret = detection_run(&frame, detections, &det_count, PREPROC_FLAG_NONE);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Detection failed during game capture");
                det_count = 0;
            }

            /* Build pixel world from edges + detections */
            sprite_init();
            pixel_world_build(&frame, emap, EDGE_DOWNSCALE_W, EDGE_DOWNSCALE_H,
                            detections, det_count);
            edge_detect_deinit();  /* release edge buffers */

            /* Reset marble to map top-center */
            marble_physics_reset();
            marble_set_position(MARBLE_INIT_X, MARBLE_INIT_Y);

            /* Init game mode display (landscape + HUD) */
            ret = display_init_game_mode();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Game mode init FAILED");
                pixel_world_destroy();
                g_game_state = GAME_STATE_IDLE;
                break;
            }

            /* Start game physics callback */
            pixel_physics_start();

            g_game_state = GAME_STATE_PLAYING;
            ESP_LOGI(TAG, "Game PLAYING - tilt to move marble!");
            break;
        }

        case GAME_STATE_PLAYING: {
            /* Render pixel game world */
            uint16_t *gbuf = (uint16_t *)display_get_game_render_buf(NULL, NULL);
            if (gbuf) {
                game_render_pixel_frame(gbuf, GAME_MAP_PIXELS, GAME_MAP_PIXELS);
            }
            display_refresh_game();

            /* Update HUD */
            marble_state_t mb;
            marble_physics_get_state(&mb);
            imu_attitude_t att;
            imu_get_attitude(&att);
            float speed = sqrtf(mb.vx * mb.vx + mb.vy * mb.vy);
            const char *bounce_label = pixel_physics_bounce_label();
            int wp_ms = marble_wall_pass_remaining_ms();

            pixel_world_t *world = pixel_world_get();
            display_update_game_hud(mb.x, mb.y, speed, att.roll, att.pitch,
                                    wp_ms, bounce_label,
                                    world ? world->objects : NULL,
                                    world ? world->object_count : 0);

            /* Check win/lose conditions */
            if (world) {
                if (world->goal_reached) {
                    ESP_LOGI(TAG, "*** YOU WIN! ***");
                    display_show_game_end(true);
                    g_game_state = GAME_STATE_WIN;
                    g_game_end_ticks = xTaskGetTickCount();
                } else if (world->player_dead) {
                    ESP_LOGI(TAG, "*** GAME OVER ***");
                    display_show_game_end(false);
                    g_game_state = GAME_STATE_LOSE;
                    g_game_end_ticks = xTaskGetTickCount();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1000 / GAME_FPS));
            break;
        }

        case GAME_STATE_WIN:
        case GAME_STATE_LOSE:
            /* Show end screen for 3 seconds, then return to idle */
            if (xTaskGetTickCount() - g_game_end_ticks > pdMS_TO_TICKS(3000)) {
                pixel_physics_stop();
                display_exit_game_mode();
                pixel_world_destroy();
                g_game_state = GAME_STATE_IDLE;
                display_set_status("Ready! Press DETECT or GAME", 0x00FF00);
                ESP_LOGI(TAG, "Game ended, back to IDLE");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        case GAME_STATE_IDLE:
        default:
            break;
        }

        /* Skip other processing when game is active */
        if (g_game_state != GAME_STATE_IDLE) {
            continue;
        }

        /* 处理检测触发 */
        if (display_detect_triggered()) {
            ESP_LOGI(TAG, "--- Detection triggered ---");

            display_set_status("Status: Capturing...", 0x00FF00);

            camera_frame_t frame;
            ret = camera_capture_frame(&frame, 2000);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Capture FAILED: %s", esp_err_to_name(ret));
                display_set_status("Capture FAILED!", 0xFF0000);
                goto detect_done;
            }

            /* 像素诊断 */
            ESP_LOGI(TAG, "Frame: %dx%d", frame.width, frame.height);
            ESP_LOGI(TAG, "  First 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                     frame.buffer[0], frame.buffer[1], frame.buffer[2], frame.buffer[3],
                     frame.buffer[4], frame.buffer[5], frame.buffer[6], frame.buffer[7]);

            /* 预处理 (仅一次, 检测和预览共用) */
            camera_frame_t display_frame = frame;
            uint8_t *proc_buf = NULL;
            uint32_t preproc_flags = display_get_preproc_flags();

            if (preproc_flags != PREPROC_FLAG_NONE) {
                proc_buf = preprocess_frame_rgb565(
                    frame.buffer, frame.width, frame.height,
                    preproc_flags, NULL);
                if (proc_buf) {
                    display_frame.buffer  = proc_buf;
                    display_frame.buf_len = frame.width * frame.height * 2;
                    ESP_LOGI(TAG, "Preprocessing applied, flags=0x%02X",
                             (unsigned)preproc_flags);
                }
            }

            /* 目标检测 */
            display_set_status("Status: Running detection...", 0x00FF00);

            detection_result_t detections[DETECTION_MAX_OBJECTS];
            int det_count = DETECTION_MAX_OBJECTS;
            ret = detection_run(&display_frame, detections, &det_count, PREPROC_FLAG_NONE);

            /* 更新预览 + 弹珠叠加 (仅赛道捕获后) */
            display_prepare_preview(&display_frame, detections, det_count);
            int pw, ph;
            uint8_t *pbuf = display_get_render_buf(&pw, &ph);
            if (pbuf && track_is_built()) {
                track_render((uint16_t *)pbuf, pw, ph, 0xF800);
                marble_draw((uint16_t *)pbuf, pw, ph);
            }
            display_refresh_preview();

            /* 释放预处理缓冲区 */
            if (proc_buf) {
                heap_caps_free(proc_buf);
                proc_buf = NULL;
            }

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Detection failed: %s", esp_err_to_name(ret));
                display_set_status("Detection FAILED!", 0xFF0000);
            } else if (det_count > 0) {
                char result_str[256];
                int off = snprintf(result_str, sizeof(result_str), "Found %d objects:", det_count);
                for (int i = 0; i < det_count && off < (int)sizeof(result_str) - 30; i++) {
                    off += snprintf(result_str + off, sizeof(result_str) - off,
                                    "\n%s (%.0f%%)", detections[i].label, detections[i].score * 100);
                }
                display_set_status(result_str, 0x00FF00);
                ESP_LOGI(TAG, "Detected %d objects", det_count);
            } else {
                display_set_status("No objects detected\nCheck objects / lighting", 0xFFCC00);
                ESP_LOGW(TAG, "No objects detected");
            }
        }

    detect_done:

        /* ================================================================
         * 边缘检测 / 游戏模式
         *
         * 两阶段:
         *   设置阶段 (track 未捕获): 相机灰度预览, 对准赛道按 Track
         *   游戏阶段 (track 已捕获): 纯画布, 赛道墙壁 + 弹珠, 无相机
         * ================================================================ */
        if (display_is_edge_view()) {
            if (!was_edge_view) {
                if (track_is_built()) {
                    ESP_LOGI(TAG, "Game mode ON");
                    display_set_status("GAME ON  Tilt to play!", 0x00FF00);
                } else {
                    ESP_LOGI(TAG, "Edge view ON (setup)");
                    display_set_status("Point camera at track & press Track",
                                       0xFFCC00);
                }
                was_edge_view = true;
                was_live_view = false;
            }

            /* ---- 游戏阶段: 纯画布渲染, 不读相机 ---- */
            if (track_is_built()) {
                /* 处理重新捕获 (Track 按钮) */
                if (display_track_capture_triggered()) {
                    camera_frame_t frame;
                    ret = camera_capture_frame(&frame, 2000);
                    if (ret == ESP_OK) {
                        /* 延迟初始化 edge (若模型已加载, PSRAM 紧张但仍应够用) */
                        if (!edge_get_downscale_buf()) {
                            edge_detect_init();
                        }
                        uint8_t *ds_buf = edge_get_downscale_buf();
                        uint8_t *emap   = edge_get_edge_map_buf();
                        edge_downscale_half(frame.buffer, frame.width,
                                            frame.height, ds_buf);
                        edge_detect_run(ds_buf, EDGE_DOWNSCALE_W,
                                       EDGE_DOWNSCALE_H, emap,
                                       CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD);

                        track_collision_init(); /* 初次时分配, 后续跳过 */
                        track_build_from_edges(emap, EDGE_DOWNSCALE_W,
                                               EDGE_DOWNSCALE_H);
                        edge_detect_deinit();   /* 释放 ~1MB PSRAM (emap已用完) */

                        /* 同一帧跑检测 + 提取物体轮廓 */
                        {
                            detection_result_t dets[DETECTION_MAX_OBJECTS];
                            int dc = DETECTION_MAX_OBJECTS;
                            if (detection_run(&frame, dets, &dc, PREPROC_FLAG_NONE) == ESP_OK && dc > 0) {
                                game_extract_contours(&frame, dets, dc);
                                ESP_LOGI(TAG, "Track + %d object contours", dc);
                            } else {
                                game_extract_contours(NULL, NULL, 0);
                            }
                        }

                        marble_physics_reset();
                        display_set_status("Track updated!  GAME ON", 0x00FF00);
                        ESP_LOGI(TAG, "Track re-captured");
                    }
                }

                /* 纯画布: 背景 + 赛道墙 + 弹珠 */
                int pw, ph;
                uint8_t *pbuf = display_get_render_buf(&pw, &ph);
                if (pbuf) {
                    game_render_frame((uint16_t *)pbuf, pw, ph);
                }
                display_refresh_preview();
                vTaskDelay(pdMS_TO_TICKS(16));   /* ~60fps 帧率限制 */
            }
            /* ---- 设置阶段: 相机灰度预览 ---- */
            else {
                camera_frame_t frame;
                ret = camera_capture_frame(&frame, 2000);
                if (ret == ESP_OK) {
                    bool just_captured = false;

                    if (display_track_capture_triggered()) {
                        /* 延迟初始化 edge (若模型已加载, PSRAM 紧张但仍应够用) */
                        if (!edge_get_downscale_buf()) {
                            edge_detect_init();
                        }
                        uint8_t *ds_buf = edge_get_downscale_buf();
                        uint8_t *emap   = edge_get_edge_map_buf();
                        edge_downscale_half(frame.buffer, frame.width,
                                            frame.height, ds_buf);
                        edge_detect_run(ds_buf, EDGE_DOWNSCALE_W,
                                       EDGE_DOWNSCALE_H, emap,
                                       CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD);

                        track_collision_init(); /* 初次时分配, 后续跳过 */
                        track_build_from_edges(emap, EDGE_DOWNSCALE_W,
                                               EDGE_DOWNSCALE_H);
                        edge_detect_deinit();   /* 释放 ~1MB PSRAM */

                        /* 同一帧跑检测 + 提取物体轮廓 */
                        {
                            detection_result_t dets[DETECTION_MAX_OBJECTS];
                            int dc = DETECTION_MAX_OBJECTS;
                            if (detection_run(&frame, dets, &dc, PREPROC_FLAG_NONE) == ESP_OK && dc > 0) {
                                game_extract_contours(&frame, dets, dc);
                                ESP_LOGI(TAG, "Track + %d object contours", dc);
                            } else {
                                game_extract_contours(NULL, NULL, 0);
                            }
                        }

                        marble_physics_reset();
                        just_captured = true;
                        ESP_LOGI(TAG, "Track captured — game on!");
                    }

                    if (just_captured) {
                        /* 刚捕获: 立即切到游戏画布 */
                        display_set_status("GAME ON  Tilt to play!", 0x00FF00);
                        int pw, ph;
                        uint8_t *pbuf = display_get_render_buf(&pw, &ph);
                        if (pbuf) {
                            game_render_frame((uint16_t *)pbuf, pw, ph);
                        }
                        display_refresh_preview();
                    } else {
                        /* 未捕获: 灰度预览 + 提示 */
                        display_update_edge_preview(&frame, NULL,
                                                    EDGE_DOWNSCALE_W,
                                                    EDGE_DOWNSCALE_H);
                        display_refresh_preview();
                    }
                }
            }
        }
        /* 实时预览模式 */
        else if (display_is_live_view()) {
            if (!was_live_view) {
                ESP_LOGI(TAG, "Live view ON");
                display_set_status("Live View ON  (press again to stop)", 0x00AAFF);
                was_live_view = true;
                was_edge_view = false;
            }

            camera_frame_t frame;
            ret = camera_capture_frame(&frame, 2000);
            if (ret == ESP_OK) {
                display_prepare_preview(&frame, NULL, 0);
                int pw, ph;
                uint8_t *pbuf = display_get_render_buf(&pw, &ph);
                if (pbuf) {
                    if (track_is_built()) {
                        track_render((uint16_t *)pbuf, pw, ph, 0xF800);
                        marble_draw((uint16_t *)pbuf, pw, ph);
                    }
                }
                display_refresh_preview();
            }
        }
        /* 空闲 */
        else {
            if (was_live_view) {
                ESP_LOGI(TAG, "Live view OFF");
                display_set_status("Live View OFF  (press DETECT)", 0x00FF00);
                was_live_view = false;
            }
            if (was_edge_view) {
                ESP_LOGI(TAG, "Edge view OFF");
                display_set_status("Edge View OFF  (press DETECT)", 0x00FF00);
                was_edge_view = false;
            }

            /* 周期性弹珠状态日志 */
            uint64_t now = esp_timer_get_time();
            if (now - last_status_log > 5000000) {
                last_status_log = now;
                imu_attitude_t att;
                marble_state_t mb;
                marble_physics_get_state(&mb);
                if (imu_get_attitude(&att) == ESP_OK) {
                    ESP_LOGI(TAG, "IMU: p=%.1f r=%.1f | Marble: (%.0f,%.0f) v=(%.0f,%.0f)",
                             att.pitch, att.roll, mb.x, mb.y, mb.vx, mb.vy);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
