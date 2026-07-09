/**
 * Main Control Task — 游戏状态机与核心业务逻辑
 *
 * 由 app_main 演化的主控任务, 负责:
 *   - 游戏状态机 (IDLE → CAPTURING → PLAYING → WIN/LOSE)
 *   - 用户交互响应 (Live View / Edge / Game / Detect 按钮)
 *   - 业务逻辑编排 (检测、边缘检测、弹珠物理、渲染)
 *   - 相机帧请求 (通过 Camera Task) 和 IMU 姿态获取 (通过 IMU Task)
 *
 * 与旧 main.c 的行为完全等价, 仅将:
 *   camera_capture_frame() → camera_capture_via_task()
 *   imu_get_attitude()     → imu_get_attitude_via_task()
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "config.h"
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"
#include "display/display_driver.h"
#include "image_processing/image_processing.hpp"
#include "edge_detection/edge_detection.h"
#include "imu/imu_driver.h"
#include "physics/marble_physics.h"
#include "track/track_collision.h"
#include "game/game_render.h"
#include "game/particles.h"
#include "pixel_game/pixel_world.h"
#include "pixel_game/pixel_physics.h"
#include "pixel_game/pixel_sprite.h"
#include "ipc/ipc.h"
#include "tasks/power_mgmt_task.h"

static const char *TAG = "main_control";

/* ---- Game state machine (与旧 main.c 一致) ---- */
typedef enum {
    GAME_STATE_IDLE = 0,
    GAME_STATE_CAPTURING,
    GAME_STATE_PLAYING,
    GAME_STATE_WIN,
    GAME_STATE_LOSE,
} game_state_t;

static game_state_t g_game_state = GAME_STATE_IDLE;
static uint64_t     g_game_end_ticks = 0;

/* Per-game timer state (reset on new game) */
static int64_t  g_play_last_time_us  = 0;
static int64_t  g_play_last_timer_us = 0;
static bool     g_play_timer_inited  = false;

void main_control_task(void *pvParams)
{
    (void)pvParams;

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  AR Sandbox - Main Control Task");
    ESP_LOGI(TAG, "==============================================");

    /* ---- 状态变量 (从旧 main.c 迁移) ---- */
    bool was_live_view = false;
    bool was_edge_view = false;
    uint64_t last_status_log = 0;

    /* ---- 主循环 ---- */
    while (1) {
        /* ================================================================
         * Pixel Game State Machine
         * ================================================================ */
        if (display_game_capture_triggered() && g_game_state == GAME_STATE_IDLE) {
            notify_user_activity();
            pm_resume_all();  /* 快速恢复外设 (相机/背光/IMU) */
            g_game_state = GAME_STATE_CAPTURING;
            ESP_LOGI(TAG, "Game capture triggered!");
        }

        if (display_game_exit_triggered()) {
            if (g_game_state == GAME_STATE_PLAYING ||
                g_game_state == GAME_STATE_WIN ||
                g_game_state == GAME_STATE_LOSE) {
                ESP_LOGI(TAG, "Game exit requested");
                notify_user_activity();
                pixel_physics_stop();
                {
                    audio_cmd_t cmd = { .cmd = AUDIO_CMD_BGM_STOP };
                    xQueueSend(g_audio_cmd_q, &cmd, 0);
                }
                particles_clear();
                display_exit_game_mode();
                pixel_world_destroy();
                sprite_deinit();
                g_game_state = GAME_STATE_IDLE;
                xEventGroupClearBits(g_sys_events, SYS_EVT_GAME_ACTIVE);
                display_set_status("Ready!  Press Detect or Start Game", 0x58A6FF);
                continue;
            }
        }

        switch (g_game_state) {

        case GAME_STATE_CAPTURING: {
            ESP_LOGI(TAG, "--- Game: Capturing frame ---");
            display_set_status("Game: Capturing...", 0x00FF00);

            camera_frame_t frame;
            esp_err_t ret = camera_capture_via_task(&frame, 2000);
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

            /* Mask detected objects from edge map — exclude object edges */
            edge_mask_detections(emap, EDGE_DOWNSCALE_W, EDGE_DOWNSCALE_H,
                                detections, det_count, 2);

            /* Build pixel world from edges + detections */
            sprite_deinit();   /* free previous game's sprites */
            sprite_init();
            pixel_world_build(&frame, emap, EDGE_DOWNSCALE_W, EDGE_DOWNSCALE_H,
                            detections, det_count);
            edge_detect_deinit();

            /* Apply difficulty settings */
            pixel_world_set_difficulty(display_get_difficulty());
            particles_clear();

            /* Reset marble */
            marble_physics_reset();
            marble_set_position(MARBLE_INIT_X, MARBLE_INIT_Y);

            /* Init game mode display */
            ret = display_init_game_mode();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Game mode init FAILED");
                pixel_world_destroy();
                g_game_state = GAME_STATE_IDLE;
                break;
            }

            pixel_physics_start();
            xEventGroupSetBits(g_sys_events, SYS_EVT_GAME_ACTIVE);

            /* Reset per-game timer state */
            g_play_timer_inited = false;

            g_game_state = GAME_STATE_PLAYING;
            {
                audio_cmd_t cmd = { .cmd = AUDIO_CMD_BGM_START };
                xQueueSend(g_audio_cmd_q, &cmd, 0);
            }
            ESP_LOGI(TAG, "Game PLAYING - tilt to move marble!");
            break;
        }

        case GAME_STATE_PLAYING: {
            if (!g_play_timer_inited) {
                g_play_last_time_us = esp_timer_get_time();
                g_play_last_timer_us = g_play_last_time_us;
                g_play_timer_inited = true;
            }

            int64_t now_us = esp_timer_get_time();
            float frame_dt = (float)(now_us - g_play_last_time_us) / 1000000.0f;
            if (frame_dt > 0.1f || frame_dt <= 0.0f) frame_dt = 1.0f / GAME_FPS;
            g_play_last_time_us = now_us;

            pixel_world_t *world = pixel_world_get();
            if (now_us - g_play_last_timer_us >= 1000000 && world && !world->respawning) {
                g_play_last_timer_us = now_us;
                if (world->time_left_sec > 0) {
                    world->time_left_sec--;
                }
                if (world->time_left_sec <= 0) {
                    ESP_LOGI(TAG, "Time's up!");
                    world->time_left_sec = 0;
                    if (world->lives > 0) {
                        pixel_world_lose_life();
                    }
                    if (world->lives <= 0) {
                        world->player_dead = true;
                    }
                }
            }

            /* Update particles */
            particles_update(frame_dt);

            /* Render pixel game world + particles */
            uint16_t *gbuf = (uint16_t *)display_get_game_render_buf(NULL, NULL);
            if (gbuf) {
                game_render_pixel_frame(gbuf, GAME_MAP_PIXELS, GAME_MAP_PIXELS);
                particles_render(gbuf, GAME_MAP_PIXELS, GAME_MAP_PIXELS);
            }
            display_refresh_game();

            /* Update HUD */
            marble_state_t mb;
            marble_physics_get_state(&mb);
            imu_attitude_t att;
            imu_get_attitude_via_task(&att);
            float speed = sqrtf(mb.vx * mb.vx + mb.vy * mb.vy);
            const char *bounce_label = pixel_physics_bounce_label();
            int wp_ms = marble_wall_pass_remaining_ms();

            display_update_game_hud(mb.x, mb.y, speed, att.roll, att.pitch,
                                    wp_ms, bounce_label,
                                    world ? world->objects : NULL,
                                    world ? world->object_count : 0,
                                    world ? world->score : 0,
                                    world ? world->lives : 0,
                                    world ? world->time_left_sec : 0,
                                    world ? world->difficulty : DIFF_NORMAL,
                                    world ? world->respawning : false,
                                    world ? world->cup_aiming : false);

            /* Check win/lose conditions */
            if (world) {
                if (world->goal_reached) {
                    ESP_LOGI(TAG, "*** YOU WIN! Score=%d ***", world->score);
                    {
                        audio_cmd_t cmd = { .cmd = AUDIO_CMD_BGM_STOP };
                        xQueueSend(g_audio_cmd_q, &cmd, 0);
                    }
                    display_show_game_end(true, world->score);
                    g_game_state = GAME_STATE_WIN;
                    g_game_end_ticks = xTaskGetTickCount();
                } else if (world->player_dead && world->lives <= 0) {
                    ESP_LOGI(TAG, "*** GAME OVER ***");
                    {
                        audio_cmd_t cmd = { .cmd = AUDIO_CMD_BGM_STOP };
                        xQueueSend(g_audio_cmd_q, &cmd, 0);
                    }
                    {
                        audio_cmd_t cmd = { .cmd = AUDIO_CMD_PLAY_SFX, .sfx = SFX_LOSE };
                        xQueueSend(g_audio_cmd_q, &cmd, 0);
                    }
                    display_show_game_end(false, world->score);
                    g_game_state = GAME_STATE_LOSE;
                    g_game_end_ticks = xTaskGetTickCount();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1000 / GAME_FPS));
            break;
        }

        case GAME_STATE_WIN:
        case GAME_STATE_LOSE:
        {
            /* Keep rendering so trophy zoom animation + particles play */
            static int64_t last_win_us = 0;
            int64_t now_win_us = esp_timer_get_time();
            if (last_win_us == 0) last_win_us = now_win_us;
            float pdt = (float)(now_win_us - last_win_us) / 1000000.0f;
            if (pdt > 0.1f) pdt = 0.1f;
            last_win_us = now_win_us;

            particles_update(pdt);
            uint16_t *gbuf = (uint16_t *)display_get_game_render_buf(NULL, NULL);
            if (gbuf) {
                game_render_pixel_frame(gbuf, GAME_MAP_PIXELS, GAME_MAP_PIXELS);
                particles_render(gbuf, GAME_MAP_PIXELS, GAME_MAP_PIXELS);
            }
            display_refresh_game();

            if (xTaskGetTickCount() - g_game_end_ticks > pdMS_TO_TICKS(3000)) {
                pixel_physics_stop();
                {
                    audio_cmd_t cmd = { .cmd = AUDIO_CMD_BGM_STOP };
                    xQueueSend(g_audio_cmd_q, &cmd, 0);
                }
                particles_clear();
                display_exit_game_mode();
                pixel_world_destroy();
                sprite_deinit();
                g_game_state = GAME_STATE_IDLE;
                xEventGroupClearBits(g_sys_events, SYS_EVT_GAME_ACTIVE);
                display_set_status("Ready!  Press Detect or Start Game", 0x58A6FF);
                ESP_LOGI(TAG, "Game ended, back to IDLE");
            }
            vTaskDelay(pdMS_TO_TICKS(1000 / GAME_FPS));
            break;
        }

        case GAME_STATE_IDLE:
        default:
            break;
        }

        /* Skip other processing when game is active */
        if (g_game_state != GAME_STATE_IDLE) {
            continue;
        }

        /* ================================================================
         * 检测触发 (Detect 按钮)
         * ================================================================ */
        if (display_detect_triggered()) {
            ESP_LOGI(TAG, "--- Detection triggered ---");
            notify_user_activity();
            pm_resume_all();  /* 快速恢复外设 (相机/背光/IMU) */

            display_set_status("Status: Capturing...", 0x00FF00);

            camera_frame_t frame;
            esp_err_t ret = camera_capture_via_task(&frame, 2000);
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

            /* 预处理 */
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

            /* 更新预览 (仅检测框，无赛道/弹珠叠加) */
            display_prepare_preview(&display_frame, detections, det_count);
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
         * 边缘检测 / 游戏模式 (Edge 视图)
         * ================================================================ */
        if (display_is_edge_view()) {
            if (!was_edge_view) {
                notify_user_activity();
                ESP_LOGI(TAG, "Edge view ON — real-time Canny");
                display_set_status("Edge detection — real-time", 0x58A6FF);
                was_edge_view = true;
                was_live_view = false;
            }

            /* ---- 实时边缘检测显示 ---- */
            {
                camera_frame_t frame;
                esp_err_t ret = camera_capture_via_task(&frame, 2000);
                if (ret == ESP_OK) {
                    /* Run Canny edge detection on every frame */
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

                    /* Display grayscale preview with edge overlay */
                    display_update_edge_preview(&frame, emap,
                                                EDGE_DOWNSCALE_W,
                                                EDGE_DOWNSCALE_H);
                    display_refresh_preview();
                } else {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        }
        /* ================================================================
         * 实时预览模式 (Live View)
         * ================================================================ */
        else if (display_is_live_view()) {
            if (!was_live_view) {
                notify_user_activity();
                was_live_view = true;
                was_edge_view = false;

                if (!track_is_built()) {
                    ESP_LOGI(TAG, "Live view ON — will auto-capture track");
                    display_set_status("请拍摄画面以确定赛道", 0xFFCC00);
                } else {
                    ESP_LOGI(TAG, "Live view ON — game ready");
                    display_set_status("GAME ON  Tilt to play!", 0x00FF00);
                }
            }

            camera_frame_t frame;
            esp_err_t ret = camera_capture_via_task(&frame, 2000);
            if (ret == ESP_OK) {
                /* Auto-capture on first frame if no track yet */
                if (!track_is_built()) {
                    ESP_LOGI(TAG, "Auto-capturing track from Live view...");
                    if (!edge_get_downscale_buf()) edge_detect_init();
                    uint8_t *ds_buf = edge_get_downscale_buf();
                    uint8_t *emap   = edge_get_edge_map_buf();
                    edge_downscale_half(frame.buffer, frame.width, frame.height, ds_buf);
                    edge_detect_run(ds_buf, EDGE_DOWNSCALE_W, EDGE_DOWNSCALE_H,
                                   emap, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD);

                    {
                        detection_result_t dets[DETECTION_MAX_OBJECTS];
                        int dc = DETECTION_MAX_OBJECTS;
                        if (detection_run(&frame, dets, &dc, PREPROC_FLAG_NONE) == ESP_OK && dc > 0) {
                            edge_mask_detections(emap, EDGE_DOWNSCALE_W, EDGE_DOWNSCALE_H,
                                                dets, dc, 2);
                            game_extract_contours(&frame, dets, dc);
                        } else {
                            game_extract_contours(NULL, NULL, 0);
                        }
                    }

                    track_collision_init();
                    track_build_from_edges(emap, EDGE_DOWNSCALE_W, EDGE_DOWNSCALE_H);
                    edge_detect_deinit();
                    marble_physics_reset();
                    display_set_status("GAME ON  Tilt to play!", 0x00FF00);
                    ESP_LOGI(TAG, "Track auto-captured — game on!");
                }

                /* Always show pure camera preview (no overlays) */
                display_prepare_preview(&frame, NULL, 0);
                display_refresh_preview();
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        /* ================================================================
         * 空闲 (IDLE)
         * ================================================================ */
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
                if (imu_get_attitude_via_task(&att) == ESP_OK) {
                    ESP_LOGI(TAG, "IMU: p=%.1f r=%.1f | Marble: (%.0f,%.0f) v=(%.0f,%.0f)",
                             att.pitch, att.roll, mb.x, mb.y, mb.vx, mb.vy);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
