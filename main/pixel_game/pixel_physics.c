/**
 * 游戏物理回调 — 处理弹珠与游戏物体的交互
 *
 * 在 100Hz 物理任务中调用, 实现:
 *   - 水果拾取 (激活穿墙Buff)
 *   - 传送门 (配对鼠标间传送)
 *   - 死亡陷阱 (剪刀)
 *   - 终点检测 (瓶子)
 *   - 杯子捕获 & 瞄准弹射 (倾斜设备选择方向)
 *   - 弹力表面区域 (键盘/手机)
 */

#include "pixel_physics.h"
#include "pixel_world.h"
#include "physics/marble_physics.h"
#include "ipc/ipc.h"
#include "game/particles.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>
#include "esp_random.h"

static const char *TAG = "pix_phys";

static float g_current_bounce = GAME_BOUNCE_DEFAULT;

/* ---- Callback invoked from physics task at 100Hz ---- */
static void physics_callback(marble_state_t *marble, float dt)
{
    pixel_world_t *world = pixel_world_get();
    if (!world || !pixel_world_is_built()) return;

    /* Cup aiming: freeze marble, read tilt for direction, launch on expiry.
     * If respawn was triggered externally (timer expiry), cancel aiming. */
    if (world->cup_aiming) {
        if (world->respawning) {
            world->cup_aiming = false;
        } else {
            marble_set_position((float)world->cup_aim_cx, (float)world->cup_aim_cy);
            marble_set_velocity(0, 0);

            float roll  = marble_get_tilt_roll();
            float pitch = marble_get_tilt_pitch();
            if (fabsf(roll) > 5.0f || fabsf(pitch) > 5.0f) {
                world->cup_aim_angle = atan2f(pitch, roll);
            }

            world->cup_aim_timer_ms -= (int)(dt * 1000.0f);
            if (world->cup_aim_timer_ms <= 0) {
                float a = world->cup_aim_angle;
                float sp = (float)GAME_CUP_LAUNCH_SPEED;
                marble_set_velocity(cosf(a) * sp, sinf(a) * sp);
                world->cup_aiming = false;
                ESP_LOGI(TAG, "Cup launch: angle=%.1f deg, speed=%.0f",
                         (double)(a * 180.0f / M_PI), (double)sp);
            }
            return;
        }
    }

    /* Skip interactions during respawn */
    if (world->respawning) {
        world->respawn_timer_ms -= (int)(dt * 1000.0f);
        if (world->respawn_timer_ms <= 0) {
            world->respawn_timer_ms = 0;
            world->respawning = false;
            marble_set_position(MARBLE_INIT_X, MARBLE_INIT_Y);
            marble_set_velocity(0, 0);
            ESP_LOGI(TAG, "Respawn complete, lives=%d", world->lives);
        }
        return;
    }

    int mx = (int)marble->x;
    int my = (int)marble->y;
    float marble_r = marble_get_radius();

    /* Reset bounce to default each tick; surface zones will override */
    marble_set_bounce_mult(1.0f);
    g_current_bounce = GAME_BOUNCE_DEFAULT;

    for (int i = 0; i < world->object_count; i++) {
        game_object_t *obj = &world->objects[i];
        if (!obj->alive) continue;

        float dx = (float)(mx - obj->pixel_x);
        float dy = (float)(my - obj->pixel_y);
        float dist = sqrtf(dx * dx + dy * dy);
        float contact_dist = marble_r + (float)obj->radius;

        switch (obj->type) {

        case GAMEOBJ_FRUIT:
            if (dist < contact_dist) {
                obj->alive = false;
                marble_activate_wall_pass(GAME_WALL_PASS_MS);
                pixel_world_add_score(GAME_SCORE_FRUIT);
                particles_spawn(obj->pixel_x, obj->pixel_y, PARTICLE_FRUIT, 12);
                ESP_LOGI(TAG, "Fruit picked up! Score=%d", world->score);
                {
                    audio_cmd_t cmd = { .cmd = AUDIO_CMD_PLAY_SFX, .sfx = SFX_FRUIT_PICKUP };
                    xQueueSend(g_audio_cmd_q, &cmd, 0);
                }
            }
            break;

        case GAMEOBJ_DEATH:
            if (dist < contact_dist) {
                particles_spawn(obj->pixel_x, obj->pixel_y, PARTICLE_DEATH, 20);
                pixel_world_lose_life();
                ESP_LOGI(TAG, "Death! Lives=%d", world->lives);
                {
                    audio_cmd_t cmd = { .cmd = AUDIO_CMD_PLAY_SFX, .sfx = SFX_DEATH };
                    xQueueSend(g_audio_cmd_q, &cmd, 0);
                }
            }
            break;

        case GAMEOBJ_GOAL:
            if (dist < contact_dist) {
                world->goal_reached = true;
                pixel_world_add_score(GAME_SCORE_GOAL);
                pixel_world_add_score(world->time_left_sec * GAME_SCORE_TIME_BONUS);
                particles_spawn(obj->pixel_x, obj->pixel_y, PARTICLE_WIN, 30);
                ESP_LOGI(TAG, "GOAL! Final score=%d", world->score);
                {
                    audio_cmd_t cmd = { .cmd = AUDIO_CMD_PLAY_SFX, .sfx = SFX_WIN };
                    xQueueSend(g_audio_cmd_q, &cmd, 0);
                }
            }
            break;

        case GAMEOBJ_PORTAL:
            if (dist < (float)obj->radius && obj->cooldown == 0) {
                int partner = obj->partner_id;
                if (partner >= 0 && partner < world->object_count) {
                    game_object_t *dst = &world->objects[partner];
                    if (dst->alive && dst->type == GAMEOBJ_PORTAL) {
                        particles_spawn(obj->pixel_x, obj->pixel_y, PARTICLE_PORTAL, 8);
                        marble_set_position((float)dst->pixel_x,
                                           (float)dst->pixel_y);
                        particles_spawn(dst->pixel_x, dst->pixel_y, PARTICLE_PORTAL, 8);
                        pixel_world_add_score(GAME_SCORE_PORTAL);
                        float speed = sqrtf(marble->vx * marble->vx +
                                           marble->vy * marble->vy);
                        if (speed < 1.0f) speed = 200.0f;
                        float angle = (float)(esp_random() % 6283) / 1000.0f;
                        marble_set_velocity(cosf(angle) * speed,
                                           sinf(angle) * speed);
                        obj->cooldown = 100;
                        dst->cooldown = 100;
                        ESP_LOGI(TAG, "Portal: [%d] → [%d]! Score=%d",
                                 i, partner, world->score);
                        {
                            audio_cmd_t cmd = { .cmd = AUDIO_CMD_PLAY_SFX, .sfx = SFX_PORTAL };
                            xQueueSend(g_audio_cmd_q, &cmd, 0);
                        }
                    }
                }
            }
            break;

        case GAMEOBJ_SURFACE:
            if (dist < (float)obj->radius) {
                /* Cup: capture & aim mechanic */
                if (obj->coco_id == 41) {
                    if (obj->cooldown == 0 && !world->cup_aiming) {
                        world->cup_aiming = true;
                        world->cup_aim_timer_ms = GAME_CUP_AIM_MS;
                        world->cup_aim_angle = -M_PI / 2.0f;  /* default: upward */
                        world->cup_aim_cx = obj->pixel_x;
                        world->cup_aim_cy = obj->pixel_y;
                        obj->cooldown = GAME_CUP_COOLDOWN_MS / 10;
                        marble_set_position((float)obj->pixel_x, (float)obj->pixel_y);
                        marble_set_velocity(0, 0);
                        {
                            audio_cmd_t cmd = { .cmd = AUDIO_CMD_PLAY_SFX,
                                                .sfx = SFX_FRUIT_PICKUP };
                            xQueueSend(g_audio_cmd_q, &cmd, 0);
                        }
                        ESP_LOGI(TAG, "Cup captured! Aiming...");
                    }
                } else {
                    /* Other surfaces: bounce modifier */
                    marble_set_bounce_mult(obj->bounce_mult);
                    g_current_bounce = GAME_BOUNCE_DEFAULT * obj->bounce_mult;
                }
            }
            break;

        default:
            break;
        }
    }

    /* Cooldown tick (portal + cup) */
    for (int i = 0; i < world->object_count; i++) {
        game_object_t *obj = &world->objects[i];
        if (obj->cooldown > 0) {
            obj->cooldown--;
        }
    }
}

/* ---- Public API ---- */

void pixel_physics_start(void)
{
    g_current_bounce = GAME_BOUNCE_DEFAULT;
    marble_physics_register_game_cb(physics_callback);
    ESP_LOGI(TAG, "Game physics started");
}

void pixel_physics_stop(void)
{
    marble_physics_unregister_game_cb();
    g_current_bounce = GAME_BOUNCE_DEFAULT;
    ESP_LOGI(TAG, "Game physics stopped");
}

const char *pixel_physics_bounce_label(void)
{
    static char buf[32];
    if (g_current_bounce < 0.20f) {
        snprintf(buf, sizeof(buf), "%.2f (LOW)", (double)g_current_bounce);
    } else if (g_current_bounce > 0.70f) {
        snprintf(buf, sizeof(buf), "%.2f (HIGH)", (double)g_current_bounce);
    } else if (g_current_bounce > 0.45f && g_current_bounce < 0.65f) {
        snprintf(buf, sizeof(buf), "%.2f (default)", (double)g_current_bounce);
    } else {
        snprintf(buf, sizeof(buf), "%.2f (MED)", (double)g_current_bounce);
    }
    return buf;
}
