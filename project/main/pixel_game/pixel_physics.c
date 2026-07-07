/**
 * 游戏物理回调 — 处理弹珠与游戏物体的交互
 *
 * 在 100Hz 物理任务中调用, 实现:
 *   - 水果拾取 (激活穿墙Buff)
 *   - 传送门 (配对鼠标间传送)
 *   - 死亡陷阱 (剪刀)
 *   - 终点检测 (瓶子)
 *   - 弹力表面区域 (杯/勺/键盘/手机)
 */

#include "pixel_physics.h"
#include "pixel_world.h"
#include "physics/marble_physics.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "pix_phys";

static float g_current_bounce = GAME_BOUNCE_DEFAULT;

/* ---- Callback invoked from physics task at 100Hz ---- */
static void physics_callback(marble_state_t *marble, float dt)
{
    pixel_world_t *world = pixel_world_get();
    if (!world || !pixel_world_is_built()) return;

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
            /* Pickup on proximity */
            if (dist < contact_dist) {
                obj->alive = false;
                marble_activate_wall_pass(GAME_WALL_PASS_MS);
                ESP_LOGI(TAG, "Fruit picked up! Wall-pass: %d ms",
                         GAME_WALL_PASS_MS);
            }
            break;

        case GAMEOBJ_DEATH:
            /* Death on contact */
            if (dist < contact_dist) {
                world->player_dead = true;
                ESP_LOGI(TAG, "Scissors touched! GAME OVER");
            }
            break;

        case GAMEOBJ_GOAL:
            /* Win on contact */
            if (dist < contact_dist) {
                world->goal_reached = true;
                ESP_LOGI(TAG, "Bottle reached! YOU WIN");
            }
            break;

        case GAMEOBJ_PORTAL:
            /* Teleport on entering portal zone */
            if (dist < (float)obj->radius && obj->cooldown == 0) {
                int partner = obj->partner_id;
                if (partner >= 0 && partner < world->object_count) {
                    game_object_t *dst = &world->objects[partner];
                    if (dst->alive && dst->type == GAMEOBJ_PORTAL) {
                        /* Teleport to paired portal */
                        marble_set_position((float)dst->pixel_x,
                                           (float)dst->pixel_y);
                        /* Randomize direction, preserve speed */
                        float speed = sqrtf(marble->vx * marble->vx +
                                           marble->vy * marble->vy);
                        float angle = (float)(rand() % 6283) / 1000.0f;
                        marble_set_velocity(cosf(angle) * speed,
                                           sinf(angle) * speed);
                        /* Both portals enter cooldown */
                        obj->cooldown = 100;  /* 100 ticks @ 100Hz = 1s */
                        dst->cooldown = 100;
                        ESP_LOGI(TAG, "Portal: [%d] → [%d]!", i, partner);
                    }
                }
            }
            break;

        case GAMEOBJ_SURFACE:
            /* Surface bounce zone */
            if (dist < (float)obj->radius) {
                marble_set_bounce_mult(obj->bounce_mult);
                g_current_bounce = GAME_BOUNCE_DEFAULT * obj->bounce_mult;
            }
            break;

        default:
            break;
        }
    }

    /* Portal cooldown tick */
    for (int i = 0; i < world->object_count; i++) {
        game_object_t *obj = &world->objects[i];
        if (obj->type == GAMEOBJ_PORTAL && obj->cooldown > 0) {
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
