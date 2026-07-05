#include "marble_physics.h"
#include "config.h"
#include "imu/imu_driver.h"
#include "track/track_collision.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "marble";

#define MARBLE_RADIUS_PX    25
#define ACCEL_MAX           3000.0f
#define FRICTION_COEFF      0.25f
#define BOUND_BOUNCE        0.35f
#define TRACK_BOUNCE        0.55f    /* 赛道墙壁恢复系数 */
#define DEAD_ZONE_DEG       3.0f

static marble_state_t g_marble;
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---- 圆周采样点 (预计算, 避免运行时 sin/cos) ---- */
#define TRACK_SAMPLES  12
static const float sample_cos[TRACK_SAMPLES] = {1,0.866f,0.5f,0,-0.5f,-0.866f,-1,-0.866f,-0.5f,0,0.5f,0.866f};
static const float sample_sin[TRACK_SAMPLES] = {0,0.5f,0.866f,1,0.866f,0.5f,0,-0.5f,-0.866f,-1,-0.866f,-0.5f};

static void physics_task(void *arg)
{
    g_marble.x = MARBLE_INIT_X;
    g_marble.y = MARBLE_INIT_Y;
    g_marble.vx = 0;
    g_marble.vy = 0;
    g_marble.rotation = 0;

    TickType_t last_wake = xTaskGetTickCount();
    const float dt = 1.0f / PHYSICS_UPDATE_HZ;
    const int period_ms = 1000 / PHYSICS_UPDATE_HZ;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));

        imu_attitude_t att;
        if (imu_get_attitude(&att) != ESP_OK) continue;

        float ax = sinf(att.roll  * M_PI / 180.0f) * ACCEL_MAX;
        float ay = sinf(att.pitch * M_PI / 180.0f) * ACCEL_MAX;

        if (fabsf(att.roll)  < DEAD_ZONE_DEG) ax = 0;
        if (fabsf(att.pitch) < DEAD_ZONE_DEG) ay = 0;

        portENTER_CRITICAL(&g_lock);

        /* 半隐式欧拉积分 */
        g_marble.vx += ax * dt;
        g_marble.vy += ay * dt;

        g_marble.vx *= 1.0f - FRICTION_COEFF * dt;
        g_marble.vy *= 1.0f - FRICTION_COEFF * dt;

        /* 试探新位置 */
        float new_x = g_marble.x + g_marble.vx * dt;
        float new_y = g_marble.y + g_marble.vy * dt;

        /* ---- 赛道墙壁碰撞检测 ---- */
        if (track_is_built()) {
            float r = MARBLE_RADIUS_PX;
            float total_nx = 0, total_ny = 0;
            int hits = 0;

            /* 圆周采样 */
            for (int i = 0; i < TRACK_SAMPLES; i++) {
                int sx = (int)(new_x + sample_cos[i] * r);
                int sy = (int)(new_y + sample_sin[i] * r);

                if (track_is_wall(sx, sy)) {
                    float nx, ny;
                    if (track_get_normal(sx, sy, &nx, &ny)) {
                        total_nx += nx;
                        total_ny += ny;
                        hits++;
                    }
                }
            }

            /* 中心点也检测 (防护: 球完全陷入墙壁) */
            if (track_is_wall((int)new_x, (int)new_y)) {
                float nx, ny;
                if (track_get_normal((int)new_x, (int)new_y, &nx, &ny)) {
                    total_nx += nx;
                    total_ny += ny;
                    hits++;
                }
            }

            if (hits > 0) {
                /* 归一化平均法线 */
                float len = sqrtf(total_nx * total_nx + total_ny * total_ny);
                if (len > 0.01f) {
                    total_nx /= len;
                    total_ny /= len;

                    /* 速度反射: v' = v - 2*(v·n)*n * restitution */
                    float vn = g_marble.vx * total_nx + g_marble.vy * total_ny;
                    if (vn < 0) {  /* 仅当朝向墙壁运动时才反弹 */
                        g_marble.vx -= (1.0f + TRACK_BOUNCE) * vn * total_nx;
                        g_marble.vy -= (1.0f + TRACK_BOUNCE) * vn * total_ny;

                        /* 推开: 将球沿法线推出墙壁 */
                        new_x += total_nx * 3.0f;
                        new_y += total_ny * 3.0f;
                    }
                }
            }
        }

        /* 应用位置 */
        g_marble.x = new_x;
        g_marble.y = new_y;

        /* 速度限制 */
        float speed = sqrtf(g_marble.vx * g_marble.vx + g_marble.vy * g_marble.vy);
        if (speed > MARBLE_MAX_SPEED) {
            g_marble.vx *= MARBLE_MAX_SPEED / speed;
            g_marble.vy *= MARBLE_MAX_SPEED / speed;
            speed = MARBLE_MAX_SPEED;
        }

        g_marble.rotation += speed * dt / MARBLE_RADIUS_PX;

        /* 地图边界反弹 (后备) */
        float mr = MARBLE_RADIUS_PX;
        if (g_marble.x < mr)           { g_marble.x = mr;          g_marble.vx = -g_marble.vx * BOUND_BOUNCE; }
        if (g_marble.x > MAP_SIZE - mr){ g_marble.x = MAP_SIZE - mr; g_marble.vx = -g_marble.vx * BOUND_BOUNCE; }
        if (g_marble.y < mr)           { g_marble.y = mr;          g_marble.vy = -g_marble.vy * BOUND_BOUNCE; }
        if (g_marble.y > MAP_SIZE - mr){ g_marble.y = MAP_SIZE - mr; g_marble.vy = -g_marble.vy * BOUND_BOUNCE; }

        portEXIT_CRITICAL(&g_lock);
    }
}

void marble_physics_init(void)
{
    g_marble.x = MARBLE_INIT_X;
    g_marble.y = MARBLE_INIT_Y;
    g_marble.vx = 0;
    g_marble.vy = 0;
    g_marble.rotation = 0;
    xTaskCreate(physics_task, "marble_phy", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Physics engine started (%d Hz, track collision enabled)", PHYSICS_UPDATE_HZ);
}

void marble_physics_get_state(marble_state_t *state)
{
    portENTER_CRITICAL(&g_lock);
    *state = g_marble;
    portEXIT_CRITICAL(&g_lock);
}

void marble_physics_reset(void)
{
    portENTER_CRITICAL(&g_lock);
    g_marble.x = MARBLE_INIT_X;
    g_marble.y = MARBLE_INIT_Y;
    g_marble.vx = 0;
    g_marble.vy = 0;
    g_marble.rotation = 0;
    portEXIT_CRITICAL(&g_lock);
}

/* ---- 3D金属球渲染 + 滚动纹理 + 投影 (不变) ---- */

static inline float sphere_z(int dx, int dy, int r2)
{
    return sqrtf((float)(r2 - dx * dx - dy * dy));
}

void marble_draw(uint16_t *buf, int w, int h)
{
    marble_state_t s;
    marble_physics_get_state(&s);

    /* 坐标变换: 物理 640×640 → 缓冲区 (512×512 game area, x偏移64)
     * 与 track_render() 保持一致 */
    int cx = (int)(s.x * 512.0f / 640.0f + 64);
    int cy = (int)(s.y * 512.0f / 640.0f);
    int r = MARBLE_RADIUS_PX * 512 / 640;
    int r2 = r * r;

    float speed = sqrtf(s.vx * s.vx + s.vy * s.vy);
    float rot_ax = 0, rot_ay = 1.0f;
    if (speed > 0.5f) {
        rot_ax =  s.vy / speed;
        rot_ay = -s.vx / speed;
    }

    float cos_a = cosf(s.rotation);
    float sin_a = sinf(s.rotation);
    float omc  = 1.0f - cos_a;
    float ax2 = rot_ax * rot_ax, ay2 = rot_ay * rot_ay, axay = rot_ax * rot_ay;

    /* ---- 投影 ---- */
    int sh_y = cy + r + 2;
    int sh_rx = r * 4 / 5;
    int sh_ry = r / 3;
    for (int sdy = -sh_ry; sdy <= sh_ry; sdy++) {
        int spy = sh_y + sdy;
        if (spy < 0 || spy >= h) continue;
        float yf = (float)sdy / sh_ry;
        int srx = (int)(sh_rx * sqrtf(1.0f - yf * yf));
        for (int sdx = -srx; sdx <= srx; sdx++) {
            int spx = cx + sdx;
            if (spx < 0 || spx >= w) continue;
            uint16_t bg = buf[spy * w + spx];
            int ir = (bg >> 11) & 0x1F;
            int ig = (bg >> 5)  & 0x3F;
            int ib = bg & 0x1F;
            buf[spy * w + spx] = (uint16_t)(((ir * 2 / 3) << 11) | ((ig * 2 / 3) << 5) | (ib * 2 / 3));
        }
    }

    /* ---- 球体 ---- */
    for (int dy = -r; dy <= r; dy++) {
        int py = cy + dy;
        if (py < 0 || py >= h) continue;
        for (int dx = -r; dx <= r; dx++) {
            int d2 = dx * dx + dy * dy;
            if (d2 > r2) continue;
            int px = cx + dx;
            if (px < 0 || px >= w) continue;

            float z  = sphere_z(dx, dy, r2);
            float nx = dx / (float)r;
            float ny = dy / (float)r;
            float nz = z  / (float)r;

            /* 旋转法线 */
            float rnx = (cos_a + omc * ax2) * nx + (omc * axay) * ny       + (sin_a * rot_ay) * nz;
            float rny = (omc * axay) * nx       + (cos_a + omc * ay2) * ny + (-sin_a * rot_ax) * nz;
            float rnz = (-sin_a * rot_ay) * nx  + (sin_a * rot_ax) * ny    + cos_a * nz;

            /* 描边 */
            if (d2 >= (r - 2) * (r - 2)) {
                buf[py * w + px] = 0x0000;
                continue;
            }

            /* 纹理: 网球式曲线 + 极点圆斑 */
            float curve1 = rnx - 0.28f * sinf(rny * M_PI * 2.0f);
            float curve2 = rny - 0.28f * sinf(rnx * M_PI * 2.0f);
            int on_curve = (fabsf(curve1) < 0.055f || fabsf(curve2) < 0.055f);

            float max_axis = fmaxf(fmaxf(fabsf(rnx), fabsf(rny)), fabsf(rnz));
            int on_spot = (max_axis > 0.935f);

            /* 光照: 光源左上 */
            float diffuse = -0.42f * nx + -0.57f * ny + 0.70f * nz;
            if (diffuse < 0) diffuse = 0;

            /* 高光 */
            float rx_ = 2.0f * diffuse * nx + 0.42f;
            float ry_ = 2.0f * diffuse * ny + 0.57f;
            float rz_ = 2.0f * diffuse * nz - 0.70f;
            float spec = rz_;
            if (spec < 0) spec = 0;
            spec = spec * spec; spec = spec * spec;
            spec = spec * spec;
            spec *= 0.5f;

            float shade = 0.18f + 0.50f * diffuse + spec;
            if (shade > 1.0f) shade = 1.0f;
            if (shade < 0.06f) shade = 0.06f;

            int ir, ig, ib;
            if (on_spot) {
                ir = (int)(shade * 30 + 8);
                ig = (int)(shade * 30 + 8);
                ib = (int)(shade * 35 + 8);
            } else if (on_curve) {
                ir = (int)(shade * 55 + 15);
                ig = (int)(shade * 55 + 15);
                ib = (int)(shade * 60 + 15);
            } else {
                ir = (int)(shade * 200 + 18);
                ig = (int)(shade * 210 + 15);
                ib = (int)(shade * 230 + 12);
            }
            if (ir > 255) ir = 255;
            if (ig > 255) ig = 255;
            if (ib > 255) ib = 255;

            buf[py * w + px] = (uint16_t)(((ir >> 3) << 11) | ((ig >> 2) << 5) | (ib >> 3));
        }
    }
}
