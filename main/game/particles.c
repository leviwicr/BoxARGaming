/**
 * Particle Effects System
 *
 * Lightweight pixel particles for game feedback:
 *   - Sparks on wall bounce
 *   - Green burst on fruit pickup
 *   - Purple swirl on portal teleport
 *   - Red explosion on death
 *   - Gold fireworks on win
 *
 * Fixed pool of 64 particles, O(n) update, trivial render.
 */

#include "particles.h"
#include "config.h"
#include <string.h>
#include <math.h>
#include "esp_random.h"

/* Pre-defined palettes (index → RGB565) */
static const uint16_t pal_spark[] = {
    0xFFFF, 0xFFE0, 0xFE40, 0xF800, 0x0000, 0x8410, 0xAD55, 0xD69A,
    0xFFFF, 0xFF60, 0xFD20, 0xFA00, 0x18C3, 0x39E7, 0x630C, 0x9CD3,
};

static const uint16_t pal_fruit[] = {
    0x07E0, 0x0660, 0x05E0, 0x04C0, 0x2720, 0x3FA0, 0x57E0, 0x6FE0,
    0x87E0, 0x06C0, 0x0640, 0x05A0, 0x04A0, 0x3700, 0x47C0, 0x0000,
};

static const uint16_t pal_portal[] = {
    0x801F, 0x981F, 0xB05F, 0xC87F, 0xE0BF, 0xF0DF, 0xF8FF, 0xFCFF,
    0xD81F, 0xE05F, 0xE89F, 0x0000, 0x501F, 0x681F, 0x783F, 0x0000,
};

static const uint16_t pal_death[] = {
    0xF800, 0xF810, 0xF820, 0xF830, 0xF840, 0xE840, 0xD840, 0xC840,
    0xF840, 0xF860, 0xF880, 0xF8A0, 0x0000, 0x3840, 0x5040, 0x7040,
};

static const uint16_t pal_win[] = {
    0xFFE0, 0xFFC0, 0xFFA0, 0xFF80, 0xFF60, 0xFF40, 0xFF20, 0xFF00,
    0xFEE0, 0xFEC0, 0xFEA0, 0xFE80, 0xFE60, 0xFE40, 0xFE20, 0xFE00,
};

static const uint16_t *palettes[] = {
    pal_spark, pal_fruit, pal_portal, pal_death, pal_win
};

static particle_t g_particles[PARTICLE_MAX];

void particles_spawn(int x, int y, particle_kind_t kind, int count)
{
    if (count > PARTICLE_MAX) count = PARTICLE_MAX;

    int spawned = 0;
    for (int i = 0; i < PARTICLE_MAX && spawned < count; i++) {
        if (!g_particles[i].alive) {
            particle_t *p = &g_particles[i];
            p->x = (float)x;
            p->y = (float)y;
            p->alive = true;
            p->life = 255;
            p->kind = (uint8_t)kind;
            p->color_idx = (uint8_t)(esp_random() & 15);

            /* Random velocity based on kind */
            float angle = (float)(esp_random() % 6283) / 1000.0f;
            float speed;
            switch (kind) {
                case PARTICLE_SPARK:
                    speed = 40.0f + (float)(esp_random() % 120);
                    p->life = 180 + (uint8_t)(esp_random() % 75);
                    break;
                case PARTICLE_FRUIT:
                    speed = 30.0f + (float)(esp_random() % 90);
                    p->life = 100 + (uint8_t)(esp_random() % 60);
                    break;
                case PARTICLE_PORTAL:
                    speed = 50.0f + (float)(esp_random() % 150);
                    p->life = 150 + (uint8_t)(esp_random() % 80);
                    break;
                case PARTICLE_DEATH:
                    speed = 60.0f + (float)(esp_random() % 200);
                    p->life = 200 + (uint8_t)(esp_random() % 55);
                    break;
                case PARTICLE_WIN:
                    speed = 80.0f + (float)(esp_random() % 250);
                    p->life = 220 + (uint8_t)(esp_random() % 35);
                    break;
                default:
                    speed = 50.0f;
                    break;
            }
            p->vx = cosf(angle) * speed;
            p->vy = sinf(angle) * speed - 30.0f; /* slight upward bias */
            spawned++;
        }
    }
}

void particles_update(float dt)
{
    for (int i = 0; i < PARTICLE_MAX; i++) {
        particle_t *p = &g_particles[i];
        if (!p->alive) continue;

        /* Fade */
        int fade = (int)(PARTICLE_FADE_RATE * 255.0f * dt);
        if (fade < 1) fade = 1;
        if (p->life <= (uint8_t)fade) {
            p->alive = false;
            continue;
        }
        p->life -= (uint8_t)fade;

        /* Gravity */
        p->vy += PARTICLE_GRAVITY * dt;

        /* Move */
        p->x += p->vx * dt;
        p->y += p->vy * dt;

        /* Bounds kill */
        if (p->x < -8 || p->x > 648 || p->y < -8 || p->y > 648) {
            p->alive = false;
        }
    }
}

void particles_render(uint16_t *buf, int buf_w, int buf_h)
{
    (void)buf_h;
    for (int i = 0; i < PARTICLE_MAX; i++) {
        particle_t *p = &g_particles[i];
        if (!p->alive) continue;

        int px = (int)p->x;
        int py = (int)p->y;
        if (px < 0 || px >= buf_w || py < 0 || py >= buf_h) continue;

        uint16_t color = palettes[p->kind][p->color_idx];

        /* Alpha blend based on remaining life */
        int alpha = (int)p->life;  /* 0-255 */
        uint16_t bg = buf[py * buf_w + px];
        int r = ((color >> 11) & 0x1F) * 255 / 31;
        int g = ((color >> 5)  & 0x3F) * 255 / 63;
        int b = ( color        & 0x1F) * 255 / 31;
        int br = ((bg >> 11) & 0x1F) * 255 / 31;
        int bg_r = ((bg >> 5)  & 0x3F) * 255 / 63;
        int bb = ( bg        & 0x1F) * 255 / 31;

        int ia = 255 - alpha;
        int out_r = (r * alpha + br * ia) / 255;
        int out_g = (g * alpha + bg_r * ia) / 255;
        int out_b = (b * alpha + bb * ia) / 255;

        buf[py * buf_w + px] = (uint16_t)(((out_r >> 3) << 11) | ((out_g >> 2) << 5) | (out_b >> 3));

        /* Also draw a neighbor pixel for brightness > 128 */
        if (alpha > 128) {
            int nx = px + 1;
            if (nx < buf_w) {
                uint16_t bg2 = buf[py * buf_w + nx];
                int br2 = ((bg2 >> 11) & 0x1F) * 255 / 31;
                int bg2_r = ((bg2 >> 5)  & 0x3F) * 255 / 63;
                int bb2 = ( bg2        & 0x1F) * 255 / 31;
                int a2 = alpha / 2;
                int ia2 = 255 - a2;
                out_r = (r * a2 + br2 * ia2) / 255;
                out_g = (g * a2 + bg2_r * ia2) / 255;
                out_b = (b * a2 + bb2 * ia2) / 255;
                buf[py * buf_w + nx] = (uint16_t)(((out_r >> 3) << 11) | ((out_g >> 2) << 5) | (out_b >> 3));
            }
        }
    }
}

void particles_clear(void)
{
    memset(g_particles, 0, sizeof(g_particles));
}
