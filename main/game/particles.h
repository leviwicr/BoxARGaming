#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PARTICLE_MAX  64

typedef enum {
    PARTICLE_SPARK,     /* wall bounce: white/yellow sparks */
    PARTICLE_FRUIT,     /* fruit pickup: green burst */
    PARTICLE_PORTAL,    /* portal teleport: purple swirl */
    PARTICLE_DEATH,     /* death trap: red explosion */
    PARTICLE_WIN,       /* goal reached: gold fireworks */
} particle_kind_t;

typedef struct {
    float    x, y;        /* position in game map (640×640) */
    float    vx, vy;      /* velocity (px/s) */
    uint8_t  life;        /* remaining life (255→0, fades out) */
    uint8_t  color_idx;   /* 0-15 palette index */
    uint8_t  kind;        /* particle_kind_t */
    bool     alive;
} particle_t;

/* Spawn particles at position */
void particles_spawn(int x, int y, particle_kind_t kind, int count);

/* Update all particles (call at ~30fps, dt in seconds) */
void particles_update(float dt);

/* Render all alive particles into the 640×640 RGB565 game buffer */
void particles_render(uint16_t *buf, int buf_w, int buf_h);

/* Kill all particles immediately */
void particles_clear(void);

#ifdef __cplusplus
}
#endif
