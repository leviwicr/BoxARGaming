#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x, y;
    float vx, vy;
    float rotation;
} marble_state_t;

void marble_physics_init(void);
void marble_physics_get_state(marble_state_t *state);
void marble_physics_reset(void);    /* 重置弹珠到初始位置 (赛道捕获时调用) */
void marble_draw(uint16_t *buf, int w, int h);

#ifdef __cplusplus
}
#endif