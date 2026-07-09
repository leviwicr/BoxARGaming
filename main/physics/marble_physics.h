#pragma once

#include <stdint.h>
#include <stdbool.h>

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
void marble_physics_reset(void);    /* Reset marble to initial position */
void marble_draw(uint16_t *buf, int w, int h);

/* Draw marble at 1:1 scale onto game buffer (640x640) */
void marble_draw_game(uint16_t *buf, int w, int h);

/* ---- Game mode extensions ---- */

/* Set marble position immediately (for portal teleport) */
void marble_set_position(float x, float y);

/* Set marble velocity immediately */
void marble_set_velocity(float vx, float vy);

/* Get marble radius (pixels in 640×640 game map) */
float marble_get_radius(void);

/* Wall-pass buff: activate for duration_ms, deactivates when expired */
void marble_activate_wall_pass(int duration_ms);
bool marble_has_wall_pass(void);
int  marble_wall_pass_remaining_ms(void);

/* Bounce coefficient override (multiplier on TRACK_BOUNCE) */
void marble_set_bounce_mult(float mult);
float marble_get_bounce_mult(void);

/* Register a per-physics-tick game callback */
typedef void (*marble_game_cb_t)(marble_state_t *marble, float dt);
void marble_physics_register_game_cb(marble_game_cb_t cb);
void marble_physics_unregister_game_cb(void);

/* Expose cached IMU tilt for cup aiming */
float marble_get_tilt_roll(void);
float marble_get_tilt_pitch(void);

#ifdef __cplusplus
}
#endif