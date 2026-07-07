#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Register the game physics callback with the marble physics engine.
 * Call once when entering game PLAYING state. */
void pixel_physics_start(void);

/* Unregister the callback (call when exiting game mode). */
void pixel_physics_stop(void);

/* Get current bounce coefficient as readable text.
 * Returns pointer to static string like "0.55 (default)". */
const char *pixel_physics_bounce_label(void);

#ifdef __cplusplus
}
#endif
