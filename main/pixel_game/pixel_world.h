#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "detection/detection_driver.h"
#include "camera/camera_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Tile types ---- */
typedef enum {
    TILE_EMPTY       = 0,  /* free space */
    TILE_STONE_WALL  = 1,  /* indestructible wall (from edge lines) */
    TILE_BOOK_WALL_3 = 2,  /* book wall: 3 hits remaining */
    TILE_BOOK_WALL_2 = 4,  /* book wall: 2 hits remaining */
    TILE_BOOK_WALL_1 = 5,  /* book wall: 1 hit remaining */
    TILE_SPOON_WALL  = 6,  /* spoon wall: stone-like, low bounce */
    TILE_BROKEN      = 3,  /* destroyed book wall (passable) */
} tile_type_t;

/* Check whether a tile type is any book wall variant (still solid) */
static inline bool tile_is_book_wall(tile_type_t t)
{
    return t == TILE_BOOK_WALL_3 || t == TILE_BOOK_WALL_2 || t == TILE_BOOK_WALL_1;
}

/* ---- Game object types ---- */
typedef enum {
    GAMEOBJ_FRUIT   = 1,  /* apple/orange/banana → pickup → wall-pass buff */
    GAMEOBJ_PORTAL  = 2,  /* mouse → teleport pair */
    GAMEOBJ_DEATH   = 3,  /* scissors → instant death */
    GAMEOBJ_GOAL    = 5,  /* bottle → win condition */
    GAMEOBJ_SURFACE = 6,  /* keyboard/phone → bounce modifier; cup → capture & aim */
} gameobj_type_t;

/* ---- Game object instance ---- */
typedef struct {
    gameobj_type_t type;
    int  coco_id;             /* original COCO class ID */
    int  pixel_x, pixel_y;    /* center in game map (0..639) */
    int  radius;              /* collision radius (pixels) */
    bool alive;               /* false = picked up / consumed */
    int  partner_id;          /* portal pair index, -1 if none */
    int  cooldown;            /* portal cooldown counter (ticks) */
    float bounce_mult;        /* bounce multiplier (only SURFACE type) */
} game_object_t;

/* ---- Game world ---- */
typedef struct {
    uint8_t        tilemap[40][40];  /* 40×40 tiles (TILE_*) */
    game_object_t  objects[10];
    int            object_count;
    bool           goal_reached;
    bool           player_dead;

    /* Gameplay state */
    int            score;            /* current score */
    int            lives;            /* remaining lives */
    int            time_left_sec;    /* countdown timer (seconds) */
    int            total_time_sec;   /* starting time for this round */
    difficulty_t   difficulty;       /* current difficulty level */
    bool           respawning;       /* true during respawn delay */
    int            respawn_timer_ms; /* countdown to respawn */

    /* Cup capture & aim */
    bool           cup_aiming;       /* true during cup aiming phase */
    int            cup_aim_timer_ms; /* aiming countdown */
    float          cup_aim_angle;    /* current aim direction (radians) */
    int            cup_aim_cx;       /* cup center X for arrow rendering */
    int            cup_aim_cy;       /* cup center Y for arrow rendering */
} pixel_world_t;

/* ---- Global world instance ---- */
void         pixel_world_init(void);
void         pixel_world_build(const camera_frame_t *frame,
                               const uint8_t *edge_map, int ew, int eh,
                               const detection_result_t *detections, int det_count);
pixel_world_t *pixel_world_get(void);
bool         pixel_world_is_built(void);
void         pixel_world_destroy(void);

/* Tile queries */
tile_type_t  pixel_world_get_tile(int tx, int ty);
bool         pixel_world_is_solid(int tx, int ty);
void         pixel_world_destroy_tile(int tx, int ty);

/* Object queries */
game_object_t *pixel_world_find_object_at(int px, int py);
game_object_t *pixel_world_get_object_by_id(int id);

/* Get COCO class display name (Chinese abbreviation) */
const char *pixel_world_coco_name(int coco_id);

/* Score and lives management */
void pixel_world_add_score(int points);
void pixel_world_lose_life(void);
bool pixel_world_is_respawning(void);
int  pixel_world_respawn_remaining_ms(void);

/* Difficulty */
void         pixel_world_set_difficulty(difficulty_t diff);
difficulty_t pixel_world_get_difficulty(void);
const char  *pixel_world_difficulty_name(difficulty_t diff);
int          pixel_world_difficulty_time(difficulty_t diff);
int          pixel_world_difficulty_lives(difficulty_t diff);
float        pixel_world_difficulty_friction(difficulty_t diff);

#ifdef __cplusplus
}
#endif
