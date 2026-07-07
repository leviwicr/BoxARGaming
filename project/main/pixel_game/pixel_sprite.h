#pragma once

#include <stdint.h>
#include "pixel_world.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 瓦片精灵: 固定 16×16 */
#define TILE_SPRITE_SIZE    16

/* 物体精灵: 尺寸不限, 当前默认 32×32 */
#define OBJ_SPRITE_SIZE     32

typedef struct {
    const uint16_t *pixels;  /* row-major RGB565, allocated in PSRAM or flash */
    uint8_t  w, h;
} sprite_t;

/* ---- Sprite accessors ---- */
const sprite_t *sprite_get_tile(tile_type_t type);
const sprite_t *sprite_get_obj(gameobj_type_t type);
const sprite_t *sprite_get_by_coco(int coco_id);
const sprite_t *sprite_get_marble(void);

/* ---- Blit functions (supports any sprite size) ---- */
void sprite_blit(uint16_t *buf, int buf_w,
                 const sprite_t *s, int dst_x, int dst_y);

void sprite_blit_keyed(uint16_t *buf, int buf_w,
                       const sprite_t *s, int dst_x, int dst_y,
                       uint16_t color_key);

void sprite_blit_keyed_edgeblend(uint16_t *buf, int buf_w,
                                  const sprite_t *s, int dst_x, int dst_y,
                                  uint16_t color_key);

/* Generate all procedural sprites (call once at init) */
void sprite_init(void);

/* Free sprite memory (call when exiting game mode) */
void sprite_deinit(void);

#ifdef __cplusplus
}
#endif
