/**
 * 像素精灵系统 — 支持变长尺寸的精灵生成
 *
 * 瓦片精灵: 16×16 (固定, TILE_SPRITE_SIZE)
 * 物体精灵: PNG资源或程序化生成, 尺寸不限
 *
 * PNG精灵 (apple/orange/bottle/mouse): 数据在 flash .rodata
 * 程序化精灵 (其余): 数据分配在 PSRAM, sprite_deinit() 释放。
 */

#include "pixel_sprite.h"
#include "../source/png_assets.h"
#include "config.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---- RGB565 helpers ---- */
#define RGB(r,g,b)  ((uint16_t)(((r>>3)<<11)|((g>>2)<<5)|(b>>3)))
#define MAGENTA     RGB(255, 0, 255)

/* ---- Sprite storage ---- */
static sprite_t g_tile_stone_wall;
static sprite_t g_tile_book_wall;
static sprite_t g_tile_broken;

static sprite_t g_obj_apple;
static sprite_t g_obj_orange;
static sprite_t g_obj_banana;
static sprite_t g_obj_mouse;
static sprite_t g_obj_scissors;
static sprite_t g_obj_bottle;
static sprite_t g_obj_cup;
static sprite_t g_obj_spoon;
static sprite_t g_obj_keyboard;
static sprite_t g_obj_cellphone;
static sprite_t g_obj_marble;

/* ---- Helper: alloc zeroed pixel buffer ---- */
static uint16_t *sprite_alloc(int w, int h)
{
    size_t sz = (size_t)w * h * 2;
    return (uint16_t *)heap_caps_calloc(1, sz,
                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static inline void set_px(uint16_t *pixels, int w, int h, int x, int y, uint16_t c)
{
    if (x >= 0 && x < w && y >= 0 && y < h)
        pixels[y * w + x] = c;
}

static void fill_rect(uint16_t *p, int w, int h,
                       int x0, int y0, int x1, int y1, uint16_t c)
{
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            set_px(p, w, h, x, y, c);
}

static void draw_circle(uint16_t *p, int w, int h,
                         int cx, int cy, int r, uint16_t c)
{
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx*dx + dy*dy <= r*r)
                set_px(p, w, h, cx+dx, cy+dy, c);
}

static void draw_circle_outline(uint16_t *p, int w, int h,
                                 int cx, int cy, int r, int thickness, uint16_t c)
{
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++) {
            int d2 = dx*dx + dy*dy;
            if (d2 <= r*r && d2 >= (r-thickness)*(r-thickness))
                set_px(p, w, h, cx+dx, cy+dy, c);
        }
}

/* ========================================================================
 * 16×16 Tile sprites (unchanged)
 * ======================================================================== */
#define TW  16
#define TH  16

static void gen_stone_wall(uint16_t *p)
{
    for (int y = 0; y < TH; y++) {
        int brick_row = y / 4;
        int offset = (brick_row & 1) ? 4 : 0;
        for (int x = 0; x < TW; x++) {
            bool mortar_h = (y % 4 == 0);
            bool mortar_v = ((x + offset) % 8 == 0);
            int bx = (x + offset) / 8;
            uint16_t c;
            if (mortar_h || mortar_v)
                c = RGB(68, 68, 72);
            else if ((bx + brick_row) & 1)
                c = RGB(120, 116, 112);
            else
                c = RGB(100, 96, 92);
            set_px(p, TW, TH, x, y, c);
        }
    }
}

static void gen_book_wall(uint16_t *p)
{
    /* Single horizontal row of books — compact, distinct from stone wall.
     * Dark background + tightly packed book spines in one row. */

    /* Dark wood background */
    for (int y = 0; y < TH; y++)
        for (int x = 0; x < TW; x++)
            set_px(p, TW, TH, x, y, RGB(38, 28, 18));

    /* Books: one row, each book is 3px wide, 10px tall, centered vertically */
    /* Spines arranged left to right with alternating colors */
    uint16_t colors[] = {
        RGB(170, 40, 30),   /* dark red    */
        RGB(25, 75, 150),   /* blue        */
        RGB(30, 130, 50),   /* green       */
        RGB(160, 120, 30),  /* brown/gold  */
        RGB(130, 40, 130),  /* purple      */
    };
    int n_colors = sizeof(colors) / sizeof(colors[0]);

    for (int i = 0; i < 5; i++) {
        int x0 = 1 + i * 3;
        int bw = (i == 4) ? 2 : 3;  /* last book slightly narrower */
        uint16_t spine = colors[i];

        for (int y = 3; y < 13; y++) {
            for (int x = x0; x < x0 + bw; x++) {
                uint16_t c = spine;
                /* Top highlight on spine */
                if (y == 3) c = RGB(255, 255, 200);  /* page edge */
                if (y == 4) {
                    int sr = ((spine >> 11) & 0x1F) * 4 / 5;
                    int sg = ((spine >> 5)  & 0x3F) * 4 / 5;
                    int sb = ( spine        & 0x1F) * 4 / 5;
                    c = (uint16_t)((sr << 11) | (sg << 5) | sb);
                }
                /* Right edge shadow */
                if (x == x0 + bw - 1) {
                    int sr = ((spine >> 11) & 0x1F) / 2;
                    int sg = ((spine >> 5)  & 0x3F) / 2;
                    int sb = ( spine        & 0x1F) / 2;
                    c = (uint16_t)((sr << 11) | (sg << 5) | sb);
                }
                set_px(p, TW, TH, x, y, c);
            }
        }
    }
}

static void gen_broken(uint16_t *p)
{
    for (int y = 0; y < TH; y++)
        for (int x = 0; x < TW; x++)
            set_px(p, TW, TH, x, y, RGB(40, 44, 38));

    static const int debris[][3] = {
        {2,3,2},{7,2,2},{11,4,2},{14,3,1},
        {3,8,3},{9,7,2},{13,8,2},
        {1,12,2},{5,11,2},{8,13,2},{12,12,3},
    };
    int n = sizeof(debris) / sizeof(debris[0]);
    for (int i = 0; i < n; i++)
        draw_circle(p, TW, TH, debris[i][0], debris[i][1], debris[i][2], RGB(80, 76, 72));
}

/* ========================================================================
 * 32×32 Object sprites (redrawn at larger size)
 * ======================================================================== */
#define OW  OBJ_SPRITE_SIZE
#define OH  OBJ_SPRITE_SIZE

/* Fill entire sprite with magenta key */
static void fill_magenta(uint16_t *p, int w, int h)
{
    for (int i = 0; i < w * h; i++) p[i] = MAGENTA;
}

/* ---- banana 32×32 ---- */
static void gen_banana_32(uint16_t *p)
{
    fill_magenta(p, OW, OH);
    /* Crescent body: thicker arc */
    for (int y = 0; y < OH; y++) {
        for (int x = 0; x < OW; x++) {
            int dx = x - 16, dy = y - 16;
            int rx = (dx * 7 + dy * 3) / 10;
            int ry = (-dx * 3 + dy * 7) / 10;
            int dist = (rx * rx) / 36 + (ry * ry) / 81;
            if (dist <= 19 && ry > -6) {
                /* Gradient from light to darker yellow */
                int grad = 200 + ry * 4;
                if (grad > 255) grad = 255;
                if (grad < 140) grad = 140;
                set_px(p, OW, OH, x, y, RGB(grad, (grad-50), 20));
            }
        }
    }
    /* Tips */
    draw_circle(p, OW, OH, 4, 12, 2, RGB(140, 110, 15));
    draw_circle(p, OW, OH, 27, 22, 2, RGB(140, 110, 15));
    /* Highlight stripe */
    for (int y = 8; y < 20; y++)
        for (int x = 10; x < 22; x++) {
            int dx = x - 16, dy = y - 16;
            int rx = (dx * 7 + dy * 3) / 10;
            int ry = (-dx * 3 + dy * 7) / 10;
            if (rx*rx/4 + ry*ry/16 <= 5 && ry > -3)
                set_px(p, OW, OH, x, y, RGB(255, 240, 80));
        }
}

/* ---- scissors 32×32 ---- */
static void gen_scissors_32(uint16_t *p)
{
    fill_magenta(p, OW, OH);
    /* X-shaped blades (thicker) */
    for (int t = -2; t <= 2; t++) {
        for (int i = -12; i <= 12; i++) {
            set_px(p, OW, OH, 16+i, 16+i+t, RGB(192, 196, 200));
            set_px(p, OW, OH, 16+i, 16-i+t, RGB(192, 196, 200));
        }
    }
    /* Blade center line highlights */
    for (int i = -10; i <= 10; i++) {
        set_px(p, OW, OH, 16+i, 16+i, RGB(230, 234, 238));
        set_px(p, OW, OH, 16+i, 16-i, RGB(230, 234, 238));
    }
    /* Handle rings (4 corners) */
    draw_circle_outline(p, OW, OH, 6, 6, 6, 3, RGB(200, 50, 30));
    draw_circle_outline(p, OW, OH, 25, 25, 6, 3, RGB(200, 50, 30));
    draw_circle_outline(p, OW, OH, 25, 6, 6, 3, RGB(200, 50, 30));
    draw_circle_outline(p, OW, OH, 6, 25, 6, 3, RGB(200, 50, 30));
    /* Center rivet */
    draw_circle(p, OW, OH, 16, 16, 3, RGB(140, 144, 148));
    draw_circle(p, OW, OH, 16, 16, 1, RGB(200, 204, 208));
    /* Danger indicators (small red dots near tips) */
    draw_circle(p, OW, OH, 3, 3, 1, RGB(255, 60, 40));
    draw_circle(p, OW, OH, 28, 28, 1, RGB(255, 60, 40));
}

/* ---- cup 32×32 ---- */
static void gen_cup_32(uint16_t *p)
{
    fill_magenta(p, OW, OH);
    /* Cup body */
    fill_rect(p, OW, OH, 6, 6, 25, 28, RGB(230, 230, 235));
    /* Shadow side */
    fill_rect(p, OW, OH, 6, 6, 10, 28, RGB(200, 200, 210));
    /* Handle */
    fill_rect(p, OW, OH, 25, 10, 29, 22, RGB(210, 210, 218));
    fill_rect(p, OW, OH, 26, 12, 28, 20, MAGENTA);   /* hole */
    /* Decorative stripe */
    fill_rect(p, OW, OH, 6, 14, 25, 16, RGB(60, 100, 210));
    /* Rim */
    fill_rect(p, OW, OH, 4, 4, 27, 6, RGB(250, 250, 255));
    /* Bottom */
    fill_rect(p, OW, OH, 8, 27, 23, 28, RGB(180, 180, 190));
}

/* ---- spoon 32×32 ---- */
static void gen_spoon_32(uint16_t *p)
{
    fill_magenta(p, OW, OH);
    /* Handle */
    fill_rect(p, OW, OH, 14, 0, 17, 12, RGB(180, 184, 190));
    /* Handle highlight */
    fill_rect(p, OW, OH, 14, 1, 15, 11, RGB(210, 214, 220));
    /* Bowl (oval head) */
    for (int y = 12; y < 29; y++)
        for (int x = 6; x < 26; x++) {
            int dx = x - 16, dy = y - 20;
            if ((dx*dx)/49 + (dy*dy)/81 <= 6)
                set_px(p, OW, OH, x, y, RGB(200, 204, 210));
        }
    /* Bowl highlight */
    for (int y = 13; y < 25; y++)
        for (int x = 8; x < 14; x++) {
            int dx = x - 16, dy = y - 20;
            if ((dx*dx)/49 + (dy*dy)/81 <= 6 && x < 12)
                set_px(p, OW, OH, x, y, RGB(235, 238, 243));
        }
    /* Bowl inner shadow */
    for (int y = 15; y < 24; y++)
        for (int x = 18; x < 24; x++) {
            int dx = x - 16, dy = y - 20;
            if ((dx*dx)/49 + (dy*dy)/81 <= 6 && x > 19)
                set_px(p, OW, OH, x, y, RGB(150, 154, 160));
        }
    /* Handle end */
    draw_circle(p, OW, OH, 15, 1, 2, RGB(200, 204, 210));
}

/* ---- keyboard 32×32 ---- */
static void gen_keyboard_32(uint16_t *p)
{
    fill_magenta(p, OW, OH);
    /* Base plate */
    fill_rect(p, OW, OH, 2, 5, 29, 28, RGB(50, 52, 56));
    /* Top row */
    for (int col = 0; col < 8; col++)
        fill_rect(p, OW, OH, 3+col*4, 6, 5+col*4, 10, RGB(190, 194, 200));
    /* Second row */
    for (int col = 0; col < 8; col++)
        fill_rect(p, OW, OH, 3+col*4, 12, 5+col*4, 16, RGB(190, 194, 200));
    /* Third row */
    for (int col = 0; col < 8; col++)
        fill_rect(p, OW, OH, 3+col*4, 18, 5+col*4, 22, RGB(190, 194, 200));
    /* Space bar */
    fill_rect(p, OW, OH, 8, 24, 23, 26, RGB(190, 194, 200));
    /* Home row bumps (F, J) */
    set_px(p, OW, OH, 12, 14, RGB(255, 255, 0));
    set_px(p, OW, OH, 20, 14, RGB(255, 255, 0));
    /* Cable */
    fill_rect(p, OW, OH, 14, 2, 17, 5, RGB(70, 72, 76));
}

/* ---- cellphone 32×32 ---- */
static void gen_cellphone_32(uint16_t *p)
{
    fill_magenta(p, OW, OH);
    /* Body */
    fill_rect(p, OW, OH, 6, 2, 25, 30, RGB(30, 32, 36));
    /* Edge highlight */
    fill_rect(p, OW, OH, 6, 2, 25, 3, RGB(60, 62, 66));
    /* Screen */
    fill_rect(p, OW, OH, 8, 5, 23, 24, RGB(40, 120, 230));
    /* Screen gloss line */
    fill_rect(p, OW, OH, 8, 5, 23, 6, RGB(80, 160, 250));
    /* App icons */
    fill_rect(p, OW, OH, 10, 8, 12, 10, RGB(240, 80, 60));
    fill_rect(p, OW, OH, 15, 8, 17, 10, RGB(60, 210, 80));
    fill_rect(p, OW, OH, 20, 8, 22, 10, RGB(240, 200, 40));
    fill_rect(p, OW, OH, 10, 12, 12, 14, RGB(100, 110, 240));
    fill_rect(p, OW, OH, 15, 12, 17, 14, RGB(240, 160, 60));
    fill_rect(p, OW, OH, 20, 12, 22, 14, RGB(160, 80, 220));
    /* Home button */
    fill_rect(p, OW, OH, 13, 27, 18, 29, RGB(80, 82, 86));
    draw_circle(p, OW, OH, 15, 28, 1, RGB(120, 122, 126));
    /* Speaker */
    fill_rect(p, OW, OH, 11, 0, 20, 1, RGB(80, 82, 86));
    /* Camera */
    draw_circle(p, OW, OH, 21, 1, 1, RGB(40, 44, 50));
}

/* ---- marble 32×32 ---- */
static void gen_marble_32(uint16_t *p)
{
    fill_magenta(p, OW, OH);
    int cx = 16, cy = 16, r = 14;

    for (int y = 0; y < OH; y++) {
        for (int x = 0; x < OW; x++) {
            int dx = x - cx, dy = y - cy;
            int d2 = dx * dx + dy * dy;
            if (d2 > r * r) continue;

            /* Outline ring (outer 2px) */
            if (d2 >= (r - 2) * (r - 2)) {
                set_px(p, OW, OH, x, y, RGB(48, 48, 54));
                continue;
            }

            /* Normalized direction from center */
            float nx = dx / (float)r, ny = dy / (float)r;
            float nz = sqrtf(1.0f - (float)d2 / (r * r));

            /* Lighting: light from top-left */
            float diffuse = -0.45f * nx + -0.35f * ny + 0.82f * nz;
            if (diffuse < 0.0f) diffuse = 0.0f;

            /* Specular highlight */
            float hx = (nx - 0.55f) * 2.5f;
            float hy = (ny + 0.45f) * 2.5f;
            float spec = 1.0f - (hx * hx + hy * hy);
            if (spec < 0.0f) spec = 0.0f;
            spec = spec * spec * spec * spec * 0.7f;

            /* Secondary bounce light from bottom-right */
            float bounce = 0.12f * (1.0f - nz);
            float rim = (1.0f - nz) * (1.0f - nz) * 0.15f;

            float shade = 0.22f + 0.52f * diffuse + spec + bounce + rim;
            if (shade > 1.0f) shade = 1.0f;

            /* Steel-blue marble tone */
            int ir = (int)(shade * 180 + 15);
            int ig = (int)(shade * 195 + 18);
            int ib = (int)(shade * 220 + 20);
            if (ir > 255) ir = 255;
            if (ig > 255) ig = 255;
            if (ib > 255) ib = 255;

            set_px(p, OW, OH, x, y, RGB(ir, ig, ib));
        }
    }

    /* Sharp specular dot */
    for (int dy = -2; dy <= 2; dy++)
        for (int dx = -2; dx <= 2; dx++)
            if (dx * dx + dy * dy <= 3)
                set_px(p, OW, OH, 9 + dx, 9 + dy, RGB(255, 255, 255));
}

/* ========================================================================
 * Public API
 * ======================================================================== */

void sprite_init(void)
{
    /* ---- 16×16 tile sprites ---- */
    g_tile_stone_wall.w = TW; g_tile_stone_wall.h = TH;
    g_tile_book_wall.w  = TW; g_tile_book_wall.h  = TH;
    g_tile_broken.w     = TW; g_tile_broken.h     = TH;

    g_tile_stone_wall.pixels = sprite_alloc(TW, TH);
    g_tile_book_wall.pixels  = sprite_alloc(TW, TH);
    g_tile_broken.pixels     = sprite_alloc(TW, TH);

    if (g_tile_stone_wall.pixels) gen_stone_wall((uint16_t *)g_tile_stone_wall.pixels);
    if (g_tile_book_wall.pixels)  gen_book_wall((uint16_t *)g_tile_book_wall.pixels);
    if (g_tile_broken.pixels)     gen_broken((uint16_t *)g_tile_broken.pixels);

    /* ---- PNG object sprites (flash, no allocation) ---- */
    g_obj_apple.pixels  = png_apple_data;  g_obj_apple.w  = PNG_APPLE_W;  g_obj_apple.h  = PNG_APPLE_H;
    g_obj_orange.pixels = png_orange_data; g_obj_orange.w = PNG_ORANGE_W; g_obj_orange.h = PNG_ORANGE_H;
    g_obj_bottle.pixels = png_bottle_data; g_obj_bottle.w = PNG_BOTTLE_W; g_obj_bottle.h = PNG_BOTTLE_H;
    g_obj_mouse.pixels  = png_mouse_data;  g_obj_mouse.w  = PNG_MOUSE_W;  g_obj_mouse.h  = PNG_MOUSE_H;

    /* ---- Procedural object sprites (PSRAM) ---- */
    g_obj_banana.w     = OW; g_obj_banana.h     = OH;
    g_obj_scissors.w   = OW; g_obj_scissors.h   = OH;
    g_obj_cup.w        = OW; g_obj_cup.h        = OH;
    g_obj_spoon.w      = OW; g_obj_spoon.h      = OH;
    g_obj_keyboard.w   = OW; g_obj_keyboard.h   = OH;
    g_obj_cellphone.w  = OW; g_obj_cellphone.h  = OH;

    g_obj_banana.pixels     = sprite_alloc(OW, OH);
    g_obj_scissors.pixels   = sprite_alloc(OW, OH);
    g_obj_cup.pixels        = sprite_alloc(OW, OH);
    g_obj_spoon.pixels      = sprite_alloc(OW, OH);
    g_obj_keyboard.pixels   = sprite_alloc(OW, OH);
    g_obj_cellphone.pixels  = sprite_alloc(OW, OH);

    if (g_obj_banana.pixels)     gen_banana_32((uint16_t *)g_obj_banana.pixels);
    if (g_obj_scissors.pixels)   gen_scissors_32((uint16_t *)g_obj_scissors.pixels);
    if (g_obj_cup.pixels)        gen_cup_32((uint16_t *)g_obj_cup.pixels);
    if (g_obj_spoon.pixels)      gen_spoon_32((uint16_t *)g_obj_spoon.pixels);
    if (g_obj_keyboard.pixels)   gen_keyboard_32((uint16_t *)g_obj_keyboard.pixels);
    if (g_obj_cellphone.pixels)  gen_cellphone_32((uint16_t *)g_obj_cellphone.pixels);

    /* ---- Marble sprite (PSRAM) ---- */
    g_obj_marble.w = OW; g_obj_marble.h = OH;
    g_obj_marble.pixels = sprite_alloc(OW, OH);
    if (g_obj_marble.pixels) gen_marble_32((uint16_t *)g_obj_marble.pixels);
}

void sprite_deinit(void)
{
    /* Tile sprites */
    if (g_tile_stone_wall.pixels) { heap_caps_free((void *)g_tile_stone_wall.pixels); g_tile_stone_wall.pixels = NULL; }
    if (g_tile_book_wall.pixels)  { heap_caps_free((void *)g_tile_book_wall.pixels);  g_tile_book_wall.pixels  = NULL; }
    if (g_tile_broken.pixels)     { heap_caps_free((void *)g_tile_broken.pixels);     g_tile_broken.pixels     = NULL; }
    /* PNG object sprites (flash, no free) */
    g_obj_apple.pixels  = NULL;
    g_obj_orange.pixels = NULL;
    g_obj_bottle.pixels = NULL;
    g_obj_mouse.pixels  = NULL;
    /* Procedural object sprites (PSRAM, must free) */
    if (g_obj_banana.pixels)     { heap_caps_free((void *)g_obj_banana.pixels);     g_obj_banana.pixels     = NULL; }
    if (g_obj_scissors.pixels)   { heap_caps_free((void *)g_obj_scissors.pixels);   g_obj_scissors.pixels   = NULL; }
    if (g_obj_cup.pixels)        { heap_caps_free((void *)g_obj_cup.pixels);        g_obj_cup.pixels        = NULL; }
    if (g_obj_spoon.pixels)      { heap_caps_free((void *)g_obj_spoon.pixels);      g_obj_spoon.pixels      = NULL; }
    if (g_obj_keyboard.pixels)   { heap_caps_free((void *)g_obj_keyboard.pixels);   g_obj_keyboard.pixels   = NULL; }
    if (g_obj_cellphone.pixels)  { heap_caps_free((void *)g_obj_cellphone.pixels);  g_obj_cellphone.pixels  = NULL; }
    /* Marble */
    if (g_obj_marble.pixels)     { heap_caps_free((void *)g_obj_marble.pixels);     g_obj_marble.pixels     = NULL; }
}

const sprite_t *sprite_get_tile(tile_type_t type)
{
    switch (type) {
        case TILE_STONE_WALL:
        case TILE_SPOON_WALL:  return &g_tile_stone_wall;
        case TILE_BOOK_WALL_3:
        case TILE_BOOK_WALL_2:
        case TILE_BOOK_WALL_1: return &g_tile_book_wall;
        case TILE_BROKEN:      return &g_tile_broken;
        default:               return NULL;
    }
}

const sprite_t *sprite_get_obj(gameobj_type_t type)
{
    switch (type) {
        case GAMEOBJ_FRUIT:   return NULL;  /* use sprite_get_by_coco for per-fruit sprite */
        case GAMEOBJ_PORTAL:  return &g_obj_mouse;
        case GAMEOBJ_DEATH:   return &g_obj_scissors;
        case GAMEOBJ_GOAL:    return &g_obj_bottle;
        case GAMEOBJ_SURFACE: return NULL;  /* use sprite_get_by_coco for per-surface sprite */
        default:              return NULL;
    }
}

const sprite_t *sprite_get_by_coco(int coco_id)
{
    switch (coco_id) {
        case 47: return &g_obj_apple;      /* apple      */
        case 49: return &g_obj_orange;     /* orange     */
        case 46: return &g_obj_banana;     /* banana     */
        case 39: return &g_obj_bottle;     /* bottle     */
        case 64: return &g_obj_mouse;      /* mouse      */
        case 76: return &g_obj_scissors;   /* scissors   */
        case 41: return &g_obj_cup;        /* cup        */
        case 44: return &g_obj_spoon;      /* spoon      */
        case 66: return &g_obj_keyboard;   /* keyboard   */
        case 67: return &g_obj_cellphone;  /* cell phone */
        default: return NULL;
    }
}

const sprite_t *sprite_get_marble(void)
{
    return &g_obj_marble;
}

void sprite_blit(uint16_t *buf, int buf_w,
                 const sprite_t *s, int dst_x, int dst_y)
{
    if (!buf || !s || !s->pixels) return;
    int sw = s->w, sh = s->h;

    for (int y = 0; y < sh; y++) {
        int by = dst_y + y;
        if (by < 0 || by >= 640) continue;
        for (int x = 0; x < sw; x++) {
            int bx = dst_x + x;
            if (bx < 0 || bx >= buf_w) continue;
            buf[by * buf_w + bx] = s->pixels[y * sw + x];
        }
    }
}

void sprite_blit_keyed(uint16_t *buf, int buf_w,
                       const sprite_t *s, int dst_x, int dst_y,
                       uint16_t color_key)
{
    if (!buf || !s || !s->pixels) return;
    int sw = s->w, sh = s->h;

    for (int y = 0; y < sh; y++) {
        int by = dst_y + y;
        if (by < 0 || by >= 640) continue;
        for (int x = 0; x < sw; x++) {
            int bx = dst_x + x;
            if (bx < 0 || bx >= buf_w) continue;
            uint16_t p = s->pixels[y * sw + x];
            if (p != color_key) {
                buf[by * buf_w + bx] = p;
            }
        }
    }
}

void sprite_blit_keyed_edgeblend(uint16_t *buf, int buf_w,
                                  const sprite_t *s, int dst_x, int dst_y,
                                  uint16_t color_key)
{
    if (!buf || !s || !s->pixels) return;
    int sw = s->w, sh = s->h;

    for (int y = 0; y < sh; y++) {
        int by = dst_y + y;
        if (by < 0 || by >= 640) continue;
        for (int x = 0; x < sw; x++) {
            int bx = dst_x + x;
            if (bx < 0 || bx >= buf_w) continue;
            uint16_t p = s->pixels[y * sw + x];
            if (p == color_key) continue;

            /* Check if any 4-neighbor is transparent (edge pixel) */
            bool is_edge = false;
            if (x > 0     && s->pixels[y * sw + (x - 1)] == color_key) is_edge = true;
            if (x < sw - 1 && s->pixels[y * sw + (x + 1)] == color_key) is_edge = true;
            if (y > 0     && s->pixels[(y - 1) * sw + x] == color_key) is_edge = true;
            if (y < sh - 1 && s->pixels[(y + 1) * sw + x] == color_key) is_edge = true;

            if (is_edge) {
                /* Blend 50/50 with background floor pixel */
                uint16_t bg = buf[by * buf_w + bx];
                int sr = (p >> 11) & 0x1F;
                int sg = (p >> 5)  & 0x3F;
                int sb =  p        & 0x1F;
                int br = (bg >> 11) & 0x1F;
                int bg_g = (bg >> 5)  & 0x3F;
                int bb =  bg        & 0x1F;
                buf[by * buf_w + bx] = (uint16_t)(
                    (((sr + br) / 2) << 11) |
                    (((sg + bg_g) / 2) << 5) |
                    ((sb + bb) / 2));
            } else {
                buf[by * buf_w + bx] = p;
            }
        }
    }
}

void sprite_blit_keyed_scaled(uint16_t *buf, int buf_w,
                               const sprite_t *s, int dst_x, int dst_y,
                               int dst_w, int dst_h, uint16_t color_key)
{
    if (!buf || !s || !s->pixels) return;
    int sw = s->w, sh = s->h;

    for (int dy = 0; dy < dst_h; dy++) {
        int sy = dy * sh / dst_h;
        int by = dst_y + dy;
        if (by < 0 || by >= 640) continue;
        for (int dx = 0; dx < dst_w; dx++) {
            int sx = dx * sw / dst_w;
            uint16_t p = s->pixels[sy * sw + sx];
            if (p == color_key) continue;
            int bx = dst_x + dx;
            if (bx < 0 || bx >= buf_w) continue;
            buf[by * buf_w + bx] = p;
        }
    }
}
