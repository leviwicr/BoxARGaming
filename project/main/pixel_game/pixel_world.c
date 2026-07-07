/**
 * 像素游戏世界 — 数据结构和世界构建
 *
 * 从边缘检测结果和COCO检测结果构建游戏世界:
 *   - 边缘 → 不可破坏石墙瓦片
 *   - COCO物体 → 游戏元素 (道具/陷阱/终点等)
 */

#include "pixel_world.h"
#include "config.h"
#include "track/track_collision.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "pixel_world";

/* ---- Global world instance ---- */
static pixel_world_t g_world;
static bool          g_built = false;

/* ---- COCO class → display name ---- */
static const char *coco_name(int coco_id)
{
    switch (coco_id) {
        case 47: return "Apple";
        case 49: return "Orange";
        case 46: return "Banana";
        case 64: return "Mouse";
        case 76: return "Scissors";
        case 73: return "Book";
        case 39: return "Bottle";
        case 41: return "Cup";
        case 44: return "Spoon";
        case 66: return "Keyboard";
        case 67: return "Phone";
        default: return "?";
    }
}

/* ---- COCO class → game object type ---- */
static gameobj_type_t coco_to_game_type(int coco_id)
{
    switch (coco_id) {
        case 47: /* apple */
        case 49: /* orange */
        case 46: /* banana */
            return GAMEOBJ_FRUIT;
        case 64: return GAMEOBJ_PORTAL;
        case 76: return GAMEOBJ_DEATH;
        case 39: return GAMEOBJ_GOAL;
        case 41: /* cup */
        case 44: /* spoon */
        case 66: /* keyboard */
        case 67: /* cell phone */
            return GAMEOBJ_SURFACE;
        default: return GAMEOBJ_FRUIT;  /* unknown → fruit (harmless pickup) */
    }
}

/* ---- COCO class → object radius ---- */
static int coco_to_radius(int coco_id)
{
    switch (coco_id) {
        case 47: case 49: case 46: return GAME_FRUIT_RADIUS;
        case 64: return GAME_PORTAL_RADIUS;
        case 76: return GAME_DEATH_RADIUS;
        case 39: return GAME_GOAL_RADIUS;
        default: return GAME_SURFACE_RADIUS;
    }
}

/* ---- COCO surface → bounce multiplier ---- */
static float coco_to_bounce(int coco_id)
{
    switch (coco_id) {
        case 41: /* cup */
        case 44: /* spoon */
            return GAME_BOUNCE_LOW / GAME_BOUNCE_DEFAULT;
        case 66: /* keyboard */
            return GAME_BOUNCE_MED / GAME_BOUNCE_DEFAULT;
        case 67: /* cell phone */
            return GAME_BOUNCE_HIGH / GAME_BOUNCE_DEFAULT;
        default:
            return 1.0f;  /* default bounce */
    }
}

/* ========================================================================
 * World Builder
 * ======================================================================== */

void pixel_world_init(void)
{
    memset(&g_world, 0, sizeof(g_world));
    g_built = false;
}

void pixel_world_build(const camera_frame_t *frame,
                       const uint8_t *edge_map, int ew, int eh,
                       const detection_result_t *detections, int det_count)
{
    if (!edge_map || ew < 320 || eh < 320) {
        ESP_LOGE(TAG, "Invalid edge map");
        return;
    }

    pixel_world_init();

    /* ---- 1. Build stone walls from edge map ---- */
    /* Edge map is 400×320, game region is center 320×320 area.
     * Maps to 640×640 tilemap: each 16×16 pixel tile checks the edge map. */
    int wall_count = 0;
    for (int ty = 0; ty < 40; ty++) {
        for (int tx = 0; tx < 40; tx++) {
            /* Tile covers pixels (tx*16 .. tx*16+15, ty*16 .. ty*16+15)
             * in 640×640 game map. This corresponds to:
             * edge_x = 40 + (tx*16)/2  ..  40 + (tx*16+15)/2
             * edge_y = (ty*16)/2        ..  (ty*16+15)/2
             *
             * Simplified: check if any edge pixel exists in the tile region. */
            int edge_x0 = 40 + tx * 8;       /* 40 = (400-320)/2 offset */
            int edge_y0 = ty * 8;
            int edge_x1 = edge_x0 + 7;
            int edge_y1 = edge_y0 + 7;

            bool has_edge = false;
            for (int ey = edge_y0; ey <= edge_y1 && !has_edge; ey++) {
                if (ey < 0 || ey >= eh) continue;
                for (int ex = edge_x0; ex <= edge_x1 && !has_edge; ex++) {
                    if (ex < 0 || ex >= ew) continue;
                    if (edge_map[ey * ew + ex] == 255) {
                        has_edge = true;
                    }
                }
            }

            if (has_edge) {
                g_world.tilemap[ty][tx] = TILE_STONE_WALL;
                wall_count++;
            }
        }
    }
    ESP_LOGI(TAG, "Stone walls: %d tiles", wall_count);

    /* ---- 2. Build collision map for physics (reuse track_collision) ---- */
    track_collision_init();
    track_build_from_edges(edge_map, ew, eh);

    /* ---- 3. Process detected objects ---- */
    int obj_idx = 0;
    int portal_indices[10];
    int portal_count = 0;

    if (!detections || det_count <= 0) {
        ESP_LOGI(TAG, "No objects detected");
        g_world.object_count = 0;
        g_built = true;
        return;
    }

    for (int i = 0; i < det_count && obj_idx < 10; i++) {
        const detection_result_t *d = &detections[i];
        int coco_id = d->category;

        gameobj_type_t gt = coco_to_game_type(coco_id);
        if (gt == GAMEOBJ_FRUIT && coco_id != 47 && coco_id != 49 && coco_id != 46)
            continue;  /* skip non-fruit items if we don't handle them */

        /* Camera coordinates → game map coordinates */
        /* Camera: 800×640, game map: center 640×640 crop */
        int cam_cx = (d->box_camera[0] + d->box_camera[2]) / 2;
        int cam_cy = (d->box_camera[1] + d->box_camera[3]) / 2;
        int game_x = cam_cx - 80;   /* (800-640)/2 = 80 offset */
        int game_y = cam_cy;

        /* Clamp to game map */
        if (game_x < 0) game_x = 0;
        if (game_x >= 640) game_x = 639;
        if (game_y < 0) game_y = 0;
        if (game_y >= 640) game_y = 639;

        /* Skip objects that land on existing stone walls (inside wall) */
        int tx = game_x / 16;
        int ty = game_y / 16;
        if (tx >= 0 && tx < 40 && ty >= 0 && ty < 40) {
            if (g_world.tilemap[ty][tx] == TILE_STONE_WALL) {
                ESP_LOGI(TAG, "  %s at wall tile (%d,%d), skipping",
                         coco_name(coco_id), tx, ty);
                continue;
            }
        }

        game_object_t *obj = &g_world.objects[obj_idx];
        obj->type    = gt;
        obj->coco_id = coco_id;
        obj->pixel_x = game_x;
        obj->pixel_y = game_y;
        obj->radius  = coco_to_radius(coco_id);
        obj->alive   = true;
        obj->partner_id = -1;
        obj->cooldown   = 0;
        obj->bounce_mult = (gt == GAMEOBJ_SURFACE) ? coco_to_bounce(coco_id) : 1.0f;

        /* Book: mark surrounding tiles as book wall */
        if (coco_id == 73) {
            int bw = (d->box_camera[2] - d->box_camera[0]) / 16;
            int bh = (d->box_camera[3] - d->box_camera[1]) / 16;
            if (bw < 1) bw = 1;
            if (bw > 6) bw = 6;
            if (bh < 1) bh = 1;
            if (bh > 6) bh = 6;

            int bx0 = (game_x - bw*8) / 16;
            int by0 = (game_y - bh*8) / 16;
            int bx1 = bx0 + bw;
            int by1 = by0 + bh;
            if (bx0 < 0) bx0 = 0;
            if (by0 < 0) by0 = 0;
            if (bx1 >= 40) bx1 = 39;
            if (by1 >= 40) by1 = 39;

            int book_tiles = 0;
            for (int bty = by0; bty <= by1; bty++)
                for (int btx = bx0; btx <= bx1; btx++)
                    if (g_world.tilemap[bty][btx] == TILE_EMPTY) {
                        g_world.tilemap[bty][btx] = TILE_BOOK_WALL;
                        book_tiles++;
                    }
            ESP_LOGI(TAG, "  %s: book wall %d tiles (%d,%d)-(%d,%d)",
                     coco_name(coco_id), book_tiles, bx0, by0, bx1, by1);
            /* Book objects themselves are wall tiles, not interactive objects */
            obj_idx--;
            goto next_object;
        }

        /* Portal: record for pairing */
        if (gt == GAMEOBJ_PORTAL) {
            portal_indices[portal_count++] = obj_idx;
        }

        ESP_LOGI(TAG, "  [%d] %s type=%d at (%d,%d) r=%d",
                 obj_idx, coco_name(coco_id), gt, game_x, game_y, obj->radius);
        obj_idx++;

    next_object:;
    }

    /* ---- 4. Pair portals ---- */
    if (portal_count >= 2) {
        /* Pair the two closest portals */
        for (int i = 0; i < portal_count - 1; i += 2) {
            int a = portal_indices[i];
            int b = portal_indices[i + 1];
            g_world.objects[a].partner_id = b;
            g_world.objects[b].partner_id = a;
            ESP_LOGI(TAG, "Portal pair: [%d]↔[%d]", a, b);
        }
    } else if (portal_count == 1) {
        ESP_LOGI(TAG, "Only 1 portal detected (need 2)");
    }

    g_world.object_count = obj_idx;

    /* Reset game state */
    g_world.goal_reached = false;
    g_world.player_dead  = false;

    g_built = true;
    ESP_LOGI(TAG, "World built: %d wall tiles, %d objects",
             wall_count, obj_idx);
}

/* ========================================================================
 * Queries
 * ======================================================================== */

pixel_world_t *pixel_world_get(void)
{
    return &g_world;
}

bool pixel_world_is_built(void)
{
    return g_built;
}

void pixel_world_destroy(void)
{
    memset(&g_world, 0, sizeof(g_world));
    g_built = false;
}

tile_type_t pixel_world_get_tile(int tx, int ty)
{
    if (tx < 0 || tx >= 40 || ty < 0 || ty >= 40)
        return TILE_STONE_WALL;  /* out of bounds = wall */
    return (tile_type_t)g_world.tilemap[ty][tx];
}

bool pixel_world_is_solid(int tx, int ty)
{
    tile_type_t t = pixel_world_get_tile(tx, ty);
    return (t == TILE_STONE_WALL || t == TILE_BOOK_WALL);
}

void pixel_world_destroy_tile(int tx, int ty)
{
    if (tx < 0 || tx >= 40 || ty < 0 || ty >= 40) return;
    if (g_world.tilemap[ty][tx] == TILE_BOOK_WALL) {
        g_world.tilemap[ty][tx] = TILE_BROKEN;
        ESP_LOGI(TAG, "Book wall destroyed at (%d,%d)", tx, ty);
    }
}

game_object_t *pixel_world_find_object_at(int px, int py)
{
    for (int i = 0; i < g_world.object_count; i++) {
        game_object_t *obj = &g_world.objects[i];
        if (!obj->alive) continue;
        int dx = px - obj->pixel_x;
        int dy = py - obj->pixel_y;
        if (dx*dx + dy*dy <= obj->radius * obj->radius)
            return obj;
    }
    return NULL;
}

game_object_t *pixel_world_get_object_by_id(int id)
{
    if (id < 0 || id >= g_world.object_count) return NULL;
    return &g_world.objects[id];
}

const char *pixel_world_coco_name(int coco_id)
{
    return coco_name(coco_id);
}
