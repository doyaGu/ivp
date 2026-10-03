/* ive_gizmo.cxx -- Gizmo implementation for IVP samples */

#include "ive_gizmo.hxx"
#include <ivp_cache_object.hxx>

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_UINT_DRAW_INDEX
#include "nuklear.h"

#include <ivp_core.hxx>

#include <SDL3/SDL.h>

#include <cmath>
#include <cstring>
#include <cstdio>

namespace ive {

/* ── Color constants ─────────────────────────────────────────────────── */

namespace gizmo_color {
    const float axis_x[3]      = {1.0f, 0.2f, 0.2f};
    const float axis_y[3]      = {0.2f, 1.0f, 0.2f};
    const float axis_z[3]      = {0.3f, 0.3f, 1.0f};
    const float axis_hover[3]  = {1.0f, 1.0f, 0.3f};
    const float axis_active[3] = {1.0f, 1.0f, 1.0f};
} /* namespace gizmo_color */

/* ── Helpers ─────────────────────────────────────────────────────────── */

static const float axis_dirs[3][3] = {
    {1,0,0}, {0,1,0}, {0,0,1}
};

static const float *axis_color(GizmoAxis a)
{
    switch (a) {
    case AXIS_X: return gizmo_color::axis_x;
    case AXIS_Y: return gizmo_color::axis_y;
    case AXIS_Z: return gizmo_color::axis_z;
    default:     return gizmo_color::axis_x;
    }
}

static const char *axis_label(GizmoAxis a)
{
    switch (a) {
    case AXIS_X: return "X";
    case AXIS_Y: return "Y";
    case AXIS_Z: return "Z";
    default:     return "";
    }
}

static float compute_arm_length(const ivp_camera_t *cam, const float pos[3])
{
    float dx = cam->eye[0] - pos[0];
    float dy = cam->eye[1] - pos[1];
    float dz = cam->eye[2] - pos[2];
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    return dist * 0.12f;
}

/* Point-to-line-segment distance in 2D */
static float point_line_dist_2d(float px, float py,
                                float ax, float ay, float bx, float by)
{
    float abx = bx - ax, aby = by - ay;
    float apx = px - ax, apy = py - ay;
    float ab2 = abx*abx + aby*aby;
    if (ab2 < 1e-8f) {
        return std::sqrt(apx*apx + apy*apy);
    }
    float t = (apx*abx + apy*aby) / ab2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float cx = ax + abx*t - px;
    float cy = ay + aby*t - py;
    return std::sqrt(cx*cx + cy*cy);
}

/* Get object world position as float[3] */
static void get_obj_pos(IVP_Real_Object *obj, float pos[3])
{
    IVP_Core *core = obj->get_core();
    const IVP_U_Point *p = core->get_position_PSI();
    pos[0] = (float)p->k[0];
    pos[1] = (float)p->k[1];
    pos[2] = (float)p->k[2];
}

/* Closest approach parameter between ray (ro+t*rd) and line (lo+s*ld).
 * Returns parameter s along the line axis. */
static float ray_line_param(const float ro[3], const float rd[3],
                            const float lo[3], const float ld[3])
{
    float dxo[3] = {ro[0]-lo[0], ro[1]-lo[1], ro[2]-lo[2]};
    float dd = rd[0]*rd[0] + rd[1]*rd[1] + rd[2]*rd[2];
    float de = rd[0]*ld[0] + rd[1]*ld[1] + rd[2]*ld[2];
    float ee = ld[0]*ld[0] + ld[1]*ld[1] + ld[2]*ld[2];
    float fd = dxo[0]*rd[0] + dxo[1]*rd[1] + dxo[2]*rd[2];
    float fe = dxo[0]*ld[0] + dxo[1]*ld[1] + dxo[2]*ld[2];
    float denom = dd*ee - de*de;
    if (std::fabs(denom) < 1e-10f) return 0.0f;
    return (de*fd - dd*fe) / denom;
}

/* Angle of a point projected onto a plane defined by an axis, relative
 * to the object center. Returns angle in radians. */
static float projected_angle(const float ro[3], const float rd[3],
                             const float center[3], int axis_idx)
{
    /* Intersect ray with the plane passing through center, normal = axis */
    const float *n = axis_dirs[axis_idx];
    float denom = n[0]*rd[0] + n[1]*rd[1] + n[2]*rd[2];
    if (std::fabs(denom) < 1e-8f) return 0.0f;
    float d[3] = {center[0]-ro[0], center[1]-ro[1], center[2]-ro[2]};
    float t = (d[0]*n[0] + d[1]*n[1] + d[2]*n[2]) / denom;
    float hit[3] = {ro[0]+rd[0]*t - center[0],
                    ro[1]+rd[1]*t - center[1],
                    ro[2]+rd[2]*t - center[2]};
    /* Get two basis vectors in the plane */
    int u_idx = (axis_idx + 1) % 3;
    int v_idx = (axis_idx + 2) % 3;
    return std::atan2(hit[v_idx], hit[u_idx]);
}

/* ── API ─────────────────────────────────────────────────────────────── */

void gizmo_init(Gizmo *g)
{
    std::memset(g, 0, sizeof(*g));
    g->mode = GIZMO_NONE;
    g->hover_axis = AXIS_NONE;
    g->active_axis = AXIS_NONE;
    g->screen_size = 80.0f;
    g->pick_threshold = 12.0f;
}

void gizmo_set_mode(Gizmo *g, GizmoMode mode)
{
    g->mode = mode;
    g->dragging = false;
    g->hover_axis = AXIS_NONE;
    g->active_axis = AXIS_NONE;
}

bool gizmo_update(Gizmo *g, IVP_Real_Object *target,
                  const ivp_camera_t *cam,
                  ivp_renderer_t *r, int win_w, int win_h)
{
    if (!g || g->mode == GIZMO_NONE || !target || !cam || !r)
        return false;

    float mx = 0.0f, my = 0.0f;
    SDL_MouseButtonFlags buttons = SDL_GetMouseState(&mx, &my);
    bool lmb = (buttons & SDL_BUTTON_MASK(SDL_BUTTON_LEFT)) != 0;

    if (!g->lmb_seeded) {
        g->prev_lmb = lmb;
        g->lmb_seeded = true;
    }

    bool lmb_pressed  = lmb && !g->prev_lmb;
    bool lmb_released = !lmb && g->prev_lmb;
    g->prev_lmb = lmb;

    float obj_pos[3];
    get_obj_pos(target, obj_pos);
    float arm = compute_arm_length(cam, obj_pos);

    /* ── Hover detection (screen-space pick) ─────────────────────────── */
    if (!g->dragging) {
        g->hover_axis = AXIS_NONE;
        float best_dist = g->pick_threshold;

        for (int i = 0; i < 3; i++) {
            float tip[3] = {
                obj_pos[0] + axis_dirs[i][0] * arm,
                obj_pos[1] + axis_dirs[i][1] * arm,
                obj_pos[2] + axis_dirs[i][2] * arm
            };
            float s_origin[2], s_tip[2];
            if (!ivp_renderer_project(r, obj_pos, s_origin)) continue;
            if (!ivp_renderer_project(r, tip, s_tip)) continue;

            float dist = point_line_dist_2d(mx, my,
                                            s_origin[0], s_origin[1],
                                            s_tip[0], s_tip[1]);
            if (dist < best_dist) {
                best_dist = dist;
                g->hover_axis = (GizmoAxis)(AXIS_X + i);
            }
        }
    }

    /* ── Drag start ──────────────────────────────────────────────────── */
    if (lmb_pressed && g->hover_axis != AXIS_NONE) {
        g->active_axis = g->hover_axis;
        g->dragging = true;
        g->drag_origin[0] = obj_pos[0];
        g->drag_origin[1] = obj_pos[1];
        g->drag_origin[2] = obj_pos[2];
        g->drag_axis_param = 0.0f;

        if (g->mode == GIZMO_ROTATE) {
            float ro[3], rd[3];
            if (screen_ray(cam, win_w, win_h, mx, my, ro, rd)) {
                g->drag_start_angle = projected_angle(ro, rd, obj_pos,
                                                      g->active_axis - AXIS_X);
            }
        }
    }

    /* ── Drag update ─────────────────────────────────────────────────── */
    if (g->dragging && lmb) {
        float ro[3], rd[3];
        if (screen_ray(cam, win_w, win_h, mx, my, ro, rd)) {
            int ai = g->active_axis - AXIS_X;

            if (g->mode == GIZMO_TRANSLATE) {
                float s = ray_line_param(ro, rd, g->drag_origin, axis_dirs[ai]);
                g->drag_axis_param = s;

                IVP_U_Point new_pos;
                new_pos.set(g->drag_origin[0] + axis_dirs[ai][0] * s,
                            g->drag_origin[1] + axis_dirs[ai][1] * s,
                            g->drag_origin[2] + axis_dirs[ai][2] * s);

                /* Read current rotation */
                IVP_Cache_Object *cache = target->get_cache_object();
                IVP_U_Quat q = cache->q_world_f_object;
                cache->remove_reference();

                target->beam_object_to_new_position(&q, &new_pos, IVP_FALSE);

                /* Zero velocity so object doesn't drift */
                IVP_Core *core = target->get_core();
                core->speed.set(0.0, 0.0, 0.0);
                core->rot_speed.set(0.0, 0.0, 0.0);
            }
            else if (g->mode == GIZMO_ROTATE) {
                float angle = projected_angle(ro, rd, obj_pos, ai);
                float delta = angle - g->drag_start_angle;
                g->drag_axis_param = delta;

                /* Build quaternion for rotation around the axis */
                double half = 0.5 * (double)delta;
                double s_half = std::sin(half);
                double c_half = std::cos(half);
                IVP_U_Quat dq;
                dq.x = axis_dirs[ai][0] * s_half;
                dq.y = axis_dirs[ai][1] * s_half;
                dq.z = axis_dirs[ai][2] * s_half;
                dq.w = c_half;

                /* Read initial rotation at drag start and compose */
                IVP_Cache_Object *cache = target->get_cache_object();
                IVP_U_Quat base_q = cache->q_world_f_object;
                cache->remove_reference();

                /* Compose: result = dq * base (apply delta in world frame) */
                IVP_U_Quat new_q;
                new_q.x = dq.w*base_q.x + dq.x*base_q.w + dq.y*base_q.z - dq.z*base_q.y;
                new_q.y = dq.w*base_q.y - dq.x*base_q.z + dq.y*base_q.w + dq.z*base_q.x;
                new_q.z = dq.w*base_q.z + dq.x*base_q.y - dq.y*base_q.x + dq.z*base_q.w;
                new_q.w = dq.w*base_q.w - dq.x*base_q.x - dq.y*base_q.y - dq.z*base_q.z;

                IVP_U_Point pos;
                pos.set(obj_pos[0], obj_pos[1], obj_pos[2]);
                target->beam_object_to_new_position(&new_q, &pos, IVP_FALSE);

                IVP_Core *core = target->get_core();
                core->speed.set(0.0, 0.0, 0.0);
                core->rot_speed.set(0.0, 0.0, 0.0);
            }
        }
    }

    /* ── Drag end ────────────────────────────────────────────────────── */
    if (lmb_released && g->dragging) {
        g->dragging = false;
        g->active_axis = AXIS_NONE;
        g->drag_axis_param = 0.0f;
    }

    return g->dragging || g->hover_axis != AXIS_NONE;
}

/* ── 3D Drawing ──────────────────────────────────────────────────────── */

#define GIZMO_CIRCLE_SEGS 48
#define PI_F 3.14159265358979323846f

static void draw_translate_handles(const Gizmo *g, const float pos[3],
                                   float arm, ivp_renderer_t *r)
{
    for (int i = 0; i < 3; i++) {
        GizmoAxis a = (GizmoAxis)(AXIS_X + i);
        float tip[3] = {
            pos[0] + axis_dirs[i][0] * arm,
            pos[1] + axis_dirs[i][1] * arm,
            pos[2] + axis_dirs[i][2] * arm
        };

        const float *col;
        if (a == g->active_axis)
            col = gizmo_color::axis_active;
        else if (a == g->hover_axis)
            col = gizmo_color::axis_hover;
        else
            col = axis_color(a);

        ivp_draw_arrow(r, pos, tip, col, arm * 0.15f);
    }

    /* During translate drag: draw measurement line with tick marks */
    if (g->dragging && g->active_axis != AXIS_NONE) {
        int ai = g->active_axis - AXIS_X;
        const float *col = gizmo_color::axis_active;
        const float dim[3] = {0.45f, 0.45f, 0.45f};
        float disp = g->drag_axis_param;

        /* Thick line from drag origin to current position */
        float cur[3] = {
            g->drag_origin[0] + axis_dirs[ai][0] * disp,
            g->drag_origin[1] + axis_dirs[ai][1] * disp,
            g->drag_origin[2] + axis_dirs[ai][2] * disp
        };
        ivp_draw_thick_line(r, g->drag_origin, cur, col, 2.0f);

        /* Tick marks at each integer unit along the axis */
        float abs_disp = std::fabs(disp);
        float sign = (disp >= 0.0f) ? 1.0f : -1.0f;
        int tick_count = (int)abs_disp;
        if (tick_count > 50) tick_count = 50;
        float tick_size = arm * 0.08f;

        /* Choose a perpendicular direction for ticks */
        int perp_idx = (ai + 1) % 3;

        for (int t = 1; t <= tick_count; t++) {
            float tick_pos[3] = {
                g->drag_origin[0] + axis_dirs[ai][0] * sign * (float)t,
                g->drag_origin[1] + axis_dirs[ai][1] * sign * (float)t,
                g->drag_origin[2] + axis_dirs[ai][2] * sign * (float)t
            };
            float tick_a[3], tick_b[3];
            std::memcpy(tick_a, tick_pos, sizeof(float)*3);
            std::memcpy(tick_b, tick_pos, sizeof(float)*3);
            tick_a[perp_idx] -= tick_size;
            tick_b[perp_idx] += tick_size;
            ivp_draw_line(r, tick_a, tick_b, dim);
        }

        /* Origin cross mark */
        for (int p = 0; p < 3; p++) {
            if (p == ai) continue;
            float ca[3], cb[3];
            std::memcpy(ca, g->drag_origin, sizeof(float)*3);
            std::memcpy(cb, g->drag_origin, sizeof(float)*3);
            ca[p] -= tick_size;
            cb[p] += tick_size;
            ivp_draw_line(r, ca, cb, dim);
        }
    }
}

static void draw_rotate_handles(const Gizmo *g, const float pos[3],
                                float arm, ivp_renderer_t *r)
{
    float pi2 = PI_F * 2.0f;

    for (int i = 0; i < 3; i++) {
        GizmoAxis a = (GizmoAxis)(AXIS_X + i);
        int u_idx = (i + 1) % 3;
        int v_idx = (i + 2) % 3;

        const float *col;
        if (a == g->active_axis)
            col = gizmo_color::axis_active;
        else if (a == g->hover_axis)
            col = gizmo_color::axis_hover;
        else
            col = axis_color(a);

        for (int s = 0; s < GIZMO_CIRCLE_SEGS; s++) {
            float a0 = (float)s / (float)GIZMO_CIRCLE_SEGS * pi2;
            float a1 = (float)(s + 1) / (float)GIZMO_CIRCLE_SEGS * pi2;
            float p0[3], p1[3];
            std::memcpy(p0, pos, sizeof(float)*3);
            std::memcpy(p1, pos, sizeof(float)*3);
            p0[u_idx] += std::cos(a0) * arm;
            p0[v_idx] += std::sin(a0) * arm;
            p1[u_idx] += std::cos(a1) * arm;
            p1[v_idx] += std::sin(a1) * arm;
            ivp_draw_line(r, p0, p1, col);
        }
    }

    /* During rotate drag: draw arc wedge showing swept angle */
    if (g->dragging && g->active_axis != AXIS_NONE) {
        int ai = g->active_axis - AXIS_X;
        int u_idx = (ai + 1) % 3;
        int v_idx = (ai + 2) % 3;

        float start = g->drag_start_angle;
        float delta = g->drag_axis_param;
        int arc_segs = (int)(std::fabs(delta) / (PI_F * 2.0f) * (float)GIZMO_CIRCLE_SEGS);
        if (arc_segs < 2) arc_segs = 2;
        if (arc_segs > GIZMO_CIRCLE_SEGS * 2) arc_segs = GIZMO_CIRCLE_SEGS * 2;

        const float *col = gizmo_color::axis_active;
        const float arc_col[3] = {
            col[0] * 0.7f, col[1] * 0.7f, col[2] * 0.7f
        };
        float inner_r = arm * 0.35f;

        /* Draw filled wedge as radial lines from center to inner arc */
        for (int s = 0; s <= arc_segs; s++) {
            float t = (float)s / (float)arc_segs;
            float angle = start + delta * t;
            float p_outer[3], p_inner[3];
            std::memcpy(p_outer, pos, sizeof(float)*3);
            std::memcpy(p_inner, pos, sizeof(float)*3);
            p_outer[u_idx] += std::cos(angle) * arm;
            p_outer[v_idx] += std::sin(angle) * arm;
            p_inner[u_idx] += std::cos(angle) * inner_r;
            p_inner[v_idx] += std::sin(angle) * inner_r;
            ivp_draw_line(r, p_inner, p_outer, arc_col);
        }

        /* Bright arc at the outer edge of the wedge */
        for (int s = 0; s < arc_segs; s++) {
            float t0 = (float)s / (float)arc_segs;
            float t1 = (float)(s + 1) / (float)arc_segs;
            float ang0 = start + delta * t0;
            float ang1 = start + delta * t1;
            float p0[3], p1[3];
            std::memcpy(p0, pos, sizeof(float)*3);
            std::memcpy(p1, pos, sizeof(float)*3);
            p0[u_idx] += std::cos(ang0) * arm;
            p0[v_idx] += std::sin(ang0) * arm;
            p1[u_idx] += std::cos(ang1) * arm;
            p1[v_idx] += std::sin(ang1) * arm;
            ivp_draw_thick_line(r, p0, p1, col, 2.0f);
        }

        /* Start-angle radial indicator line */
        {
            float sa[3];
            std::memcpy(sa, pos, sizeof(float)*3);
            sa[u_idx] += std::cos(start) * arm * 1.1f;
            sa[v_idx] += std::sin(start) * arm * 1.1f;
            const float ind[3] = {0.5f, 0.5f, 0.5f};
            ivp_draw_line(r, pos, sa, ind);
        }
    }
}

void gizmo_draw_3d(const Gizmo *g, IVP_Real_Object *target,
                   const ivp_camera_t *cam, ivp_renderer_t *r)
{
    if (!g || g->mode == GIZMO_NONE || !target || !cam || !r) return;

    float pos[3];
    get_obj_pos(target, pos);
    float arm = compute_arm_length(cam, pos);

    if (g->mode == GIZMO_TRANSLATE)
        draw_translate_handles(g, pos, arm, r);
    else if (g->mode == GIZMO_ROTATE)
        draw_rotate_handles(g, pos, arm, r);
}

/* ── 2D Overlay (Nuklear) ────────────────────────────────────────────── */

static struct nk_color nk_axis_color(GizmoAxis a, const Gizmo *g)
{
    if (a == g->active_axis) return nk_rgb(255, 255, 255);
    if (a == g->hover_axis)  return nk_rgb(255, 255, 80);
    switch (a) {
    case AXIS_X: return nk_rgb(255, 70, 70);
    case AXIS_Y: return nk_rgb(70, 255, 70);
    case AXIS_Z: return nk_rgb(90, 90, 255);
    default:     return nk_rgb(180, 180, 180);
    }
}

/* Estimate world-space arm length from projection so labels match 3D */
static float estimate_arm_from_proj(ivp_renderer_t *r, const float pos[3])
{
    float p1[3] = {pos[0]+1, pos[1], pos[2]};
    float s0[2], s1[2];
    if (ivp_renderer_project(r, pos, s0) && ivp_renderer_project(r, p1, s1)) {
        float px_per_unit = std::sqrt((s1[0]-s0[0])*(s1[0]-s0[0]) +
                                      (s1[1]-s0[1])*(s1[1]-s0[1]));
        if (px_per_unit > 0.01f)
            return 80.0f / px_per_unit;
    }
    return 2.0f;
}

void gizmo_draw_overlay(const Gizmo *g, IVP_Real_Object *target,
                        struct nk_context *nk,
                        ivp_renderer_t *r, int win_w, int win_h)
{
    if (!g || g->mode == GIZMO_NONE || !target || !nk || !r) return;

    float pos[3];
    get_obj_pos(target, pos);
    float arm = estimate_arm_from_proj(r, pos);

    /* Fullscreen transparent overlay */
    nk_flags flags = NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_NO_INPUT |
                     NK_WINDOW_BACKGROUND;
    if (!nk_begin(nk, "__gizmo_overlay",
                  nk_rect(0, 0, (float)win_w, (float)win_h), flags)) {
        nk_end(nk);
        return;
    }

    struct nk_command_buffer *canvas = nk_window_get_canvas(nk);

    /* ── Axis tip badges: colored circle + letter ────────────────────── */
    for (int i = 0; i < 3; i++) {
        float tip[3] = {
            pos[0] + axis_dirs[i][0] * arm * 1.15f,
            pos[1] + axis_dirs[i][1] * arm * 1.15f,
            pos[2] + axis_dirs[i][2] * arm * 1.15f
        };
        float scr[2];
        if (!ivp_renderer_project(r, tip, scr)) continue;

        GizmoAxis a = (GizmoAxis)(AXIS_X + i);
        struct nk_color col = nk_axis_color(a, g);
        struct nk_color bg = nk_rgba(col.r/4, col.g/4, col.b/4, 200);

        /* Background circle */
        float badge_r = 10.0f;
        struct nk_rect badge = nk_rect(scr[0] - badge_r, scr[1] - badge_r,
                                       badge_r * 2, badge_r * 2);
        nk_fill_circle(canvas, badge, bg);
        nk_stroke_circle(canvas, badge, 1.5f, col);

        /* Letter centered in badge */
        struct nk_rect lbl = nk_rect(scr[0] - 5, scr[1] - 9, 14, 18);
        nk_draw_text(canvas, lbl, axis_label(a), 1,
                     nk->style.font, nk_rgba(0,0,0,0), col);
    }

    /* ── Drag readout: graphical gauge + value ───────────────────────── */
    if (g->dragging && g->active_axis != AXIS_NONE) {
        float smx = 0.0f, smy = 0.0f;
        SDL_GetMouseState(&smx, &smy);

        struct nk_color acol = nk_axis_color(g->active_axis, g);
        struct nk_color bg_dark = nk_rgba(20, 24, 30, 210);
        struct nk_color bg_fill = nk_rgba(acol.r, acol.g, acol.b, 80);

        /* Panel position: offset from cursor */
        float px = smx + 22.0f;
        float py = smy - 40.0f;
        if (px + 160 > (float)win_w) px = smx - 182.0f;
        if (py < 4) py = 4.0f;

        if (g->mode == GIZMO_TRANSLATE) {
            /* ── Translation bar gauge ───────────────────────────── */
            float panel_w = 156.0f, panel_h = 44.0f;
            struct nk_rect panel = nk_rect(px, py, panel_w, panel_h);
            nk_fill_rect(canvas, panel, 4.0f, bg_dark);
            nk_stroke_rect(canvas, panel, 4.0f, 1.0f, nk_rgba(60,60,60,200));

            /* Axis label */
            struct nk_rect lbl = nk_rect(px + 6, py + 2, 20, 18);
            nk_draw_text(canvas, lbl, axis_label(g->active_axis), 1,
                         nk->style.font, nk_rgba(0,0,0,0), acol);

            /* Value text */
            char buf[32];
            std::sprintf(buf, "%.2f", (double)g->drag_axis_param);
            int len = 0; while (buf[len]) len++;
            struct nk_rect vtxt = nk_rect(px + 24, py + 2, 120, 18);
            nk_draw_text(canvas, vtxt, buf, len,
                         nk->style.font, nk_rgba(0,0,0,0), nk_rgb(220,220,220));

            /* Horizontal bar gauge */
            float bar_x = px + 6.0f;
            float bar_y = py + 22.0f;
            float bar_w = panel_w - 12.0f;
            float bar_h = 14.0f;
            struct nk_rect bar_bg = nk_rect(bar_x, bar_y, bar_w, bar_h);
            nk_fill_rect(canvas, bar_bg, 2.0f, nk_rgba(40, 44, 50, 255));

            /* Fill: map displacement to bar. Center = zero, scale +-10 units to full bar */
            float range = 10.0f;
            float center_x = bar_x + bar_w * 0.5f;
            float fill_frac = g->drag_axis_param / range;
            if (fill_frac > 1.0f) fill_frac = 1.0f;
            if (fill_frac < -1.0f) fill_frac = -1.0f;
            float fill_px = fill_frac * (bar_w * 0.5f);

            if (fill_px >= 0) {
                struct nk_rect fill = nk_rect(center_x, bar_y + 1,
                                              fill_px, bar_h - 2);
                nk_fill_rect(canvas, fill, 1.0f, bg_fill);
            } else {
                struct nk_rect fill = nk_rect(center_x + fill_px, bar_y + 1,
                                              -fill_px, bar_h - 2);
                nk_fill_rect(canvas, fill, 1.0f, bg_fill);
            }

            /* Center line (zero mark) */
            nk_stroke_line(canvas, center_x, bar_y, center_x, bar_y + bar_h,
                           1.0f, nk_rgba(120, 120, 120, 200));

            /* Tick marks at 25% and 75% */
            for (int t = 1; t <= 3; t++) {
                if (t == 2) continue; /* skip center, already drawn */
                float tx = bar_x + bar_w * (float)t / 4.0f;
                nk_stroke_line(canvas, tx, bar_y + bar_h - 3,
                               tx, bar_y + bar_h,
                               1.0f, nk_rgba(80, 80, 80, 180));
            }
        }
        else if (g->mode == GIZMO_ROTATE) {
            /* ── Rotation arc gauge ──────────────────────────────── */
            float panel_w = 100.0f, panel_h = 100.0f;
            struct nk_rect panel = nk_rect(px, py, panel_w, panel_h);
            nk_fill_rect(canvas, panel, 4.0f, bg_dark);
            nk_stroke_rect(canvas, panel, 4.0f, 1.0f, nk_rgba(60,60,60,200));

            /* Arc gauge in center of panel */
            float cx = px + panel_w * 0.5f;
            float cy = py + panel_h * 0.5f + 2.0f;
            float gauge_r = 30.0f;

            /* Background circle (track) */
            struct nk_rect circ = nk_rect(cx - gauge_r, cy - gauge_r,
                                          gauge_r * 2, gauge_r * 2);
            nk_stroke_circle(canvas, circ, 1.5f, nk_rgba(50, 54, 60, 255));

            /* Filled arc showing angle -- Nuklear uses screen angles
             * (CW from 3 o'clock). Convert from our angle. */
            float delta_deg = g->drag_axis_param * 180.0f / PI_F;
            float start_nk = -PI_F * 0.5f; /* 12 o'clock */
            float end_nk = start_nk + g->drag_axis_param;

            /* Normalize: ensure a_min < a_max for nk_fill_arc */
            float a_min, a_max;
            if (end_nk >= start_nk) {
                a_min = start_nk; a_max = end_nk;
            } else {
                a_min = end_nk; a_max = start_nk;
            }
            nk_fill_arc(canvas, cx, cy, gauge_r, a_min, a_max, bg_fill);
            nk_stroke_arc(canvas, cx, cy, gauge_r, a_min, a_max, 2.0f, acol);

            /* Start radial line */
            float sx = cx + std::cos(start_nk) * gauge_r;
            float sy = cy + std::sin(start_nk) * gauge_r;
            nk_stroke_line(canvas, cx, cy, sx, sy, 1.0f,
                           nk_rgba(120, 120, 120, 180));

            /* Current radial line */
            float ex = cx + std::cos(end_nk) * gauge_r;
            float ey = cy + std::sin(end_nk) * gauge_r;
            nk_stroke_line(canvas, cx, cy, ex, ey, 1.5f, acol);

            /* Dot at current angle tip */
            struct nk_rect dot = nk_rect(ex - 3, ey - 3, 6, 6);
            nk_fill_circle(canvas, dot, acol);

            /* Axis label at top-left */
            struct nk_rect lbl = nk_rect(px + 5, py + 2, 20, 18);
            nk_draw_text(canvas, lbl, axis_label(g->active_axis), 1,
                         nk->style.font, nk_rgba(0,0,0,0), acol);

            /* Degree text below gauge */
            char buf[32];
            std::sprintf(buf, "%.1f", (double)delta_deg);
            int len = 0; while (buf[len]) len++;
            /* degree symbol (just append a small "o" since bitmap fonts vary) */
            buf[len] = ' '; buf[len+1] = 'd'; buf[len+2] = 'e';
            buf[len+3] = 'g'; buf[len+4] = '\0';
            len += 4;
            struct nk_rect dtxt = nk_rect(px + 5, py + panel_h - 20,
                                          panel_w - 10, 18);
            nk_draw_text(canvas, dtxt, buf, len,
                         nk->style.font, nk_rgba(0,0,0,0), nk_rgb(220,220,220));
        }
    }

    nk_end(nk);
}

bool gizmo_is_active(const Gizmo *g)
{
    return g && g->dragging;
}

} /* namespace ive */
