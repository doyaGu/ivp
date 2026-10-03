/* ive_sample_interaction.cxx -- Mouse drag + focus for legacy C++ IVP */

#include "ive_sample_interaction.hxx"
#include "ive_sample_common.hxx"

#include <ivp_real_object.hxx>

#include <SDL3/SDL.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace ive {

/* ── Ray helpers ──────────────────────────────────────────────────────── */

static bool ray_sphere_hit(const float ro[3], const float rd[3],
                           const float c[3], float r, float *t_hit)
{
    float ocx = ro[0] - c[0];
    float ocy = ro[1] - c[1];
    float ocz = ro[2] - c[2];
    float b = ocx * rd[0] + ocy * rd[1] + ocz * rd[2];
    float cterm = ocx * ocx + ocy * ocy + ocz * ocz - r * r;
    float disc = b * b - cterm;
    if (disc < 0.0f) return false;
    float s = std::sqrt(disc);
    float t0 = -b - s;
    float t1 = -b + s;
    float t = (t0 > 0.0f) ? t0 : t1;
    if (t <= 0.0f) return false;
    *t_hit = t;
    return true;
}

/* screen_ray() is now in ive_sample_common.cxx */

/* ── Object enumeration helpers ───────────────────────────────────────── */

static int env_object_count(IVP_Environment *env)
{
    /* Walk the root cluster to count objects.
     * The legacy API doesn't expose a direct count, so we use the
     * root cluster's object vector. As a practical workaround we
     * iterate via the static_object sentinel. For the samples we
     * store objects in the SceneObjects array and enumerate there.
     * However, for a generic approach we iterate from index 0
     * until we get NULL. */
    (void)env;
    return 0; /* placeholder - actual picking is done below via stored refs */
}

/* For picking we scan all objects through the environment's
 * internal structure. Since legacy IVP doesn't have a clean
 * iterator we use a brute-force approach: we collect dynamic
 * objects during dragger_update by walking the cluster. */

/* ── Dragger ──────────────────────────────────────────────────────────── */

static void dragger_clear(Dragger *d)
{
    d->active = false;
    d->target = NULL;
    d->has_target_point = false;
    d->depth = 0.0f;
    d->ramp = 0.0f;
}

void dragger_init(Dragger *d)
{
    std::memset(d, 0, sizeof(*d));
    d->pick_radius_scale = 1.15f;
    d->min_depth = 0.5f;
    d->stiffness = 22.0f;
    d->damping = 8.5f;
    d->max_force = 70.0f;
    d->ramp_rate = 4.0f;
    dragger_clear(d);
}

/* Collect all moveable objects from the environment's root cluster.
 * We access the IVP_Cluster -> objects. For legacy code the simplest
 * way is to iterate through the cluster tree. However the public API
 * doesn't expose this cleanly. Instead we rely on the user storing
 * object pointers. For the dragger we do a simplified pick: we walk
 * through all objects by traversing the cluster. */

/* Helper to collect dynamic objects for ray picking */
static void collect_dynamic_objects(IVP_Environment *env,
                                    IVP_Real_Object **out, int *count, int max_objs)
{
    /* Access the root cluster's child objects.
     * Since legacy IVP doesn't have a public object enumeration API,
     * we use the cluster hierarchy. The root cluster is obtained via
     * get_root_cluster(), then we iterate its objects. */
    IVP_Cluster *root = env->get_root_cluster();
    if (!root) { *count = 0; return; }

    /* IVP_Cluster contains a vector of IVP_Real_Objects (its children).
     * We access the objects member. */
    *count = 0;
    /* Walk all objects in the cluster. The IVP_Cluster inherits from
     * IVP_Object and contains objects. For simplicity, we rely on
     * the environment's global_object_listeners to have tracked creation.
     * As a workaround, we use client_data stored in the environment. */

    /* Actually, IVP_Cluster has a method to enumerate. But since the
     * headers aren't fully exposed for this, we use a different approach:
     * iterate the collision system's OV tree. For these simple samples,
     * the most reliable method is to keep our own array of objects.
     * The dragger will skip objects it can't find. */
    (void)root;
    (void)out;
    (void)max_objs;
}

void dragger_update(Dragger *d, const ivp_camera_t *cam,
                    IVP_Environment *env,
                    int win_w, int win_h, float frame_dt)
{
    if (!d || !cam || !env) return;

    float mx = 0.0f, my = 0.0f;
    SDL_MouseButtonFlags buttons = SDL_GetMouseState(&mx, &my);
    bool lmb = (buttons & SDL_BUTTON_MASK(SDL_BUTTON_LEFT)) != 0;

    if (!d->lmb_seeded) {
        d->prev_lmb_down = lmb;
        d->lmb_seeded = true;
    }

    float ray_o[3], ray_d[3];
    bool ray_ok = screen_ray(cam, win_w, win_h, mx, my, ray_o, ray_d);

    /* On LMB press: pick nearest dynamic object */
    if (lmb && !d->prev_lmb_down) {
        dragger_clear(d);
        if (ray_ok) {
            /* Pick from objects stored in environment->client_data.
             * Convention: samples store IVP_Real_Object** array as
             * env->client_data, with count in first slot trick.
             * Actually we use a simpler approach: we store a static
             * array that samples populate. See the sample pattern. */
            IVP_Real_Object **objs = NULL;
            int obj_count = 0;

            /* Check if env->client_data holds our pick list */
            if (env->client_data) {
                IVP_Sample_Pick_List *pl =
                    (IVP_Sample_Pick_List *)env->client_data;
                obj_count = pl->count;
                objs = pl->objs;
            }

            float best_t = 1e30f;
            IVP_Real_Object *best = NULL;

            for (int i = 0; i < obj_count; i++) {
                IVP_Real_Object *obj = objs[i];
                if (!obj) continue;
                IVP_Core *core = obj->get_core();
                if (!core) continue;
                if (core->physical_unmoveable) continue;

                float center[3];
                point_to_float3(core->get_position_PSI(), center);
                float radius = (float)core->upper_limit_radius;
                if (radius < 0.05f) radius = 0.05f;
                radius *= d->pick_radius_scale;

                float t_hit = 0.0f;
                if (ray_sphere_hit(ray_o, ray_d, center, radius, &t_hit)
                    && t_hit < best_t) {
                    best_t = t_hit;
                    best = obj;
                }
            }

            if (best) {
                d->active = true;
                d->target = best;
                d->depth = (best_t > d->min_depth) ? best_t : d->min_depth;
                d->ramp = 0.0f;
            }
        }
    }

    /* On LMB release: clear */
    if (!lmb && d->prev_lmb_down) {
        dragger_clear(d);
    }

    /* Apply spring-damper force while dragging */
    if (d->active && lmb && ray_ok && d->target) {
        d->target_ws[0] = ray_o[0] + ray_d[0] * d->depth;
        d->target_ws[1] = ray_o[1] + ray_d[1] * d->depth;
        d->target_ws[2] = ray_o[2] + ray_d[2] * d->depth;
        d->has_target_point = true;

        IVP_Core *core = d->target->get_core();
        const IVP_U_Point *pos = core->get_position_PSI();

        float ex = d->target_ws[0] - (float)pos->k[0];
        float ey = d->target_ws[1] - (float)pos->k[1];
        float ez = d->target_ws[2] - (float)pos->k[2];

        float fx = d->stiffness * ex - d->damping * (float)core->speed.k[0];
        float fy = d->stiffness * ey - d->damping * (float)core->speed.k[1];
        float fz = d->stiffness * ez - d->damping * (float)core->speed.k[2];

        d->ramp += frame_dt * d->ramp_rate;
        if (d->ramp > 1.0f) d->ramp = 1.0f;
        fx *= d->ramp; fy *= d->ramp; fz *= d->ramp;

        float fmag = std::sqrt(fx*fx + fy*fy + fz*fz);
        if (fmag > d->max_force && fmag > 1e-6f) {
            float s = d->max_force / fmag;
            fx *= s; fy *= s; fz *= s;
        }

        /* Apply as velocity change (impulse / mass) */
        IVP_U_Float_Point impulse;
        impulse.set(fx * frame_dt, fy * frame_dt, fz * frame_dt);
        d->target->async_add_speed_object_ws(&impulse);
    } else {
        d->has_target_point = false;
    }

    d->prev_lmb_down = lmb;
}

/* ── Focus ────────────────────────────────────────────────────────────── */

void focus_init(FocusController *f)
{
    std::memset(f, 0, sizeof(*f));
    f->follow_enabled = true;
    f->follow_lerp = 7.0f;
    f->vertical_offset = 0.0f;
}

static IVP_Real_Object *find_next_in_list(IVP_Environment *env,
                                          IVP_Real_Object *current,
                                          int step)
{
    if (!env->client_data) return NULL;
    IVP_Sample_Pick_List *pl = (IVP_Sample_Pick_List *)env->client_data;
    int count = pl->count;
    IVP_Real_Object **objs = pl->objs;

    if (count <= 0) return NULL;

    int cur_idx = -1;
    for (int i = 0; i < count; i++) {
        if (objs[i] == current) { cur_idx = i; break; }
    }

    /* Find next dynamic object */
    int start = (cur_idx >= 0) ? cur_idx : 0;
    for (int n = 1; n <= count; n++) {
        int idx = (start + step * n) % count;
        if (idx < 0) idx += count;
        IVP_Real_Object *obj = objs[idx];
        if (!obj) continue;
        IVP_Core *core = obj->get_core();
        if (core && !core->physical_unmoveable) return obj;
    }
    return NULL;
}

void focus_update(FocusController *f, ivp_camera_t *cam,
                  IVP_Environment *env, float dt)
{
    if (!f || !cam || !env) return;

    const bool *keys = SDL_GetKeyboardState(NULL);
    bool tab = keys[SDL_SCANCODE_TAB];
    bool c_key = keys[SDL_SCANCODE_C];
    bool shift = keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT];

    if (tab && !f->prev_tab) {
        int step = shift ? -1 : 1;
        IVP_Real_Object *next = find_next_in_list(env, f->focused, step);
        if (next) f->focused = next;
    }

    if (c_key && !f->prev_c) {
        f->follow_enabled = !f->follow_enabled;
    }

    if (f->focused && f->follow_enabled) {
        IVP_Core *core = f->focused->get_core();
        if (core) {
            const IVP_U_Point *pos = core->get_position_PSI();
            float target[3] = {
                (float)pos->k[0],
                (float)pos->k[1] + f->vertical_offset,
                (float)pos->k[2]
            };
            float alpha = dt * f->follow_lerp;
            if (alpha > 1.0f) alpha = 1.0f;
            cam->target[0] += (target[0] - cam->target[0]) * alpha;
            cam->target[1] += (target[1] - cam->target[1]) * alpha;
            cam->target[2] += (target[2] - cam->target[2]) * alpha;
        }
    }

    f->prev_tab = tab;
    f->prev_c = c_key;
}

/* ── Overlays ─────────────────────────────────────────────────────────── */

void draw_interaction_overlays(ivp_renderer_t *r, const Dragger *d,
                               const FocusController *f)
{
    const float drag_col[3] = {1.0f, 0.8f, 0.2f};
    const float focus_col[3] = {0.2f, 0.8f, 1.0f};

    if (d && d->active && d->target && d->has_target_point) {
        IVP_Core *core = d->target->get_core();
        float from[3];
        point_to_float3(core->get_position_PSI(), from);
        ivp_draw_arrow(r, from, d->target_ws, drag_col, 0.25f);
    }

    if (f && f->focused) {
        IVP_Core *core = f->focused->get_core();
        float pos[3];
        point_to_float3(core->get_position_PSI(), pos);
        float hs[3] = {0.6f, 0.6f, 0.6f};
        ivp_draw_wire_box(r, pos, hs, NULL, focus_col);
    }
}

void draw_interaction_hud(ivp_renderer_t *r, float x, float y)
{
    const float pri[3] = {0.6f, 0.6f, 0.6f};
    const float sec[3] = {0.5f, 0.5f, 0.5f};
    ivp_draw_text_2d(r, x, y,
        "WASD+QE:pan  RMB:orbit  Scroll:zoom", pri);
    ivp_draw_text_2d(r, x, y + 18.0f,
        "LMB:drag  Tab:focus  C:follow", sec);
}

} /* namespace ive */
