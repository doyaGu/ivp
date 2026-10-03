/* ive_sample_interaction.hxx -- Mouse drag + focus for legacy C++ IVP objects */
#ifndef IVE_SAMPLE_INTERACTION_HXX
#define IVE_SAMPLE_INTERACTION_HXX

#include "render.h"

#include <ivp_physics.hxx>
#include <ivp_core.hxx>

/* ── Pick list layout (shared between samples and interaction code) ──── */

struct IVP_Sample_Pick_List {
    int count;
    IVP_Real_Object *objs[64];
};

namespace ive {

/* ── Dragger (LMB spring-damper drag) ─────────────────────────────────── */

struct Dragger {
    bool active;
    bool has_target_point;
    bool prev_lmb_down;
    bool lmb_seeded;

    IVP_Real_Object *target;
    float target_ws[3];
    float depth;
    float ramp;

    /* spring-damper settings */
    float pick_radius_scale;
    float min_depth;
    float stiffness;
    float damping;
    float max_force;
    float ramp_rate;
};

void dragger_init(Dragger *d);
void dragger_update(Dragger *d, const ivp_camera_t *cam,
                    IVP_Environment *env,
                    int win_w, int win_h, float frame_dt);

/* ── Focus controller (Tab cycle + camera follow) ─────────────────────── */

struct FocusController {
    bool follow_enabled;
    bool prev_tab;
    bool prev_c;

    IVP_Real_Object *focused;
    float follow_lerp;
    float vertical_offset;
};

void focus_init(FocusController *f);
void focus_update(FocusController *f, ivp_camera_t *cam,
                  IVP_Environment *env, float dt);

/* ── Overlay drawing ──────────────────────────────────────────────────── */

void draw_interaction_overlays(ivp_renderer_t *r, const Dragger *d,
                               const FocusController *f);
void draw_interaction_hud(ivp_renderer_t *r, float x, float y);

} /* namespace ive */

#endif /* IVE_SAMPLE_INTERACTION_HXX */
