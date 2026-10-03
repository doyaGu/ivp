/* ive_quickstart_maxforce.cxx -- Constraints with a maximum force
 *
 * Based on IVP Manual section 5.17: Constraints with a maximum force.
 * Demonstrates: IVP_CFE_BREAK and IVP_CFE_CLIP maximpulse_type values.
 *
 * Two pairs of objects are connected by ball-socket constraints:
 *
 *   Left pair  (green/blue):  IVP_CFE_BREAK  -- constraint BREAKS when
 *                             impulse exceeds the threshold. The link
 *                             disappears permanently.
 *
 *   Right pair (orange/red):  IVP_CFE_CLIP   -- constraint gives way
 *                             temporarily but tries to recover.
 *
 * Press SPACE to throw the attached objects upward so they exceed the
 * maximpulse threshold. Observe which constraint breaks and which clips.
 *
 * Manual notes:
 *   - All six maximpulse values must currently be set to the SAME value.
 *   - All six maximpulse_type values must currently be the SAME type.
 *   - IVP_CFE_CLIP is partially implemented; use IVP_CFE_BREAK in production.
 */

#include "ive_sample_app.hxx"

#include <ivp_template_constraint.hxx>
#include <ivp_listener_object.hxx>
#include <SDL3/SDL.h>

#include <cstdio>

static bool g_break_broken = false;
static bool g_clip_broken  = false;  /* CFE_CLIP rarely truly breaks, but track state */

/* Listener to detect when the 'break' constraint's attached cube is
 * revived after the constraint breaks (it will start free-falling). */
class BreakListener : public IVP_Listener_Object {
public:
    void event_object_created(IVP_Event_Object *) {}
    void event_object_deleted(IVP_Event_Object *) {}
    void event_object_frozen(IVP_Event_Object *)  {}
    void event_object_revived(IVP_Event_Object *) {}
};

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title       = "IVP Quickstart: Maxforce Constraints (Sec 5.17)";
    cfg.orbit_dist  = 20.0f;
    cfg.orbit_pitch = -20.0f;
    cfg.target_y    = -3.0f;
    cfg.friction    = 0.5;
    cfg.elasticity  = 0.3;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    double hs  = 0.35;
    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 12.0, 0.5, 12.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 12.0f, 0.5f, 12.0f);

    /* ── Left pair: IVP_CFE_BREAK ── */
    IVP_U_Point p_break_ref(-2.5, -2.0, 0.0);
    IVP_Polygon *cube_break_ref = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.0, &q, &p_break_ref);

    IVP_U_Point p_break_dyn(-2.5, -4.0, 0.0);
    IVP_Polygon *cube_break_dyn = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p_break_dyn);

    {
        IVP_Template_Constraint ct;
        ct.set_ballsocket_ws(cube_break_ref, &p_break_ref, cube_break_dyn);

        /* Set BREAK maximpulse -- all six axes must use the same value */
        float break_impulse = 4.0f; /* N*s: roughly a moderate hit will break it */
        ct.set_max_translation_impulse(IVP_CFE_BREAK, break_impulse);
        ct.set_max_rotation_impulse(IVP_CFE_BREAK, break_impulse);

        IVP_Controller_Factory::create_constraint(app->env, &ct);
    }

    /* ── Right pair: IVP_CFE_CLIP ── */
    IVP_U_Point p_clip_ref(2.5, -2.0, 0.0);
    IVP_Polygon *cube_clip_ref = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.0, &q, &p_clip_ref);

    IVP_U_Point p_clip_dyn(2.5, -4.0, 0.0);
    IVP_Polygon *cube_clip_dyn = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p_clip_dyn);

    {
        IVP_Template_Constraint ct;
        ct.set_ballsocket_ws(cube_clip_ref, &p_clip_ref, cube_clip_dyn);

        /* Set CLIP maximpulse -- constraint gives way but tries to recover */
        float clip_impulse = 4.0f;
        ct.set_max_translation_impulse(IVP_CFE_CLIP, clip_impulse);
        ct.set_max_rotation_impulse(IVP_CFE_CLIP, clip_impulse);

        IVP_Controller_Factory::create_constraint(app->env, &ct);
    }

    ive::app_add_pick(app, cube_break_ref);
    ive::app_add_pick(app, cube_break_dyn);
    ive::app_add_pick(app, cube_clip_ref);
    ive::app_add_pick(app, cube_clip_dyn);

    ive::app_save_initial_state(app);

    bool prev_space = false;

    while (ive::app_begin_frame(app)) {
        if (ive::app_check_reset(app)) {
            g_break_broken = false;
            g_clip_broken  = false;
        }

        /* SPACE: throw both attached cubes upward to exceed the maximpulse */
        if (ive::key_just_pressed(SDL_SCANCODE_SPACE, &prev_space)) {
            IVP_U_Float_Point impulse(0.0f, -12.0f, 0.0f); /* strong upward kick */
            cube_break_dyn->async_add_speed_object_ws(&impulse);
            cube_clip_dyn->async_add_speed_object_ws(&impulse);
        }

        ive::app_step(app);
        ive::app_draw_grid(app);

        /* Draw pairs.  Static refs in static_obj colour, dynamics in pair colour. */
        ive::draw_object_box(app->renderer, cube_break_ref, hs, hs, hs, ive::color::static_obj);
        ive::draw_object_box(app->renderer, cube_break_dyn, hs, hs, hs, ive::color::object_b);

        ive::draw_object_box(app->renderer, cube_clip_ref,  hs, hs, hs, ive::color::static_obj);
        ive::draw_object_box(app->renderer, cube_clip_dyn,  hs, hs, hs, ive::color::object_a);

        /* Draw connector lines between anchored pairs */
        ive::draw_spring_line(app->renderer, cube_break_ref, cube_break_dyn, ive::color::object_b);
        ive::draw_spring_line(app->renderer, cube_clip_ref,  cube_clip_dyn,  ive::color::object_a);

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Maxforce Constraints", 270, 420)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            ive::app_nk_spacing(app);
            ive::app_nk_label(app, "Left  (blue):  IVP_CFE_BREAK");
            ive::app_nk_label(app, "  Constraint deletes itself at limit.");
            ive::app_nk_label(app, "Right (orange): IVP_CFE_CLIP");
            ive::app_nk_label(app, "  Constraint gives way, recovers.");

            ive::app_nk_spacing(app);
            ive::app_nk_label(app, "maximpulse = 4.0 N*s (both pairs)");

            ive::app_nk_controls(app, "SPACE: kick attached cubes upward");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
