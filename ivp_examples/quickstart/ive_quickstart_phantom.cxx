/* ive_quickstart_phantom.cxx -- Phantom objects
 *
 * Based on IVP Manual section 6.11: Phantom objects.
 * Demonstrates: IVP_Template_Phantom, IVP_Controller_Phantom,
 *               IVP_Listener_Phantom, IVP_Real_Object::convert_to_phantom().
 *
 * A phantom is an object that participates in collision *detection* but
 * not in collision *resolution* -- it is a trigger volume.  Objects
 * entering or leaving the phantom's volume generate listener callbacks.
 *
 * Scene:
 *   - A large static trigger sphere (the phantom) hangs in the scene.
 *   - Several dynamic balls fall through it.
 *   - Balls currently INSIDE the phantom are drawn in a warning colour.
 *   - The info panel counts how many balls are inside.
 *
 * IVP_Template_Phantom fields used:
 *   manage_intruding_objects  -- engine maintains the intruder set for us
 *   exit_policy_extra_radius  -- hysteresis: object must move this far beyond
 *                                the phantom boundary before exit fires
 */

#include "ive_sample_app.hxx"

#include <ivp_phantom.hxx>
#include <SDL3/SDL.h>

#include <cstdio>
#include <cstring>

/* ── Global state shared between listener and main loop ──────────────── */

#define MAX_BALLS 12

static IVP_Real_Object *g_balls[MAX_BALLS];
static bool             g_inside[MAX_BALLS];   /* true when ball is in phantom */
static int              g_num_balls = 0;
static int              g_inside_count = 0;

static int ball_index(IVP_Real_Object *obj) {
    for (int i = 0; i < g_num_balls; i++)
        if (g_balls[i] == obj) return i;
    return -1;
}

/* ── Phantom listener ─────────────────────────────────────────────────── */

class My_Phantom_Listener : public IVP_Listener_Phantom {
public:
    /* Called when a new object's distance to the phantom edge reaches 0 */
    virtual void mindist_entered_volume(IVP_Controller_Phantom * /*ctrl*/,
                                        IVP_Mindist_Base *mindist)
    {
        /* mindist->get_objects() would give us the pair -- but we use the
         * intruding-objects set instead, which is simpler (see core_entered). */
        (void)mindist;
    }

    virtual void mindist_left_volume(IVP_Controller_Phantom * /*ctrl*/,
                                     IVP_Mindist_Base * /*mindist*/) {}

    /*
     * core_entered_volume / core_left_volume are the high-level callbacks:
     * fired once per object core whenever it enters / exits the phantom.
     * These are cleaner than mindist_entered/left for typical trigger use.
     */
    virtual void core_entered_volume(IVP_Controller_Phantom * /*ctrl*/,
                                     IVP_Core *core)
    {
        /* A core may be shared by multiple objects; iterate its objects */
        for (int i = 0; i < g_num_balls; i++) {
            if (g_balls[i]->get_core() == core && !g_inside[i]) {
                g_inside[i] = true;
                g_inside_count++;
            }
        }
    }

    virtual void core_left_volume(IVP_Controller_Phantom * /*ctrl*/,
                                  IVP_Core *core)
    {
        for (int i = 0; i < g_num_balls; i++) {
            if (g_balls[i]->get_core() == core && g_inside[i]) {
                g_inside[i] = false;
                g_inside_count--;
            }
        }
    }

    virtual void phantom_is_going_to_be_deleted_event(IVP_Controller_Phantom * /*ctrl*/) {}
};

static My_Phantom_Listener g_phantom_listener;

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title       = "IVP Quickstart: Phantom Objects (Sec 6.11)";
    cfg.orbit_dist  = 22.0f;
    cfg.orbit_pitch = -20.0f;
    cfg.target_y    = -3.0f;
    cfg.friction    = 0.3;
    cfg.elasticity  = 0.5;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 1.0, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 12.0, 0.5, 12.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 12.0f, 0.5f, 12.0f);

    /* ── Create phantom: a static ball converted to phantom ── */
    /* Place it in the middle of the scene; its shape defines the trigger volume */
    IVP_U_Point phantom_pos(0.0, -3.5, 0.0);
    IVP_Real_Object *phantom_obj = ive::create_ball(
        app->env, &app->mat, 1.6 /* radius */, 0.0 /* unmovable */, &q, &phantom_pos);

    /* Template controlling the phantom's detection behaviour */
    IVP_Template_Phantom pt;
    pt.manage_intruding_objects = IVP_TRUE;    /* engine tracks intruder set */
    pt.manage_intruding_cores   = IVP_TRUE;    /* also track by core */
    pt.dont_check_for_unmoveables = IVP_TRUE;  /* only dynamic objects trigger events */
    pt.exit_policy_extra_radius = 0.2f;        /* require 0.2m clearance before exit fires */

    phantom_obj->convert_to_phantom(&pt);

    /* Attach our listener */
    IVP_Controller_Phantom *ctrl = phantom_obj->get_controller_phantom();
    ctrl->add_listener_phantom(&g_phantom_listener);

    /* ── Dynamic balls dropped from above ── */
    memset(g_inside, 0, sizeof(g_inside));

    for (int i = 0; i < 6; i++) {
        double x = (i % 3 - 1) * 1.2;
        double z = (i / 3)     * 1.2 - 0.6;
        IVP_U_Point pos(x, -6.0 - i * 0.8, z);
        double r = 0.25 + i * 0.02;
        IVP_Real_Object *b = ive::create_ball(app->env, &app->mat, r, 0.5, &q, &pos);
        g_balls[g_num_balls] = b;
        g_inside[g_num_balls] = false;
        g_num_balls++;
        ive::app_add_pick(app, b);
    }

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        if (ive::app_check_reset(app)) {
            /* Reset inside flags */
            memset(g_inside, 0, sizeof(g_inside));
            g_inside_count = 0;
        }

        ive::app_step(app);
        ive::app_draw_grid(app);

        /* Draw phantom volume as a wireframe sphere */
        float ph_pos[3];
        ive::point_to_float3(phantom_obj->get_core()->get_position_PSI(), ph_pos);
        ivp_draw_wire_sphere(app->renderer, ph_pos, 1.6f, ive::color::highlight);

        /* Draw dynamic balls -- warning colour if inside phantom */
        for (int i = 0; i < g_num_balls; i++) {
            double r = 0.25 + i * 0.02;
            const float *col = g_inside[i] ? ive::color::warning : ive::color::object_b;
            ive::draw_object_ball(app->renderer, g_balls[i], r, col);
        }

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Phantom Objects", 270, 420)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            ive::app_nk_spacing(app);

            char buf[128];
            std::sprintf(buf, "Balls inside phantom: %d / %d", g_inside_count, g_num_balls);
            ive::app_nk_label(app, buf);

            ive::app_nk_spacing(app);
            ive::app_nk_label(app, "Phantom = trigger volume:");
            ive::app_nk_label(app, "  no collision response,");
            ive::app_nk_label(app, "  fires enter/leave callbacks.");
            ive::app_nk_label(app, "Yellow sphere = phantom boundary.");
            ive::app_nk_label(app, "Orange balls  = inside phantom.");

            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
