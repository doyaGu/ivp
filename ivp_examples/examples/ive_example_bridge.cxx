/* ive_example_bridge.cxx -- Bridge of constraint-connected planks
 *
 * Two static pillars with a chain of planks connected by ball-socket
 * constraints. A heavy box is dropped onto the bridge.
 */

#include "ive_sample_app.hxx"

#include <ivp_template_constraint.hxx>
#include <SDL3/SDL.h>

#include <cstdio>

#define NUM_PLANKS 10

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Example: Bridge";
    cfg.orbit_dist = 20.0f;
    cfg.orbit_pitch = -20.0f;
    cfg.target_y = -1.0f;
    cfg.friction = 0.6;
    cfg.elasticity = 0.5;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 20.0, 0.5, 5.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 20.0f, 0.5f, 5.0f);

    /* Left pillar */
    double pillar_hx = 0.5, pillar_hy = 1.5, pillar_hz = 0.5;
    IVP_U_Point pl(-5.0, -pillar_hy, 0.0);
    IVP_Polygon *left_pillar = ive::create_box(app->env, &app->mat,
        pillar_hx, pillar_hy, pillar_hz, 0.0, &q, &pl);

    /* Right pillar */
    IVP_U_Point pr(5.0, -pillar_hy, 0.0);
    IVP_Polygon *right_pillar = ive::create_box(app->env, &app->mat,
        pillar_hx, pillar_hy, pillar_hz, 0.0, &q, &pr);

    /* Planks spanning between pillars */
    double plank_hx = 0.4, plank_hy = 0.06, plank_hz = 0.4;
    double start_x = -4.0;
    double end_x = 4.0;
    double spacing = (end_x - start_x) / (NUM_PLANKS - 1);
    double plank_y = -(pillar_hy * 2.0) - plank_hy;

    IVP_Polygon *planks[NUM_PLANKS];
    for (int i = 0; i < NUM_PLANKS; i++) {
        double x = start_x + i * spacing;
        IVP_U_Point pp(x, plank_y, 0.0);
        planks[i] = ive::create_box(app->env, &app->mat,
            plank_hx, plank_hy, plank_hz, 0.3, &q, &pp);
    }

    /* Connect first plank to left pillar */
    {
        IVP_U_Point anchor_ws(start_x - plank_hx, plank_y, 0.0);
        IVP_Template_Constraint ct;
        ct.set_ballsocket_ws(left_pillar, &anchor_ws, planks[0]);
        IVP_Controller_Factory::create_constraint(app->env, &ct);
    }

    /* Connect adjacent planks */
    for (int i = 0; i < NUM_PLANKS - 1; i++) {
        double x = start_x + (i + 0.5) * spacing;
        IVP_U_Point anchor_ws(x, plank_y, 0.0);
        IVP_Template_Constraint ct;
        ct.set_ballsocket_ws(planks[i], &anchor_ws, planks[i + 1]);
        IVP_Controller_Factory::create_constraint(app->env, &ct);
    }

    /* Connect last plank to right pillar */
    {
        IVP_U_Point anchor_ws(end_x + plank_hx, plank_y, 0.0);
        IVP_Template_Constraint ct;
        ct.set_ballsocket_ws(right_pillar, &anchor_ws, planks[NUM_PLANKS - 1]);
        IVP_Controller_Factory::create_constraint(app->env, &ct);
    }

    /* Heavy box dropped onto the bridge */
    IVP_U_Point pb(0.0, -6.0, 0.0);
    IVP_Polygon *heavy_box = ive::create_box(app->env, &app->mat, 0.4, 0.4, 0.4, 5.0, &q, &pb);

    for (int i = 0; i < NUM_PLANKS; i++) ive::app_add_pick(app, planks[i]);
    ive::app_add_pick(app, heavy_box);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);
        ive::app_draw_grid(app);

        ive::draw_object_box(app->renderer, left_pillar, pillar_hx, pillar_hy, pillar_hz, ive::color::static_obj);
        ive::draw_object_box(app->renderer, right_pillar, pillar_hx, pillar_hy, pillar_hz, ive::color::static_obj);

        for (int i = 0; i < NUM_PLANKS; i++)
            ive::draw_object_box(app->renderer, planks[i], plank_hx, plank_hy, plank_hz, ive::color::object_b);

        /* Draw connection lines between adjacent planks */
        for (int i = 0; i < NUM_PLANKS - 1; i++)
            ive::draw_spring_line(app->renderer, planks[i], planks[i + 1], ive::color::object_b);

        ive::draw_object_box(app->renderer, heavy_box, 0.4, 0.4, 0.4, ive::color::warning);

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Bridge", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            char buf[64];
            std::sprintf(buf, "Planks: %d", NUM_PLANKS);
            ive::app_nk_label(app, buf);

            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
