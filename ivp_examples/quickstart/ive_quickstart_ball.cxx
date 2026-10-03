/* ive_quickstart_ball.cxx -- A simple ball falling down
 *
 * Based on IVP Manual section 5.1: Creating a simple ball.
 * Demonstrates: environment creation, ball object, gravity, velocity arrow.
 */

#include "ive_sample_app.hxx"

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Ball";
    cfg.orbit_dist = 20.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    /* Ball at (0, -5, 0), radius 1m, mass 1kg */
    IVP_U_Point pos(0.0, -5.0, 0.0);
    IVP_Ball *ball = ive::create_ball(app->env, &app->mat, 1.0, 1.0, &q, &pos);

    ive::app_add_pick(app, ball);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_ball(app->renderer, ball, 1.0, ive::color::object_a);
        ive::draw_velocity_arrow(app->renderer, ball, ive::color::velocity);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Ball", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_object_info(app, "Ball", ball);
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
