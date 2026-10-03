/* ive_quickstart_collision.cxx -- A ball colliding with a cube
 *
 * Based on IVP Manual section 5.4: A ball colliding with a cube.
 * Demonstrates: collision between dynamic ball and tilted static cube.
 */

#include "ive_sample_app.hxx"

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Collision";
    cfg.orbit_dist = 20.0f;
    cfg.target_y = -5.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    /* Static tilted cube at origin */
    IVP_U_Quat q_cube(IVP_U_Point(0.0, 0.0, 0.3));
    IVP_U_Point pos_cube(0.0, -2.0, 0.0);
    IVP_Polygon *cube = ive::create_box(app->env, &app->mat, 1.5, 0.5, 1.5, 0.0, &q_cube, &pos_cube);

    /* Dynamic ball above the cube */
    IVP_U_Quat q_ball; q_ball.init();
    IVP_U_Point pos_ball(0.0, -8.0, 0.0);
    IVP_Ball *ball = ive::create_ball(app->env, &app->mat, 0.5, 1.0, &q_ball, &pos_ball);

    ive::app_add_pick(app, cube);
    ive::app_add_pick(app, ball);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube, 1.5, 0.5, 1.5, ive::color::static_obj);
        ive::draw_object_ball(app->renderer, ball, 0.5, ive::color::object_a);
        ive::draw_velocity_arrow(app->renderer, ball, ive::color::velocity);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Collision", 250, 400)) {
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
