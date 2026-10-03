/* ive_quickstart_cube.cxx -- A simple static cube floating in mid-air
 *
 * Based on IVP Manual section 5.2: Creating a simple cube.
 * Demonstrates: point soup surface builder, static (unmoveable) objects.
 */

#include "ive_sample_app.hxx"

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Cube";
    cfg.orbit_dist = 15.0f;
    cfg.target_y = -5.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    /* Static cube at (0, -5, 0), half-extents 0.5m */
    IVP_U_Point pos(0.0, -5.0, 0.0);
    IVP_Polygon *cube = ive::create_box(app->env, &app->mat, 0.5, 0.5, 0.5, 0.0, &q, &pos);

    ive::app_add_pick(app, cube);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube, 0.5, 0.5, 0.5, ive::color::object_b);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Cube", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_object_info(app, "Cube (static)", cube);
            ive::app_nk_label(app, "This cube is physically unmoveable");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
