/* ive_quickstart_stiff_spring.cxx -- Two cubes connected with a stiff spring
 *
 * Based on IVP Manual section 5.6 (stiff spring variant).
 * Demonstrates: IVP_Controller_Stiff_Spring, numerically stable springs.
 */

#include "ive_sample_app.hxx"

#include <ivp_controller_stiff_spring.hxx>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Stiff Spring";
    cfg.orbit_dist = 15.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    double hs = 0.3;
    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    IVP_U_Point p1(-1.0, -5.0, 0.0);
    IVP_U_Point p2( 1.0, -5.0, 0.0);
    IVP_Polygon *cube1 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p1);
    IVP_Polygon *cube2 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p2);

    /* Create stiff spring */
    IVP_Template_Anchor anchor1, anchor2;
    anchor1.set_anchor_position_os(cube1, 0.0, 0.0, 0.0);
    anchor2.set_anchor_position_os(cube2, 0.0, 0.0, 0.0);

    IVP_Template_Stiff_Spring ss_template;
    ss_template.anchors[0] = &anchor1;
    ss_template.anchors[1] = &anchor2;
    ss_template.spring_constant = 0.002f;
    ss_template.spring_len = 0.5f;
    ss_template.spring_damp = 0.01f;
    new IVP_Controller_Stiff_Spring(app->env, &ss_template);

    ive::app_add_pick(app, cube1);
    ive::app_add_pick(app, cube2);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube1, hs, hs, hs, ive::color::object_b);
        ive::draw_object_box(app->renderer, cube2, hs, hs, hs, ive::color::object_a);
        ive::draw_spring_line(app->renderer, cube1, cube2, ive::color::spring);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Stiff Spring", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_label(app, "Stiff Spring: k=0.002, len=0.5, damp=0.01");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
