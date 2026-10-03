/* ive_quickstart_actuator_spring.cxx -- Two cubes connected with a spring
 *
 * Based on IVP Manual section 5.6: Connecting two objects with a spring.
 * Demonstrates: IVP_Template_Spring, IVP_Template_Anchor, spring visualization.
 */

#include "ive_sample_app.hxx"

#include <ivp_actuator_spring.hxx>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Spring";
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

    /* Two cubes side by side */
    IVP_U_Point p1(-1.5, -5.0, 0.0);
    IVP_U_Point p2( 1.5, -5.0, 0.0);
    IVP_Polygon *cube1 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p1);
    IVP_Polygon *cube2 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p2);

    /* Create spring between cube centers */
    IVP_Template_Anchor anchor1, anchor2;
    anchor1.set_anchor_position_os(cube1, 0.0, 0.0, 0.0);
    anchor2.set_anchor_position_os(cube2, 0.0, 0.0, 0.0);

    IVP_Template_Spring spring_template;
    spring_template.spring_values_are_relative = IVP_FALSE;
    spring_template.spring_constant = 10.0f;
    spring_template.spring_len = 0.7f;
    spring_template.spring_damp = 0.1f;
    spring_template.rel_pos_damp = 0.05f;
    spring_template.spring_force_only_on_stretch = IVP_FALSE;
    spring_template.anchors[0] = &anchor1;
    spring_template.anchors[1] = &anchor2;
    IVP_Controller_Factory::create_spring(app->env, &spring_template);

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
        if (ive::app_begin_info_panel(app, "Spring", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_label(app, "Spring: k=10, len=0.7, damp=0.1");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
