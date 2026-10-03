/* ive_quickstart_hinge.cxx -- Two cubes connected by a hinge constraint
 *
 * Based on IVP Manual section 5.10: Connecting two objects with a hinge.
 * Demonstrates: IVP_Template_Constraint::set_hinge_Ros, free rotation axis.
 */

#include "ive_sample_app.hxx"

#include <ivp_template_constraint.hxx>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Hinge";
    cfg.orbit_dist = 15.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    double hs = 0.4;
    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    /* Reference (static) cube */
    IVP_U_Point p1(0.0, -5.0, 0.0);
    IVP_Polygon *cube1 = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.0, &q, &p1);

    /* Attached (dynamic) cube, offset along X */
    IVP_U_Point p2(2.0, -5.0, 0.0);
    IVP_Polygon *cube2 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p2);

    /* Hinge: pivot at cube1's center, free axis along Z (in cube1's object space) */
    IVP_U_Point hinge_coord(0.0, 0.0, 0.0);
    IVP_U_Point hinge_axis(0.0, 0.0, 1.0);

    IVP_Template_Constraint ct;
    ct.set_hinge_Ros(cube1, &hinge_coord, &hinge_axis, cube2);
    IVP_Controller_Factory::create_constraint(app->env, &ct);

    ive::app_add_pick(app, cube1);
    ive::app_add_pick(app, cube2);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube1, hs, hs, hs, ive::color::static_obj);
        ive::draw_object_box(app->renderer, cube2, hs, hs, hs, ive::color::object_b);
        ive::draw_spring_line(app->renderer, cube1, cube2, ive::color::highlight);

        /* Draw hinge axis at cube1 */
        float pos1[3];
        ive::point_to_float3(cube1->get_core()->get_position_PSI(), pos1);
        float axis_a[3] = {pos1[0], pos1[1], pos1[2] - 1.5f};
        float axis_b[3] = {pos1[0], pos1[1], pos1[2] + 1.5f};
        ivp_draw_arrow(app->renderer, axis_a, axis_b, ive::color::highlight, 0.15f);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Hinge", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_object_info(app, "Static cube", cube1);
            ive::app_nk_object_info(app, "Dynamic cube", cube2);
            ive::app_nk_label(app, "Hinge axis: Z (shown in yellow)");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
