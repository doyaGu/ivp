/* ive_quickstart_rotmot.cxx -- Rotation motor spinning a cube
 *
 * Based on IVP Manual section 5.8: Adding a rotation motor.
 * Demonstrates: IVP_Template_Rot_Mot, rotation axis definition via anchors.
 */

#include "ive_sample_app.hxx"

#include <ivp_actuator.hxx>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Rotation Motor";
    cfg.orbit_dist = 12.0f;
    cfg.target_y = -5.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    double hs = 0.5;
    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    IVP_U_Point pos(0.0, -5.0, 0.0);
    IVP_Polygon *cube = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &pos);

    /* Rotation motor on Y-axis: two anchors above and below center */
    IVP_Template_Anchor a_lo, a_hi;
    a_lo.set_anchor_position_os(cube, 0.0, -1.0, 0.0);
    a_hi.set_anchor_position_os(cube, 0.0,  1.0, 0.0);

    IVP_Template_Rot_Mot rmt;
    rmt.max_rotation_speed = 40.0f;
    rmt.power = 4.0f;
    rmt.max_torque = 3.8f;
    rmt.anchors[0] = &a_lo;
    rmt.anchors[1] = &a_hi;
    IVP_Controller_Factory::create_rotmot(app->env, &rmt);

    ive::app_add_pick(app, cube);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube, hs, hs, hs, ive::color::object_a);

        /* Draw rotation axis */
        float p1[3], p2[3];
        float cpos[3];
        ive::point_to_float3(cube->get_core()->get_position_PSI(), cpos);
        p1[0] = cpos[0]; p1[1] = cpos[1] - 1.5f; p1[2] = cpos[2];
        p2[0] = cpos[0]; p2[1] = cpos[1] + 1.5f; p2[2] = cpos[2];
        ivp_draw_arrow(app->renderer, p1, p2, ive::color::highlight, 0.2f);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Rotation Motor", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_object_info(app, "Cube", cube);
            ive::app_nk_label(app, "Motor: speed=40, power=4, torque=3.8");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
