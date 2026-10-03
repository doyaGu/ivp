/* ive_quickstart_check_distance.cxx -- Distance checker with active value chain
 *
 * Based on IVP Manual section 5.13: Checking distances between objects.
 * Demonstrates: IVP_Actuator_Check_Dist, IVP_U_Active_Switch, hinge+rotmot.
 * Pendulum swings near a box; when in range, the box spins via active values.
 */

#include "ive_sample_app.hxx"

#include <ivp_template_constraint.hxx>
#include <ivp_actuator.hxx>
#include <ivu_active_value.hxx>

#include <cstdio>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Check Distance";
    cfg.orbit_dist = 20.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    double hs = 0.3;
    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    /* Anchor (static) for pendulum */
    IVP_U_Point p_anchor(0.0, -6.0, 0.0);
    IVP_Polygon *anchor_cube = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.0, &q, &p_anchor);

    /* Pendulum end (dynamic) */
    IVP_U_Point p_pend(3.0, -3.0, 0.0);
    IVP_Polygon *pend_cube = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p_pend);

    /* Hinge connecting anchor to pendulum */
    IVP_U_Point hinge_pt(0.0, 0.0, 0.0);
    IVP_U_Point hinge_ax(0.0, 0.0, 1.0);
    IVP_Template_Constraint hinge_ct;
    hinge_ct.set_hinge_Ros(anchor_cube, &hinge_pt, &hinge_ax, pend_cube);
    IVP_Controller_Factory::create_constraint(app->env, &hinge_ct);

    /* Rotating box (dynamic) */
    IVP_U_Point p_rot(3.0, -1.0, 0.0);
    IVP_Polygon *rot_cube = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p_rot);

    /* Active value chain: condition -> switch -> rotmot power */
    IVP_U_Active_Terminal_Int *condition =
        new IVP_U_Active_Terminal_Int("condition", 0);
    IVP_U_Active_Terminal_Double *rot_active =
        new IVP_U_Active_Terminal_Double("rot_active", 10.0);
    IVP_U_Active_Terminal_Double *rot_inactive =
        new IVP_U_Active_Terminal_Double("rot_inactive", 0.0);
    IVP_U_Active_Switch *rotmot_output =
        new IVP_U_Active_Switch("switch", condition, rot_inactive, rot_active);

    /* Rotation motor on the rotating box, powered by the switch */
    IVP_Template_Anchor a_lo, a_hi;
    a_lo.set_anchor_position_os(rot_cube, 0.0, -1.0, 0.0);
    a_hi.set_anchor_position_os(rot_cube, 0.0,  1.0, 0.0);

    IVP_Template_Rot_Mot rmt;
    rmt.max_rotation_speed = 40.0f;
    rmt.active_float_power = rotmot_output;
    rmt.max_torque = 100.8f;
    rmt.anchors[0] = &a_lo;
    rmt.anchors[1] = &a_hi;
    IVP_Controller_Factory::create_rotmot(app->env, &rmt);

    /* Distance checker: pendulum <-> rotating box, range 3m */
    IVP_Template_Check_Dist dist_t;
    dist_t.objects[0] = pend_cube;
    dist_t.objects[1] = rot_cube;
    dist_t.range = 3.0f;
    dist_t.mod_is_outside = condition;

    /* Convert object centers to world coords */
    IVP_U_Float_Point center(0.0, 0.0, 0.0);
    {
        IVP_U_Point wc;
        pend_cube->transform_position_to_world_coords(&center, &wc);
        dist_t.position_world_space[0].set(&wc);
    }
    {
        IVP_U_Point wc;
        rot_cube->transform_position_to_world_coords(&center, &wc);
        dist_t.position_world_space[1].set(&wc);
    }
    IVP_Controller_Factory::create_check_dist(app->env, &dist_t);

    ive::app_add_pick(app, anchor_cube);
    ive::app_add_pick(app, pend_cube);
    ive::app_add_pick(app, rot_cube);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);

        ive::draw_object_box(app->renderer, anchor_cube, hs, hs, hs, ive::color::static_obj);
        ive::draw_object_box(app->renderer, pend_cube, hs, hs, hs, ive::color::object_b);
        ive::draw_object_box(app->renderer, rot_cube, hs, hs, hs, ive::color::object_a);
        ive::draw_spring_line(app->renderer, anchor_cube, pend_cube, ive::color::spring);

        /* Draw range sphere around rotating box */
        float rot_pos[3];
        ive::point_to_float3(rot_cube->get_core()->get_position_PSI(), rot_pos);
        ivp_draw_wire_sphere(app->renderer, rot_pos, 3.0f, ive::color::highlight);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Check Distance", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            bool in_range = (condition->give_int_value() == 0);
            char buf[128];
            std::sprintf(buf, "Pendulum %s range (motor %s)",
                         in_range ? "IN" : "OUT of", in_range ? "ON" : "OFF");
            ive::app_nk_object_info(app, buf, pend_cube);
            ive::app_nk_object_info(app, "Rotating box", rot_cube);

            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
