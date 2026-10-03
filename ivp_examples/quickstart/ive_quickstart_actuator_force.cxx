/* ive_quickstart_actuator_force.cxx -- Force actuator (magnet)
 *
 * Based on IVP Manual section 5.7: Connecting two objects with a force actuator.
 * Demonstrates: IVP_Template_Force, set_force() toggle. Press SPACE to toggle.
 */

#include "ive_sample_app.hxx"

#include <ivp_actuator.hxx>
#include <SDL3/SDL.h>

#include <cstdio>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Force Actuator";
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

    IVP_U_Point p1(-2.0, -5.0, 0.0);
    IVP_U_Point p2( 2.0, -5.0, 0.0);
    IVP_Polygon *cube1 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p1);
    IVP_Polygon *cube2 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p2);

    /* Force actuator between centers */
    IVP_Template_Anchor a1, a2;
    a1.set_anchor_position_os(cube1, 0.0, 0.0, 0.0);
    a2.set_anchor_position_os(cube2, 0.0, 0.0, 0.0);

    IVP_Template_Force force_template;
    force_template.anchors[0] = &a1;
    force_template.anchors[1] = &a2;
    force_template.force = 5.0f;
    force_template.push_first_object = IVP_TRUE;
    force_template.push_second_object = IVP_FALSE;

    IVP_Actuator_Force *actuator = IVP_Controller_Factory::create_force(app->env, &force_template);
    bool force_on = true;

    ive::app_add_pick(app, cube1);
    ive::app_add_pick(app, cube2);

    ive::app_save_initial_state(app);

    bool prev_space = false;

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        /* Toggle force with SPACE */
        if (ive::key_just_pressed(SDL_SCANCODE_SPACE, &prev_space)) {
            force_on = !force_on;
            actuator->set_force(force_on ? 5.0f : 0.0f);
        }

        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube1, hs, hs, hs, ive::color::object_b);
        ive::draw_object_box(app->renderer, cube2, hs, hs, hs, ive::color::object_a);

        /* Draw force line when active */
        ive::draw_spring_line(app->renderer, cube1, cube2,
                              force_on ? ive::color::warning : ive::color::static_obj);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Force Actuator", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            char buf[64];
            std::sprintf(buf, "Force: %s", force_on ? "ON" : "OFF");
            ive::app_nk_label(app, buf);

            ive::app_nk_controls(app, "SPACE: toggle force");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
