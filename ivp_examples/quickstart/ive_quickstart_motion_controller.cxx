/* ive_quickstart_motion_controller.cxx -- Box seeking a target position
 *
 * Based on IVP Manual section 5.16: Motion Controller.
 * Demonstrates: IVP_Controller_Motion, set_target_position_ws().
 * Press T to set a new target; the box slides toward it.
 */

#include "ive_sample_app.hxx"

#include <ivp_controller_motion.hxx>
#include <SDL3/SDL.h>

#include <cstdio>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Motion Controller";
    cfg.orbit_dist = 20.0f;
    cfg.orbit_pitch = -30.0f;
    cfg.target_y = -1.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    /* Ground */
    IVP_U_Quat q; q.init();
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 10.0, 0.5, 10.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 10.0f, 0.5f, 10.0f);

    /* Dynamic box */
    double hs = 0.5;
    IVP_U_Point pos(0.0, -hs, 0.0);
    IVP_Polygon *cube = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &pos);

    /* Motion controller */
    IVP_Template_Controller_Motion mct;
    mct.force_factor = 0.8f;
    mct.damp_factor = 1.2f;
    mct.torque_factor = 0.0f;
    mct.angular_damp_factor = 0.5f;
    mct.max_translation_force.set(50.0, 50.0, 50.0);
    mct.max_torque = 10.0f;

    IVP_Controller_Motion *mc = new IVP_Controller_Motion(cube, &mct);

    IVP_U_Point target_pos(0.0, -hs, 0.0);
    mc->set_target_position_ws(&target_pos);

    ive::app_add_pick(app, cube);

    ive::app_save_initial_state(app);

    bool prev_t = false;
    float tx = 0.0f, tz = 0.0f;

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        /* T key: move target to a new random-ish position */
        if (ive::key_just_pressed(SDL_SCANCODE_T, &prev_t)) {
            tx = (float)(((int)(tx * 10 + 37) % 100) - 50) * 0.1f;
            tz = (float)(((int)(tz * 10 + 53) % 100) - 50) * 0.1f;
            target_pos.set(tx, -hs, tz);
            mc->set_target_position_ws(&target_pos);
        }

        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube, hs, hs, hs, ive::color::object_b);

        /* Draw target marker */
        float tpos[3] = {(float)target_pos.k[0], (float)target_pos.k[1], (float)target_pos.k[2]};
        ivp_draw_wire_sphere(app->renderer, tpos, 0.3f, ive::color::highlight);

        /* Draw arrow from cube to target */
        float cpos[3];
        ive::point_to_float3(cube->get_core()->get_position_PSI(), cpos);
        ivp_draw_arrow(app->renderer, cpos, tpos, ive::color::velocity, 0.15f);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Motion Controller", 260, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            ive::app_nk_object_info(app, "Cube", cube);

            char buf[128];
            std::sprintf(buf, "Target: (%.1f, %.1f, %.1f)",
                         target_pos.k[0], target_pos.k[1], target_pos.k[2]);
            ive::app_nk_label(app, buf);

            ive::app_nk_controls(app, "T: move target to new position");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
