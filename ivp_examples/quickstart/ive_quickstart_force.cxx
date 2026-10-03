/* ive_quickstart_force.cxx -- Applying forces to an object
 *
 * Based on IVP Manual section 5.5: Applying forces to an object.
 * Demonstrates: async_add_speed_object_ws (S key), async_push_object_ws (U key).
 */

#include "ive_sample_app.hxx"

#include <SDL3/SDL.h>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Force";
    cfg.orbit_dist = 15.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    /* Static ground plane */
    IVP_U_Quat qg; qg.init();
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 10.0, 0.5, 10.0, 0.0, &qg, &pg);
    ive::app_set_ground(app, ground, 10.0f, 0.5f, 10.0f);

    /* Dynamic cube sitting on the ground */
    double hs = 0.5;
    IVP_U_Quat qc; qc.init();
    IVP_U_Point pc(0.0, -hs, 0.0);
    IVP_Polygon *cube = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &qc, &pc);

    ive::app_add_pick(app, cube);

    ive::app_save_initial_state(app);

    bool prev_s = false;
    bool prev_u = false;

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        /* S key: add speed (uniform push upward) */
        if (ive::key_just_pressed(SDL_SCANCODE_S, &prev_s)) {
            IVP_U_Float_Point speed(0.0, -3.0, 0.0);
            cube->async_add_speed_object_ws(&speed);
        }

        /* U key: push at corner (causes rotation) */
        if (ive::key_just_pressed(SDL_SCANCODE_U, &prev_u)) {
            IVP_U_Float_Point corner_os(hs, hs, hs);
            IVP_U_Point world_coords;
            cube->transform_position_to_world_coords(&corner_os, &world_coords);
            IVP_U_Float_Point force(0.0, -100.0, 0.0);
            cube->async_push_object_ws(&world_coords, &force);
        }

        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube, hs, hs, hs, ive::color::object_b);
        ive::draw_velocity_arrow(app->renderer, cube, ive::color::velocity);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Force", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_object_info(app, "Cube", cube);
            ive::app_nk_controls(app,
                "S: add speed upward\n"
                "U: push at corner (torque)");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
