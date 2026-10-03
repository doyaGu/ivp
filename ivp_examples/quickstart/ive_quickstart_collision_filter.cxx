/* ive_quickstart_collision_filter.cxx -- Exclusive pair collision filter
 *
 * Based on IVP Manual section 5.14: Collision Filter.
 * Demonstrates: IVP_Collision_Filter_Exclusive_Pair, disable/enable collisions.
 * Press SPACE to toggle collision between two objects.
 */

#include "ive_sample_app.hxx"

#include <ivp_collision_filter.hxx>
#include <SDL3/SDL.h>

#include <cstdio>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Collision Filter";
    cfg.orbit_dist = 15.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    /* Replace the default environment with one that has a collision filter */
    delete app->env;

    IVP_Collision_Filter_Exclusive_Pair *cfilt = new IVP_Collision_Filter_Exclusive_Pair();
    IVP_Application_Environment app_env;
    app_env.collision_filter = cfilt;

    IVP_Environment_Manager *mgr = IVP_Environment_Manager::get_environment_manager();
    app->env = mgr->create_environment(&app_env, "IVP_Sample", 0);
    IVP_U_Point gravity(0.0, 9.81, 0.0);
    app->env->set_gravity(&gravity);
    app->env->set_delta_PSI_time(1.0 / 60.0);
    app->env->client_data = &app->pick_list;

    double hs = 0.4;
    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 10.0, 0.5, 10.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 10.0f, 0.5f, 10.0f);

    /* Two dynamic cubes that can overlap when filter is active */
    IVP_U_Point p1(-0.5, -6.0, 0.0);
    IVP_U_Point p2( 0.5, -3.0, 0.0);
    IVP_Polygon *cube1 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p1);
    IVP_Polygon *cube2 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p2);

    bool filter_active = false;

    ive::app_add_pick(app, cube1);
    ive::app_add_pick(app, cube2);

    ive::app_save_initial_state(app);

    bool prev_space = false;

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        if (ive::key_just_pressed(SDL_SCANCODE_SPACE, &prev_space)) {
            filter_active = !filter_active;
            if (filter_active) {
                cfilt->disable_collision_between_objects(cube1, cube2);
            } else {
                cfilt->enable_collision_between_objects(cube1, cube2);
            }
        }

        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);

        const float *c1c = filter_active ? ive::color::static_obj : ive::color::object_b;
        const float *c2c = filter_active ? ive::color::static_obj : ive::color::object_a;
        ive::draw_object_box(app->renderer, cube1, hs, hs, hs, c1c);
        ive::draw_object_box(app->renderer, cube2, hs, hs, hs, c2c);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Collision Filter", 260, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            char buf[128];
            std::sprintf(buf, "Collision: %s",
                         filter_active ? "DISABLED (pass through)" : "ENABLED (solid)");
            ive::app_nk_object_info(app, buf, cube1);

            ive::app_nk_controls(app, "SPACE: toggle collision filter");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
