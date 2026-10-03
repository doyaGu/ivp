/* ive_quickstart_active_value.cxx -- Active value chain demonstration
 *
 * Based on IVP Manual section 5.11: Active Values.
 * Demonstrates: IVP_U_Active_Terminal_Double, IVP_U_Active_Mult chain.
 * Press SPACE to push cube1 upward; cube2 receives 1.5x the speed.
 */

#include "ive_sample_app.hxx"

#include <ivu_active_value.hxx>
#include <SDL3/SDL.h>

#include <cstdio>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Active Value";
    cfg.orbit_dist = 18.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    double hs = 0.4;
    IVP_U_Quat q; q.init();

    /* Static ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 10.0, 0.5, 10.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 10.0f, 0.5f, 10.0f);

    /* Two dynamic cubes */
    IVP_U_Point p1(-2.0, -hs, 0.0);
    IVP_U_Point p2( 2.0, -hs, 0.0);
    IVP_Polygon *cube1 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p1);
    IVP_Polygon *cube2 = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &p2);

    /* Active value chain: input * factor = output
     * a = speed terminal, b = factor (1.5), c = a * b */
    IVP_U_Active_Terminal_Double *av_speed =
        new IVP_U_Active_Terminal_Double("speed", 0.0);
    IVP_U_Active_Terminal_Double *av_factor =
        new IVP_U_Active_Terminal_Double("factor", 1.5);
    IVP_U_Active_Mult *av_mult =
        new IVP_U_Active_Mult("result", av_speed, av_factor);

    ive::app_add_pick(app, cube1);
    ive::app_add_pick(app, cube2);

    ive::app_save_initial_state(app);

    bool prev_space = false;

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        if (ive::key_just_pressed(SDL_SCANCODE_SPACE, &prev_space)) {
            /* Push cube1 up */
            IVP_U_Float_Point speed(0.0, -4.0, 0.0);
            cube1->async_add_speed_object_ws(&speed);

            /* Set the active value; cube2 gets multiplied speed */
            av_speed->set_double(4.0);
            double result = av_mult->give_double_value();
            IVP_U_Float_Point speed2(0.0, -result, 0.0);
            cube2->async_add_speed_object_ws(&speed2);
        }

        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, cube1, hs, hs, hs, ive::color::object_b);
        ive::draw_object_box(app->renderer, cube2, hs, hs, hs, ive::color::object_a);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Active Value", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            char buf[128];
            std::sprintf(buf, "speed(%.1f) * factor(%.1f) = %.1f",
                         av_speed->give_double_value(),
                         av_factor->give_double_value(),
                         av_mult->give_double_value());
            ive::app_nk_object_info(app, "Cube1 (orange)", cube1);
            ive::app_nk_object_info(app, "Cube2 (blue)", cube2);

            ive::app_nk_controls(app, "SPACE: push cube1 (cube2 = 1.5x)");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    delete av_mult;
    delete av_factor;
    delete av_speed;
    ive::app_destroy(app);
    return 0;
}
