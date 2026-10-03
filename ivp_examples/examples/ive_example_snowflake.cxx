/* ive_example_snowflake.cxx -- Spring platform with falling objects
 *
 * A platform of boxes connected by springs, with objects dropped on top.
 */

#include "ive_sample_app.hxx"

#include <ivp_actuator_spring.hxx>
#include <SDL3/SDL.h>

#include <cstdio>

#define GRID_SIZE 4
#define NUM_PLATFORM (GRID_SIZE * GRID_SIZE)
#define NUM_DROPS 6

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title       = "IVP Example: Snowflake";
    cfg.orbit_dist  = 25.0f;
    cfg.orbit_pitch = -25.0f;
    cfg.target_y    = -2.0f;
    cfg.friction    = 0.5;
    cfg.elasticity  = 0.4;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    double hs = 0.3;
    double spacing = 1.2;
    IVP_U_Quat q; q.init();

    /* Create grid of platform boxes */
    IVP_Polygon *platform[NUM_PLATFORM];
    for (int gz = 0; gz < GRID_SIZE; gz++) {
        for (int gx = 0; gx < GRID_SIZE; gx++) {
            int idx = gz * GRID_SIZE + gx;
            double x = (gx - GRID_SIZE / 2.0 + 0.5) * spacing;
            double z = (gz - GRID_SIZE / 2.0 + 0.5) * spacing;
            IVP_U_Point pos(x, -2.0, z);
            platform[idx] = ive::create_box(app->env, &app->mat, hs, hs * 0.3, hs, 0.5, &q, &pos);
        }
    }

    /* Connect adjacent platform boxes with springs */
    for (int gz = 0; gz < GRID_SIZE; gz++) {
        for (int gx = 0; gx < GRID_SIZE; gx++) {
            int idx = gz * GRID_SIZE + gx;
            /* Right neighbor */
            if (gx + 1 < GRID_SIZE) {
                int nb = gz * GRID_SIZE + (gx + 1);
                IVP_Template_Anchor a1, a2;
                a1.set_anchor_position_os(platform[idx], 0.0, 0.0, 0.0);
                a2.set_anchor_position_os(platform[nb], 0.0, 0.0, 0.0);
                IVP_Template_Spring st;
                st.spring_constant = 8.0f;
                st.spring_len = (float)spacing;
                st.spring_damp = 0.2f;
                st.anchors[0] = &a1;
                st.anchors[1] = &a2;
                IVP_Controller_Factory::create_spring(app->env, &st);
            }
            /* Front neighbor */
            if (gz + 1 < GRID_SIZE) {
                int nb = (gz + 1) * GRID_SIZE + gx;
                IVP_Template_Anchor a1, a2;
                a1.set_anchor_position_os(platform[idx], 0.0, 0.0, 0.0);
                a2.set_anchor_position_os(platform[nb], 0.0, 0.0, 0.0);
                IVP_Template_Spring st;
                st.spring_constant = 8.0f;
                st.spring_len = (float)spacing;
                st.spring_damp = 0.2f;
                st.anchors[0] = &a1;
                st.anchors[1] = &a2;
                IVP_Controller_Factory::create_spring(app->env, &st);
            }
        }
    }

    /* Pin corner boxes */
    platform[0]->set_pinned(IVP_TRUE);
    platform[GRID_SIZE - 1]->set_pinned(IVP_TRUE);
    platform[NUM_PLATFORM - GRID_SIZE]->set_pinned(IVP_TRUE);
    platform[NUM_PLATFORM - 1]->set_pinned(IVP_TRUE);

    /* Falling objects */
    IVP_Real_Object *drops[NUM_DROPS];
    for (int i = 0; i < NUM_DROPS; i++) {
        double x = (i - NUM_DROPS / 2.0) * 0.6;
        IVP_U_Point pos(x, -6.0 - i * 1.5, 0.0);
        drops[i] = ive::create_ball(app->env, &app->mat, 0.25, 1.0, &q, &pos);
    }

    /* Add pickable objects */
    for (int i = 0; i < NUM_PLATFORM; i++)
        ive::app_add_pick(app, platform[i]);
    for (int i = 0; i < NUM_DROPS; i++)
        ive::app_add_pick(app, drops[i]);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);
        ive::app_draw_grid(app);

        for (int i = 0; i < NUM_PLATFORM; i++)
            ive::draw_object_box(app->renderer, platform[i], hs, hs * 0.3, hs, ive::color::spring);

        /* Draw spring connections */
        for (int gz = 0; gz < GRID_SIZE; gz++)
            for (int gx = 0; gx < GRID_SIZE; gx++) {
                int idx = gz * GRID_SIZE + gx;
                if (gx + 1 < GRID_SIZE)
                    ive::draw_spring_line(app->renderer, platform[idx], platform[gz * GRID_SIZE + gx + 1], ive::color::highlight);
                if (gz + 1 < GRID_SIZE)
                    ive::draw_spring_line(app->renderer, platform[idx], platform[(gz + 1) * GRID_SIZE + gx], ive::color::highlight);
            }

        for (int i = 0; i < NUM_DROPS; i++)
            ive::draw_object_ball(app->renderer, drops[i], 0.25, ive::color::warning);

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Spring Platform", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            char buf[64];
            std::sprintf(buf, "Platform: %dx%d grid", GRID_SIZE, GRID_SIZE);
            ive::app_nk_label(app, buf);

            std::sprintf(buf, "Spring K: %.1f", 8.0);
            ive::app_nk_label(app, buf);

            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
