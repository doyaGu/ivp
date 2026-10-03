/* ive_example_domino.cxx -- Chain of falling domino tiles
 *
 * A row of thin tall boxes. The first is tipped, causing a cascade.
 */

#include "ive_sample_app.hxx"

#include <cstdio>
#include <cmath>

#define NUM_TILES 25

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title       = "IVP Example: Domino";
    cfg.orbit_dist  = 25.0f;
    cfg.orbit_pitch = -20.0f;
    cfg.target_x    = 5.0f;
    cfg.target_y    = -1.0f;
    cfg.friction    = 0.6;
    cfg.elasticity  = 0.5;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    /* Ground */
    IVP_U_Quat q; q.init();
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 20.0, 0.5, 5.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 20.0f, 0.5f, 5.0f);

    /* Domino tiles: thin (0.1) x tall (0.6) x wide (0.3) */
    double tw = 0.1, th = 0.6, td = 0.3;
    double spacing = 0.55;

    IVP_Real_Object *tiles[NUM_TILES];
    for (int i = 0; i < NUM_TILES; i++) {
        IVP_U_Point pos(-5.0 + i * spacing, -th, 0.0);
        tiles[i] = ive::create_box(app->env, &app->mat, tw, th, td, 0.3, &q, &pos);
    }

    /* Tip the first tile slightly */
    IVP_U_Float_Point angular_vel(0.0, 0.0, -2.0);
    tiles[0]->async_add_rot_speed_object_cs(&angular_vel);

    /* Add pickable objects */
    for (int i = 0; i < NUM_TILES; i++)
        ive::app_add_pick(app, tiles[i]);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);
        ive::app_draw_grid(app);

        for (int i = 0; i < NUM_TILES; i++)
            ive::draw_object_box(app->renderer, tiles[i], tw, th, td, ive::color::highlight);

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Domino Chain", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            char buf[64];
            std::sprintf(buf, "Tiles: %d", NUM_TILES);
            ive::app_nk_label(app, buf);

            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
