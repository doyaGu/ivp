/* ive_quickstart_cut.cxx -- Chopping an object with IVP_Compact_Modify::chop
 *
 * Based on IVP Manual section 5.15: Cutting an object.
 * Demonstrates: IVP_Compact_Modify::chop(), creating two pieces from one.
 */

#include "ive_sample_app.hxx"

#include <ivp_compact_modify.hxx>

#include <cstdio>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Cut";
    cfg.orbit_dist = 15.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    /* Build original box surface */
    IVP_Compact_Surface *original = ive::build_box_surface(0.6, 0.6, 0.6);

    /* Chop it: cut along X axis, remove 0.2m */
    IVP_U_Float_Point chop_dir(1.0, 0.0, 0.0);
    IVP_Compact_Surface *piece1 = IVP_Compact_Modify::chop(original, &chop_dir, 0.2f);

    /* Chop the other way for piece2 */
    IVP_U_Float_Point chop_dir2(-1.0, 0.0, 0.0);
    IVP_Compact_Surface *piece2 = IVP_Compact_Modify::chop(original, &chop_dir2, 0.2f);

    /* Create objects from the two pieces */
    IVP_SurfaceManager_Polygon *sm1 = new IVP_SurfaceManager_Polygon(piece1 ? piece1 : original);
    IVP_Template_Real_Object t1;
    ive::configure_dynamic_template(&t1, &app->mat, 1.0);
    ive::set_box_inertia(&t1, 1.0, 0.4, 0.6, 0.6);
    IVP_U_Point pos1(-0.8, -5.0, 0.0);
    IVP_Polygon *obj1 = app->env->create_polygon(sm1, &t1, &q, &pos1);
    ive::wake_and_enable(obj1);

    IVP_SurfaceManager_Polygon *sm2 = new IVP_SurfaceManager_Polygon(piece2 ? piece2 : original);
    IVP_Template_Real_Object t2;
    ive::configure_dynamic_template(&t2, &app->mat, 1.0);
    ive::set_box_inertia(&t2, 1.0, 0.4, 0.6, 0.6);
    IVP_U_Point pos2(0.8, -5.0, 0.0);
    IVP_Polygon *obj2 = app->env->create_polygon(sm2, &t2, &q, &pos2);
    ive::wake_and_enable(obj2);

    ive::app_add_pick(app, obj1);
    ive::app_add_pick(app, obj2);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, obj1, 0.4, 0.6, 0.6, ive::color::object_b);
        ive::draw_object_box(app->renderer, obj2, 0.4, 0.6, 0.6, ive::color::object_a);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Cut (Chop)", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            ive::app_nk_object_info(app, "Piece 1 (orange)", obj1);
            ive::app_nk_object_info(app, "Piece 2 (blue)", obj2);

            ive::app_nk_label(app, "Original box chopped into two pieces");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
