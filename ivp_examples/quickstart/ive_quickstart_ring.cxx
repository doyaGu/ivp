/* ive_quickstart_ring.cxx -- A concave ring falling down
 *
 * Based on IVP Manual section 5.3: Creating a concave ring.
 * Demonstrates: ledge soup builder, concave objects, IVP_U_Quat orientation.
 */

#include "ive_sample_app.hxx"

#include <ivp_surbuild_ledge_soup.hxx>
#include <ivp_template_surbuild.hxx>

/* Build a concave ring from 4 convex ledges (box bars forming a square frame) */
static IVP_Compact_Surface *build_ring_surface() {
    IVP_SurfaceBuilder_Ledge_Soup ledge_soup;

    struct BarDef { double x0, x1, y0, y1, z0, z1; };
    BarDef bars[4] = {
        {-3.0, -2.0, -0.5, 0.5, -3.0, 3.0},
        { 2.0,  3.0, -0.5, 0.5, -3.0, 3.0},
        {-2.0,  2.0, -0.5, 0.5, -3.0,-2.0},
        {-2.0,  2.0, -0.5, 0.5,  2.0, 3.0}
    };

    for (int b = 0; b < 4; b++) {
        IVP_U_Point pts[8];
        IVP_U_Vector<IVP_U_Point> points;
        int idx = 0;
        for (int sy = 0; sy <= 1; sy++)
            for (int sz = 0; sz <= 1; sz++)
                for (int sx = 0; sx <= 1; sx++) {
                    pts[idx].set(sx ? bars[b].x1 : bars[b].x0,
                                 sy ? bars[b].y1 : bars[b].y0,
                                 sz ? bars[b].z1 : bars[b].z0);
                    points.add(&pts[idx]);
                    idx++;
                }
        IVP_Compact_Ledge *ledge =
            IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&points);
        ledge_soup.insert_ledge(ledge);
    }

    IVP_Template_Surbuild_LedgeSoup tls;
    tls.build_root_convex_hull = IVP_TRUE;
    tls.merge_points = IVP_SLMP_MERGE_AND_REALLOCATE;
    return ledge_soup.compile(&tls);
}

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Ring";
    cfg.orbit_dist = 25.0f;
    cfg.target_y = -3.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    IVP_Compact_Surface *compact = build_ring_surface();
    IVP_SurfaceManager_Polygon *surman = new IVP_SurfaceManager_Polygon(compact);

    IVP_Template_Real_Object templ;
    ive::configure_dynamic_template(&templ, &app->mat, 2.0);
    ive::set_box_inertia(&templ, 2.0, 3.0, 0.5, 3.0);

    IVP_U_Quat orientation(IVP_U_Point(-0.7, 0.1, 0.1));
    IVP_U_Point pos(0.0, -8.0, 0.0);
    IVP_Polygon *ring = app->env->create_polygon(surman, &templ, &orientation, &pos);
    ive::wake_and_enable(ring);

    ive::app_add_pick(app, ring);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);
        ive::draw_object_box(app->renderer, ring, 3.0, 0.5, 3.0, ive::color::object_a);
        ive::draw_velocity_arrow(app->renderer, ring, ive::color::velocity);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Ring", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_object_info(app, "Ring", ring);
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
