/* ive_quickstart_rope.cxx -- A chain of linked boxes forming a rope
 *
 * Based on IVP Manual section 5.9: Creating a rope.
 * Demonstrates: ball-socket constraints, nocoll_group_ident, become_pinned().
 */

#include "ive_sample_app.hxx"

#include <ivp_template_constraint.hxx>

#define NUM_LINKS 8

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Rope";
    cfg.orbit_dist = 15.0f;
    cfg.orbit_pitch = -15.0f;
    cfg.target_y = -2.0f;
    cfg.friction = 0.5;
    cfg.elasticity = 0.5;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    double lw = 0.15, lh = 0.2, ld = 0.15;
    double spacing = lh * 2.0 + 0.05;
    double start_y = -2.0;

    IVP_Real_Object *links[NUM_LINKS];

    /* Build one shared surface for all rope links */
    IVP_Compact_Surface *link_surface = ive::build_box_surface(lw, lh, ld);

    for (int i = 0; i < NUM_LINKS; i++) {
        IVP_SurfaceManager_Polygon *sm = new IVP_SurfaceManager_Polygon(link_surface);

        IVP_Template_Real_Object tro;
        ive::configure_dynamic_template(&tro, &app->mat, 0.05);
        ive::set_box_inertia(&tro, 0.05, lw, lh, ld);
        /* Manual section 5.9 specifies low damping so the rope swings freely */
        tro.speed_damp_factor = 0.001;
        tro.rot_speed_damp_factor.set(0.001, 0.001, 0.001);

        IVP_U_Point pos(0.0, start_y + i * spacing, 0.0);
        IVP_Polygon *link = app->env->create_polygon(sm, &tro, &q, &pos);
        ive::wake_and_enable(link);
        link->change_nocoll_group_ident("rope");
        links[i] = link;
    }

    /* Pin the first link */
    links[0]->set_pinned(IVP_TRUE);

    /* Ball-socket constraints between adjacent links */
    for (int i = 0; i < NUM_LINKS - 1; i++) {
        IVP_U_Point joint_pos(0.0, start_y + (i + 0.5) * spacing, 0.0);

        IVP_Template_Constraint ct;
        ct.set_ballsocket_ws(links[i], &joint_pos, links[i + 1]);
        IVP_Controller_Factory::create_constraint(app->env, &ct);
    }

    for (int i = 0; i < NUM_LINKS; i++)
        ive::app_add_pick(app, links[i]);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);

        for (int i = 0; i < NUM_LINKS; i++)
            ive::draw_object_box(app->renderer, links[i], lw, lh, ld, ive::color::object_b);

        /* Connection lines */
        for (int i = 0; i < NUM_LINKS - 1; i++)
            ive::draw_spring_line(app->renderer, links[i], links[i + 1], ive::color::spring);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Rope", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_label(app, "Links: 8 | First link pinned");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
