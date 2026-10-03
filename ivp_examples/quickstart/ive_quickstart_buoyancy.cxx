/* ive_quickstart_buoyancy.cxx -- Fluid dynamics / buoyancy
 *
 * Based on IVP Manual section 5.18: Using fluid dynamics.
 * Demonstrates: IVP_Template_Buoyancy, IVP_Liquid_Surface_Descriptor_Simple,
 *               IVP_Attacher_To_Cores_Buoyancy, IVP_U_Set_Active.
 * Objects fall into water and float based on density.
 */

#include "ive_sample_app.hxx"

#include <ivp_liquid_surface_descript.hxx>
#include <ivp_controller_buoyancy.hxx>
#include <ivu_set.hxx>

#include <cstdio>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Buoyancy";
    cfg.orbit_dist = 20.0f;
    cfg.target_y = -3.0f;
    cfg.friction = 0.5;
    cfg.elasticity = 0.5;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Set_Active<IVP_Core> *floating_cores = new IVP_U_Set_Active<IVP_Core>(64);

    IVP_U_Quat q; q.init();

    /* Light cube (cork-like) - mass 0.2 for 0.6^3 volume ~ density 926 */
    double hs = 0.3;
    IVP_U_Point p1(-2.0, -8.0, 0.0);
    IVP_Polygon *light_cube = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.2, &q, &p1);
    floating_cores->add_element(light_cube->get_core());

    /* Medium cube (wood-like) - mass 0.5 */
    IVP_U_Point p2(0.0, -8.0, 0.0);
    IVP_Polygon *med_cube = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.5, &q, &p2);
    floating_cores->add_element(med_cube->get_core());

    /* Heavy ball (steel-like) - mass 5 */
    IVP_U_Point p3(2.0, -8.0, 0.0);
    IVP_Ball *heavy_ball = ive::create_ball(app->env, &app->mat, 0.4, 5.0, &q, &p3);
    floating_cores->add_element(heavy_ball->get_core());

    /* Water surface at y=0 (normal pointing up = negative Y in IVP) */
    IVP_U_Float_Hesse water_surface(0.0f, 1.0f, 0.0f, 0.0f);
    IVP_U_Float_Point current_speed(0.0f, 0.0f, 0.0f);
    IVP_Liquid_Surface_Descriptor_Simple *lsd =
        new IVP_Liquid_Surface_Descriptor_Simple(&water_surface, &current_speed);

    /* Buoyancy parameters */
    IVP_Template_Buoyancy buoy;
    buoy.medium_density = 998.0f;
    buoy.pressure_damp_factor = 1.0f;
    buoy.viscosity_factor = 0.01f;
    buoy.torque_factor = 0.0f;
    buoy.viscosity_input_factor = 0.1f;

    /* Attach buoyancy solver */
    new IVP_Attacher_To_Cores_Buoyancy(buoy, floating_cores, lsd);

    ive::app_add_pick(app, light_cube);
    ive::app_add_pick(app, med_cube);
    ive::app_add_pick(app, heavy_ball);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);

        /* Draw water surface as a quad */
        float water_corners[4][3] = {
            {-6.0f, 0.0f, -6.0f},
            { 6.0f, 0.0f, -6.0f},
            { 6.0f, 0.0f,  6.0f},
            {-6.0f, 0.0f,  6.0f}
        };
        ivp_draw_wire_quad(app->renderer, water_corners, ive::color::water);

        ive::draw_object_box(app->renderer, light_cube, hs, hs, hs, ive::color::highlight);
        ive::draw_object_box(app->renderer, med_cube, hs, hs, hs, ive::color::object_b);
        ive::draw_object_ball(app->renderer, heavy_ball, 0.4, ive::color::static_obj);

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Buoyancy", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            ive::app_nk_object_info(app, "Light (cork)", light_cube);
            ive::app_nk_object_info(app, "Medium (wood)", med_cube);
            ive::app_nk_object_info(app, "Heavy (steel)", heavy_ball);

            ive::app_nk_label(app, "Light=cork  Medium=wood  Heavy=steel");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
