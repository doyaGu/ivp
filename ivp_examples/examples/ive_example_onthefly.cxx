/* ive_example_onthefly.cxx -- Runtime property changes
 *
 * Cubes on a ground plane. Keyboard toggles mass, friction, spring.
 * 1-5: set mass, G: spring toggle, R: reset.
 */

#include "ive_sample_app.hxx"

#include <ivp_actuator_spring.hxx>
#include <SDL3/SDL.h>

#include <cstdio>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title       = "IVP Example: On The Fly";
    cfg.orbit_dist  = 18.0f;
    cfg.orbit_pitch = -25.0f;
    cfg.target_y    = -2.0f;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    /* Two custom materials -- do not use app->mat */
    IVP_Material_Simple mat_lo(0.3, 0.2);
    IVP_Material_Simple mat_hi(0.9, 0.8);

    IVP_U_Quat q; q.init();
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &mat_hi, 10.0, 0.5, 10.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 10.0f, 0.5f, 10.0f);

    double hs = 0.4;
    IVP_U_Point p1(-1.5, -3.0, 0.0);
    IVP_U_Point p2( 1.5, -3.0, 0.0);
    IVP_Polygon *cube1 = ive::create_box(app->env, &mat_lo, hs, hs, hs, 1.0, &q, &p1);
    IVP_Polygon *cube2 = ive::create_box(app->env, &mat_lo, hs, hs, hs, 1.0, &q, &p2);

    /* Optional spring */
    IVP_Template_Anchor a1, a2;
    a1.set_anchor_position_os(cube1, 0.0, 0.0, 0.0);
    a2.set_anchor_position_os(cube2, 0.0, 0.0, 0.0);
    IVP_Template_Spring st;
    st.spring_constant = 5.0f;
    st.spring_len = 1.0f;
    st.spring_damp = 0.1f;
    st.anchors[0] = &a1;
    st.anchors[1] = &a2;
    IVP_Actuator_Spring *spring = IVP_Controller_Factory::create_spring(app->env, &st);

    bool spring_on = true;
    float current_mass = 1.0f;

    ive::app_add_pick(app, cube1);
    ive::app_add_pick(app, cube2);

    bool prev_s = false, prev_r = false;
    bool prev_1 = false, prev_2 = false, prev_3 = false, prev_4 = false, prev_5 = false;

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        /* Mass changes */
        if (ive::key_just_pressed(SDL_SCANCODE_1, &prev_1)) { current_mass = 0.5f; cube1->change_mass(0.5); cube2->change_mass(0.5); }
        if (ive::key_just_pressed(SDL_SCANCODE_2, &prev_2)) { current_mass = 1.0f; cube1->change_mass(1.0); cube2->change_mass(1.0); }
        if (ive::key_just_pressed(SDL_SCANCODE_3, &prev_3)) { current_mass = 2.0f; cube1->change_mass(2.0); cube2->change_mass(2.0); }
        if (ive::key_just_pressed(SDL_SCANCODE_4, &prev_4)) { current_mass = 5.0f; cube1->change_mass(5.0); cube2->change_mass(5.0); }
        if (ive::key_just_pressed(SDL_SCANCODE_5, &prev_5)) { current_mass = 10.0f; cube1->change_mass(10.0); cube2->change_mass(10.0); }

        /* Toggle spring */
        if (ive::key_just_pressed(SDL_SCANCODE_G, &prev_s)) {
            spring_on = !spring_on;
            if (spring) spring->set_constant(spring_on ? 5.0f : 0.0f);
        }

        /* Reset velocities -- async_add_speed_object_ws is *additive*, so we
         * must negate the current core speed to bring the velocity to zero. */
        if (ive::key_just_pressed(SDL_SCANCODE_R, &prev_r)) {
            IVP_Core *core1 = cube1->get_core();
            IVP_Core *core2 = cube2->get_core();
            IVP_U_Float_Point neg1, neg2;
            neg1.set(-core1->speed.k[0], -core1->speed.k[1], -core1->speed.k[2]);
            neg2.set(-core2->speed.k[0], -core2->speed.k[1], -core2->speed.k[2]);
            cube1->async_add_speed_object_ws(&neg1);
            cube2->async_add_speed_object_ws(&neg2);
            /* Also zero rotational speed */
            IVP_U_Float_Point zrot(0.0f, 0.0f, 0.0f);
            IVP_U_Float_Point nrot1, nrot2;
            nrot1.set(-core1->rot_speed.k[0], -core1->rot_speed.k[1], -core1->rot_speed.k[2]);
            nrot2.set(-core2->rot_speed.k[0], -core2->rot_speed.k[1], -core2->rot_speed.k[2]);
            cube1->async_add_rot_speed_object_cs(&nrot1);
            cube2->async_add_rot_speed_object_cs(&nrot2);
        }

        ive::app_step(app);
        ive::app_draw_grid(app);

        ive::draw_object_box(app->renderer, cube1, hs, hs, hs, ive::color::object_b);
        ive::draw_object_box(app->renderer, cube2, hs, hs, hs, ive::color::object_a);
        if (spring_on)
            ive::draw_spring_line(app->renderer, cube1, cube2, ive::color::highlight);

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "On The Fly", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            char buf[128];
            std::sprintf(buf, "Mass: %.1f", (double)current_mass);
            ive::app_nk_label(app, buf);

            std::sprintf(buf, "Spring: %s", spring_on ? "ON" : "OFF");
            ive::app_nk_label(app, buf);

            ive::app_nk_controls(app,
                "1-5: set mass\n"
                "G: toggle spring\n"
                "R: reset velocities");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
