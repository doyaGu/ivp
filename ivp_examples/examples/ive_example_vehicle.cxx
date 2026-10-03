/* ive_example_vehicle.cxx -- Simple vehicle with spring suspension
 *
 * Body box with 4 wheel balls connected by springs.
 * Up/Down arrows for forward/backward thrust, Left/Right for turning.
 */

#include "ive_sample_app.hxx"

#include <ivp_actuator_spring.hxx>
#include <SDL3/SDL.h>

#include <cstdio>
#include <cmath>

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Example: Vehicle";
    cfg.orbit_dist = 20.0f;
    cfg.orbit_pitch = -25.0f;
    cfg.target_y = -2.0f;
    cfg.friction = 0.5;
    cfg.elasticity = 0.4;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    /* Local materials for ground and wheels */
    IVP_Material_Simple mat_ground(0.8, 0.7);
    IVP_Material_Simple mat_wheel(0.9, 0.8);

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &mat_ground, 20.0, 0.5, 20.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 20.0f, 0.5f, 20.0f);

    /* Body */
    double bx = 1.0, by = 0.3, bz = 0.5;
    IVP_U_Point pb(0.0, -1.5, 0.0);
    IVP_Polygon *body = ive::create_box(app->env, &app->mat, bx, by, bz, 8.0, &q, &pb);

    /* 4 Wheels */
    double wheel_r = 0.25;
    IVP_U_Point wpos[4];
    wpos[0].set( bx,  -1.5 - by - 0.3,  bz);   /* front-right */
    wpos[1].set( bx,  -1.5 - by - 0.3, -bz);   /* front-left */
    wpos[2].set(-bx,  -1.5 - by - 0.3,  bz);   /* rear-right */
    wpos[3].set(-bx,  -1.5 - by - 0.3, -bz);   /* rear-left */

    IVP_Ball *wheels[4];
    for (int i = 0; i < 4; i++)
        wheels[i] = ive::create_ball(app->env, &mat_wheel, wheel_r, 0.5, &q, &wpos[i]);

    /* Connect wheels to body with springs (suspension) */
    IVP_Actuator_Spring *springs[4];
    double anchor_offsets[4][3] = {
        { bx, by, bz}, { bx, by, -bz},
        {-bx, by, bz}, {-bx, by, -bz}
    };
    for (int i = 0; i < 4; i++) {
        IVP_Template_Anchor a1, a2;
        a1.set_anchor_position_os(body,
            anchor_offsets[i][0], anchor_offsets[i][1], anchor_offsets[i][2]);
        a2.set_anchor_position_os(wheels[i], 0.0, 0.0, 0.0);

        IVP_Template_Spring st;
        st.spring_constant = 30.0f;
        st.spring_len = 0.3f;
        st.spring_damp = 2.0f;
        st.anchors[0] = &a1;
        st.anchors[1] = &a2;
        springs[i] = IVP_Controller_Factory::create_spring(app->env, &st);
    }

    ive::app_add_pick(app, body);
    for (int i = 0; i < 4; i++) ive::app_add_pick(app, wheels[i]);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        /* Vehicle controls (continuous key_pressed, before step) */
        float thrust = 8.0f;
        float turn_torque = 3.0f;

        if (ive::key_pressed(SDL_SCANCODE_UP)) {
            IVP_U_Float_Point fwd(0.0, 0.0, 0.0);
            IVP_Core *core = body->get_core();
            const IVP_U_Matrix *m = core->get_m_world_f_core_PSI();
            fwd.set(m->get_elem(0, 0) * thrust * app->dt,
                    0.0,
                    m->get_elem(2, 0) * thrust * app->dt);
            body->async_add_speed_object_ws(&fwd);
        }
        if (ive::key_pressed(SDL_SCANCODE_DOWN)) {
            IVP_Core *core = body->get_core();
            const IVP_U_Matrix *m = core->get_m_world_f_core_PSI();
            IVP_U_Float_Point bwd(
                -m->get_elem(0, 0) * thrust * app->dt,
                0.0,
                -m->get_elem(2, 0) * thrust * app->dt);
            body->async_add_speed_object_ws(&bwd);
        }
        if (ive::key_pressed(SDL_SCANCODE_LEFT)) {
            IVP_U_Float_Point torque(0.0, -turn_torque * app->dt, 0.0);
            body->async_add_rot_speed_object_cs(&torque);
        }
        if (ive::key_pressed(SDL_SCANCODE_RIGHT)) {
            IVP_U_Float_Point torque(0.0, turn_torque * app->dt, 0.0);
            body->async_add_rot_speed_object_cs(&torque);
        }

        ive::app_step(app);
        ive::app_draw_grid(app);

        ive::draw_object_box(app->renderer, body, bx, by, bz, ive::color::object_a);

        for (int i = 0; i < 4; i++) {
            ive::draw_object_ball(app->renderer, wheels[i], wheel_r, ive::color::highlight);
            ive::draw_spring_line(app->renderer, body, wheels[i], ive::color::spring);
        }

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Vehicle", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);
            ive::app_nk_controls(app,
                "Up/Down: thrust\n"
                "Left/Right: turn");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
