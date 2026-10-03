/* ive_example_black_hole.cxx -- Custom gravity controller (radial attractor)
 *
 * ~200 objects are attracted toward a central "black hole" point by a custom
 * IVP_Controller subclass that applies radial forces each simulation step.
 */

#include "ive_sample_app.hxx"

#include <ivp_controller.hxx>
#include <cstdio>
#include <cmath>

#define NUM_OBJECTS 120

class Black_Hole_Controller : public IVP_Controller_Independent {
    IVP_Environment *m_env;
    IVP_Real_Object *m_objects[NUM_OBJECTS];
    int m_count;
    IVP_U_Point m_center;
    double m_strength;

public:
    Black_Hole_Controller(IVP_Environment *env, IVP_U_Point center, double strength)
        : m_env(env), m_count(0), m_center(center), m_strength(strength) {}

    void add_object(IVP_Real_Object *obj) {
        if (m_count < NUM_OBJECTS) m_objects[m_count++] = obj;
    }

    void do_simulation_controller(IVP_Event_Sim *es, IVP_U_Vector<IVP_Core> *) {
        double dt = es->delta_time;
        for (int i = 0; i < m_count; i++) {
            IVP_Core *core = m_objects[i]->get_core();
            if (core->physical_unmoveable) continue;
            const IVP_U_Point *pos = core->get_position_PSI();
            IVP_U_Point dir;
            dir.subtract(&m_center, pos);
            double dist = dir.real_length();
            if (dist < 0.5) continue;
            double force_mag = m_strength / (dist * dist);
            if (force_mag > 50.0) force_mag = 50.0;
            dir.mult(force_mag * dt / dist);
            IVP_U_Float_Point impulse;
            impulse.set(dir.k[0], dir.k[1], dir.k[2]);
            m_objects[i]->async_add_speed_object_ws(&impulse);
        }
    }

    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_ACTUATOR; }
    void core_is_going_to_be_deleted_event(IVP_Core *) {}
};

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title       = "IVP Example: Black Hole";
    cfg.orbit_dist  = 30.0f;
    cfg.target_y    = 0.0f;
    cfg.friction    = 0.5;
    cfg.elasticity  = 0.3;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Point center(0.0, 0.0, 0.0);
    Black_Hole_Controller *bhc = new Black_Hole_Controller(app->env, center, 80.0);

    IVP_Real_Object *objects[NUM_OBJECTS];
    double hs = 0.2;
    IVP_U_Quat q; q.init();

    for (int i = 0; i < NUM_OBJECTS; i++) {
        double angle = i * 2.3998;
        double radius = 5.0 + (i % 10) * 0.8;
        double x = radius * std::cos(angle);
        double z = radius * std::sin(angle);
        double y = -3.0 + (i % 7) * 1.0;
        IVP_U_Point pos(x, y, z);

        if (i % 3 == 0) {
            objects[i] = ive::create_ball(app->env, &app->mat, hs, 0.5, &q, &pos);
        } else {
            objects[i] = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.5, &q, &pos);
        }
        bhc->add_object(objects[i]);
    }

    /* Register controller with each core */
    for (int i = 0; i < NUM_OBJECTS; i++)
        app->env->get_controller_manager()->add_controller_to_core(bhc, objects[i]->get_core());

    /* Add pickable objects (limit to 64) */
    for (int i = 0; i < NUM_OBJECTS && i < 64; i++)
        ive::app_add_pick(app, objects[i]);

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);
        ive::app_draw_grid(app);

        /* Draw center sphere */
        float cp[3] = {0.0f, 0.0f, 0.0f};
        ivp_draw_wire_sphere(app->renderer, cp, 0.5f, ive::color::warning);

        for (int i = 0; i < NUM_OBJECTS; i++) {
            if (i % 3 == 0)
                ive::draw_object_ball(app->renderer, objects[i], hs, ive::color::object_b);
            else
                ive::draw_object_box(app->renderer, objects[i], hs, hs, hs, ive::color::object_a);
        }

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Black Hole", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            char buf[64];
            std::sprintf(buf, "Objects: %d", NUM_OBJECTS);
            ive::app_nk_label(app, buf);

            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
