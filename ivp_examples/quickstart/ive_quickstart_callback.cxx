/* ive_quickstart_callback.cxx -- Object event and collision listeners
 *
 * Based on IVP Manual section 5.12: Interfacing with callbacks.
 * Demonstrates: IVP_Listener_Object (freeze/revive), IVP_Listener_Collision.
 *
 * A falling cube: as soon as the cube comes to a standstill (frozen),
 * it is pushed back up into the air -- exactly as described in §5.12.
 * Additional cubes show colour-coded collision state.
 */

#include "ive_sample_app.hxx"

#include <ivp_listener_object.hxx>
#include <ivp_listener_collision.hxx>

#include <cstdio>
#include <cstring>

/* Per-object color state: 0=green(moving), 1=blue(frozen), 2=red(colliding) */
static int obj_state[8];
static float collision_flash[8];

/* Index 0 is the 'main' cube that relaunches when frozen */
static IVP_Real_Object *g_main_cube = NULL;

class My_Object_Listener : public IVP_Listener_Object {
public:
    int idx;
    bool relaunch; /* if true, push the object upward when it freezes */
    My_Object_Listener(int i, bool rl = false) : idx(i), relaunch(rl) {}
    void event_object_created(IVP_Event_Object *) {}
    void event_object_deleted(IVP_Event_Object *) {}
    void event_object_frozen(IVP_Event_Object *ev) {
        obj_state[idx] = 1;
        if (relaunch && g_main_cube) {
            /* Push the cube upward (negative Y = up in IVP gravity direction) */
            IVP_U_Float_Point impulse(0.0f, -6.0f, 0.0f);
            g_main_cube->async_add_speed_object_ws(&impulse);
        }
    }
    void event_object_revived(IVP_Event_Object *) { obj_state[idx] = 0; }
};

class My_Collision_Listener : public IVP_Listener_Collision {
public:
    int idx;
    My_Collision_Listener(int i) :
        IVP_Listener_Collision(IVP_LISTENER_COLLISION_CALLBACK_POST_COLLISION),
        idx(i) {}
    void event_collision(IVP_Event_Collision *) {
        collision_flash[idx] = 0.3f;
    }
};

static void state_color(int idx, float dt, float out[3]) {
    if (collision_flash[idx] > 0.0f) {
        collision_flash[idx] -= dt;
        out[0] = ive::color::warning[0];
        out[1] = ive::color::warning[1];
        out[2] = ive::color::warning[2];
    } else if (obj_state[idx] == 1) {
        out[0] = ive::color::object_a[0];
        out[1] = ive::color::object_a[1];
        out[2] = ive::color::object_a[2];
    } else {
        out[0] = ive::color::highlight[0];
        out[1] = ive::color::highlight[1];
        out[2] = ive::color::highlight[2];
    }
}

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Quickstart: Callbacks";
    cfg.orbit_dist = 20.0f;
    cfg.target_y = -3.0f;
    cfg.friction = 0.8;
    cfg.elasticity = 0.5;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    /* Ground */
    IVP_U_Quat q; q.init();
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 10.0, 0.5, 10.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 10.0f, 0.5f, 10.0f);

    /* Several dynamic cubes.
     * Cube 0 is the 'main' cube that relaunches when it freezes (§5.12).   */
    double hs = 0.4;
    IVP_Real_Object *cubes[4];
    for (int i = 0; i < 4; i++) {
        obj_state[i] = 0;
        collision_flash[i] = 0.0f;

        IVP_U_Point pos(-3.0 + i * 2.0, -3.0 - i * 1.5, 0.0);
        cubes[i] = ive::create_box(app->env, &app->mat, hs, hs, hs, 1.0, &q, &pos);

        bool is_main = (i == 0);
        if (is_main) g_main_cube = cubes[i];

        My_Object_Listener *ol = new My_Object_Listener(i, is_main);
        cubes[i]->add_listener_object(ol);

        My_Collision_Listener *cl = new My_Collision_Listener(i);
        cubes[i]->add_listener_collision(cl);

        ive::app_add_pick(app, cubes[i]);
    }

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        ive::app_step(app);

        /* Draw */
        ive::app_draw_grid(app);

        for (int i = 0; i < 4; i++) {
            float col[3];
            state_color(i, app->dt, col);
            ive::draw_object_box(app->renderer, cubes[i], hs, hs, hs, col);
        }

        /* Overlays */
        ive::app_draw_overlays(app);

        /* Info panel */
        if (ive::app_begin_info_panel(app, "Callbacks", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            for (int i = 0; i < 4; i++) {
                char label[32];
                std::sprintf(label, "Cube %d", i);
                ive::app_nk_object_info(app, label, cubes[i]);
            }

            ive::app_nk_label(app, "Green=moving  Blue=frozen  Red=collision");
            ive::app_nk_label(app, "Cube 0: relaunched when frozen (Sec 5.12)");
            ive::app_nk_controls(app);
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
