/* ive_example_universe.cxx -- IVP_Universe_Manager demonstration
 *
 * Demonstrates the IVP_Universe_Manager API (Manual 搂6.14):
 *   - ensure_objects_in_environment(): called by the engine to request
 *     that all non-moving objects within a sphere around a dynamic object
 *     be added to the simulation so collisions can be resolved.
 *   - object_no_longer_needed(): called when a static object has no more
 *     dynamic neighbours and can be removed to save memory.
 *   - provide_universe_settings(): returns thresholds controlling how
 *     often the engine performs these checks.
 *
 * Scene: a large "world" of static boxes arranged in a grid, but only the
 * boxes close to a bouncing dynamic ball are in the simulation at any time.
 * A status display shows how many boxes are currently in/out of sim.
 *
 * Press N to drop another dynamic ball. Press D to delete the focused one.
 */

#include "ive_sample_app.hxx"

#include <ivp_universe_manager.hxx>
#include <SDL3/SDL.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

/* 鈹€鈹€ World grid of "off-sim" static boxes 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€ */

#define WORLD_GRID_W  10
#define WORLD_GRID_D  10
#define WORLD_TOTAL   (WORLD_GRID_W * WORLD_GRID_D)

struct WorldBox {
    double wx, wz;       /* world-space position */
    IVP_Real_Object *obj; /* NULL when not in sim */
};

static WorldBox g_world[WORLD_TOTAL];
static int      g_world_count = 0;
static IVP_Environment *g_env = NULL;
static IVP_Material_Simple *g_mat = NULL;

/* 鈹€鈹€ Universe Manager implementation 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€ */

class My_Universe_Manager : public IVP_Universe_Manager
{
public:
    int objects_in_sim;
    int objects_outside;

    My_Universe_Manager() : objects_in_sim(0), objects_outside(0) {}

    /*
     * The engine calls this every PSI (physics step interval) for each
     * dynamic object in the simulation. We must ensure every non-moving
     * object within 'sphere_radius' of 'sphere_center' is in the sim.
     */
    virtual void ensure_objects_in_environment(IVP_Real_Object * /*requester*/,
                                               IVP_U_Float_Point *sphere_center,
                                               IVP_DOUBLE sphere_radius)
    {
        IVP_DOUBLE r2 = sphere_radius * sphere_radius;
        for (int i = 0; i < g_world_count; i++) {
            WorldBox &wb = g_world[i];
            double dx = wb.wx - sphere_center->k[0];
            double dz = wb.wz - sphere_center->k[2];
            double dist2 = dx*dx + dz*dz;

            if (dist2 < r2) {
                if (!wb.obj) {
                    /* Box is outside sim -- add it now */
                    IVP_U_Quat q; q.init();
                    IVP_U_Point pos(wb.wx, 0.4, wb.wz);
                    double hs = 0.35;
                    wb.obj = ive::create_box(g_env, g_mat, hs, hs, hs, 0.0, &q, &pos);
                    objects_in_sim++;
                    objects_outside--;
                }
            }
        }
    }

    /*
     * The engine calls this when a static object has no more dynamic
     * neighbours and can safely be removed from the simulation.
     */
    virtual void object_no_longer_needed(IVP_Real_Object *obj)
    {
        for (int i = 0; i < g_world_count; i++) {
            if (g_world[i].obj == obj) {
                obj->delete_and_check_vicinity();
                g_world[i].obj = NULL;
                objects_in_sim--;
                objects_outside++;
                return;
            }
        }
    }

    virtual void event_object_deleted(IVP_Real_Object *obj)
    {
        /* Called just before the engine deletes the object itself.
         * Null out our pointer so we don't hold a dangling reference. */
        for (int i = 0; i < g_world_count; i++) {
            if (g_world[i].obj == obj) {
                g_world[i].obj = NULL;
                objects_in_sim--;
                objects_outside++;
                return;
            }
        }
    }

    virtual const IVP_Universe_Manager_Settings *provide_universe_settings()
    {
        static IVP_Universe_Manager_Settings s;
        /* Let the engine start checking after 10 objects are in the sim,
         * and verify up to 20 per second at second threshold. */
        s.num_objects_in_environment_threshold_0 = 10;
        s.check_objects_per_second_threshold_0   = 5;
        s.num_objects_in_environment_threshold_1 = 30;
        s.check_objects_per_second_threshold_1   = 20;
        return &s;
    }
};

static My_Universe_Manager g_universe_manager;

/* 鈹€鈹€ Dynamic ball tracking 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€ */

#define MAX_BALLS 16

struct BallEntry {
    IVP_Real_Object *obj;
    double radius;
};
static BallEntry g_balls[MAX_BALLS];
static int g_num_balls = 0;

int main(int, char **) {
    /* Build world grid of box positions (not yet in sim) */
    g_world_count = 0;
    for (int xi = 0; xi < WORLD_GRID_W; xi++) {
        for (int zi = 0; zi < WORLD_GRID_D; zi++) {
            WorldBox &wb = g_world[g_world_count++];
            wb.wx  = (xi - WORLD_GRID_W / 2) * 1.5;
            wb.wz  = (zi - WORLD_GRID_D / 2) * 1.5;
            wb.obj = NULL;
        }
    }
    g_universe_manager.objects_in_sim  = 0;
    g_universe_manager.objects_outside = g_world_count;

    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Example: Universe Manager";
    cfg.orbit_dist  = 28.0f;
    cfg.orbit_pitch = -35.0f;
    cfg.target_y    = -1.0f;
    cfg.friction    = 0.5;
    cfg.elasticity  = 0.6;
    cfg.universe_manager = &g_universe_manager;  /* Install before env creation */

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    g_env = app->env;
    IVP_Material_Simple mat(cfg.friction, cfg.elasticity);
    g_mat = &mat;

    IVP_U_Quat q; q.init();

    /* One large static ground plane always in sim */
    IVP_U_Point pg(0.0, 1.0, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &mat, 20.0, 0.5, 20.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 20.0f, 0.5f, 20.0f);

    /* Spawn one initial dynamic ball */
    {
        IVP_U_Point pos(0.0, -4.0, 0.0);
        BallEntry e;
        e.radius = 0.45;
        e.obj = ive::create_ball(app->env, &mat, e.radius, 2.0, &q, &pos);
        g_balls[g_num_balls++] = e;
        ive::app_add_pick(app, e.obj);
    }

    ive::app_save_initial_state(app);

    bool prev_n = false, prev_d = false;
    unsigned int seed = 54321;

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);

        /* Spawn a new ball (N key) */
        if (ive::key_just_pressed(SDL_SCANCODE_N, &prev_n) && g_num_balls < MAX_BALLS) {
            seed = seed * 1103515245u + 12345u;
            double x = ((int)((seed >> 16) % 100) - 50) * 0.05;
            seed = seed * 1103515245u + 12345u;
            double z = ((int)((seed >> 16) % 100) - 50) * 0.05;
            IVP_U_Point pos(x, -6.0, z);
            BallEntry e;
            e.radius = 0.3 + ((seed >> 20) % 20) * 0.01;
            e.obj = ive::create_ball(app->env, &mat, e.radius, 1.5, &q, &pos);
            g_balls[g_num_balls++] = e;
            ive::app_add_pick(app, e.obj);
        }

        /* Delete focused ball (D key) */
        if (ive::key_just_pressed(SDL_SCANCODE_D, &prev_d) && app->focus.focused) {
            for (int i = 0; i < g_num_balls; i++) {
                if (g_balls[i].obj == app->focus.focused) {
                    ive::app_delete_object(app, g_balls[i].obj);
                    for (int j = i; j < g_num_balls - 1; j++)
                        g_balls[j] = g_balls[j + 1];
                    g_num_balls--;
                    break;
                }
            }
        }

        ive::app_step(app);
        ive::app_draw_grid(app);

        /* Draw world boxes currently in sim */
        for (int i = 0; i < g_world_count; i++) {
            if (g_world[i].obj) {
                double hs = 0.35;
                ive::draw_object_box(app->renderer, g_world[i].obj,
                    hs, hs, hs, ive::color::object_a);
            }
        }

        /* Draw dynamic balls */
        for (int i = 0; i < g_num_balls; i++) {
            ive::draw_object_ball(app->renderer, g_balls[i].obj,
                g_balls[i].radius, ive::color::object_b);
        }

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Universe Manager", 270, 440)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            ive::app_nk_spacing(app);

            char buf[128];
            std::sprintf(buf, "World boxes total: %d", g_world_count);
            ive::app_nk_label(app, buf);
            std::sprintf(buf, "  In simulation:  %d", g_universe_manager.objects_in_sim);
            ive::app_nk_label(app, buf);
            std::sprintf(buf, "  Outside sim:    %d", g_universe_manager.objects_outside);
            ive::app_nk_label(app, buf);
            std::sprintf(buf, "Dynamic balls:    %d", g_num_balls);
            ive::app_nk_label(app, buf);

            ive::app_nk_spacing(app);
            ive::app_nk_label(app, "IVP_Universe_Manager API (Sec 6.14):");
            ive::app_nk_label(app, "ensure_objects_in_environment()");
            ive::app_nk_label(app, "object_no_longer_needed()");
            ive::app_nk_label(app, "provide_universe_settings()");

            ive::app_nk_controls(app,
                "N: drop a ball\n"
                "D: delete focused ball");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
