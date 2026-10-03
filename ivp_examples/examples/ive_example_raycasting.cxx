/* ive_example_raycasting.cxx -- IVP_Ray_Solver demonstration
 *
 * Demonstrates both ray solver modes (Manual §6.15):
 *   IVP_Ray_Solver_Min       -- nearest hit only     (press R)
 *   IVP_Ray_Solver_Min_Hash  -- all hits along ray   (press H)
 *
 * Results are visualised:
 *   Min mode   : single hit sphere at nearest intersection
 *   Min_Hash   : spheres at every intersection, sorted by distance
 */

#include "ive_sample_app.hxx"

#include <ivp_ray_solver.hxx>
#include <SDL3/SDL.h>

#include <cstdio>
#include <cmath>

/* Persistent ray visualization */
struct HitPoint {
    float pos[3];
    float dist;
};

#define MAX_HASH_HITS 32

struct RayVis {
    bool active;
    float start[3];
    float end[3];
    bool did_hit;
    /* Min mode: single nearest hit */
    float hit_dist;
    /* Min_Hash mode: all sorted hits */
    bool   hash_mode;   /* true = Min_Hash, false = Min */
    HitPoint hash_hits[MAX_HASH_HITS];
    int     num_hash_hits;
};

#define NUM_SCENE_OBJECTS 8

int main(int, char **) {
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    cfg.title = "IVP Example: Raycasting";
    cfg.orbit_dist = 18.0f;
    cfg.orbit_pitch = -20.0f;
    cfg.target_y = -2.0f;
    cfg.friction = 0.5;
    cfg.elasticity = 0.4;

    ive::SampleApp *app = ive::app_create(&cfg);
    if (!app) return 1;

    IVP_U_Quat q; q.init();

    /* Ground */
    IVP_U_Point pg(0.0, 0.5, 0.0);
    IVP_Polygon *ground = ive::create_box(app->env, &app->mat, 15.0, 0.5, 15.0, 0.0, &q, &pg);
    ive::app_set_ground(app, ground, 15.0f, 0.5f, 15.0f);

    /* Scene objects: mix of boxes and balls at various positions */
    struct SceneObj {
        IVP_Real_Object *obj;
        bool is_ball;
        double size;
    };
    SceneObj scene[NUM_SCENE_OBJECTS];

    double hs = 0.4;
    /* Static boxes */
    {
        IVP_U_Point p(-3.0, -1.0, 0.0);
        scene[0].obj = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.0, &q, &p);
        scene[0].is_ball = false; scene[0].size = hs;
    }
    {
        IVP_U_Point p(3.0, -1.0, 0.0);
        scene[1].obj = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.0, &q, &p);
        scene[1].is_ball = false; scene[1].size = hs;
    }
    {
        IVP_U_Point p(0.0, -1.0, 3.0);
        scene[2].obj = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.0, &q, &p);
        scene[2].is_ball = false; scene[2].size = hs;
    }
    {
        IVP_U_Point p(0.0, -1.0, -3.0);
        scene[3].obj = ive::create_box(app->env, &app->mat, hs, hs, hs, 0.0, &q, &p);
        scene[3].is_ball = false; scene[3].size = hs;
    }
    /* Dynamic objects */
    {
        IVP_U_Point p(-1.5, -4.0, 1.5);
        scene[4].obj = ive::create_ball(app->env, &app->mat, 0.35, 1.0, &q, &p);
        scene[4].is_ball = true; scene[4].size = 0.35;
    }
    {
        IVP_U_Point p(1.5, -4.0, -1.5);
        scene[5].obj = ive::create_ball(app->env, &app->mat, 0.35, 1.0, &q, &p);
        scene[5].is_ball = true; scene[5].size = 0.35;
    }
    {
        IVP_U_Point p(0.0, -5.0, 0.0);
        scene[6].obj = ive::create_box(app->env, &app->mat, 0.5, 0.5, 0.5, 2.0, &q, &p);
        scene[6].is_ball = false; scene[6].size = 0.5;
    }
    {
        IVP_U_Point p(2.0, -3.5, 2.0);
        scene[7].obj = ive::create_ball(app->env, &app->mat, 0.3, 0.8, &q, &p);
        scene[7].is_ball = true; scene[7].size = 0.3;
    }

    for (int i = 0; i < NUM_SCENE_OBJECTS; i++)
        ive::app_add_pick(app, scene[i].obj);

    RayVis ray_vis;
    ray_vis.active = false;
    ray_vis.did_hit = false;
    ray_vis.hit_dist = 0.0f;
    ray_vis.hash_mode = false;
    ray_vis.num_hash_hits = 0;

    IVP_Real_Object *last_hit_obj = NULL;
    bool prev_r_key = false;
    bool prev_h_key = false;

    ive::app_save_initial_state(app);

    while (ive::app_begin_frame(app)) {
        ive::app_check_reset(app);
        /* Cast ray on R (Min mode) or H (Min_Hash mode) */
        bool do_min      = ive::key_just_pressed(SDL_SCANCODE_R, &prev_r_key);
        bool do_min_hash = ive::key_just_pressed(SDL_SCANCODE_H, &prev_h_key);

        if (do_min || do_min_hash) {
            /* Compute camera eye position from orbit parameters */
            float yaw_rad = app->cam.orbit_yaw * 3.14159265f / 180.0f;
            float pitch_rad = app->cam.orbit_pitch * 3.14159265f / 180.0f;
            float cx = app->cam.target[0] + app->cam.orbit_distance * std::cos(pitch_rad) * std::sin(yaw_rad);
            float cy = app->cam.target[1] + app->cam.orbit_distance * std::sin(pitch_rad);
            float cz = app->cam.target[2] + app->cam.orbit_distance * std::cos(pitch_rad) * std::cos(yaw_rad);

            /* Ray from eye toward target */
            float dx = app->cam.target[0] - cx;
            float dy = app->cam.target[1] - cy;
            float dz = app->cam.target[2] - cz;
            float len = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (len > 0.001f) { dx /= len; dy /= len; dz /= len; }

            float ray_length = 100.0f;

            IVP_Ray_Solver_Template templ;
            templ.ray_start_point.set(cx, cy, cz);
            templ.ray_normized_direction.set(dx, dy, dz);
            templ.ray_length = ray_length;
            templ.ray_flags = IVP_RAY_SOLVER_ALL;

            ray_vis.start[0] = cx; ray_vis.start[1] = cy; ray_vis.start[2] = cz;
            ray_vis.active   = true;
            ray_vis.hash_mode = do_min_hash;
            ray_vis.num_hash_hits = 0;
            last_hit_obj = NULL;

            if (do_min) {
                /* ── IVP_Ray_Solver_Min: nearest hit only ── */
                IVP_Ray_Solver_Min solver(&templ);
                solver.check_ray_against_all_objects_in_sim(app->env);

                IVP_Ray_Hit *hit = solver.get_ray_hit();
                if (hit) {
                    ray_vis.did_hit  = true;
                    ray_vis.hit_dist = (float)hit->hit_distance;
                    last_hit_obj = hit->hit_real_object;
                    ray_vis.end[0] = cx + dx * ray_vis.hit_dist;
                    ray_vis.end[1] = cy + dy * ray_vis.hit_dist;
                    ray_vis.end[2] = cz + dz * ray_vis.hit_dist;
                } else {
                    ray_vis.did_hit  = false;
                    ray_vis.hit_dist = 0.0f;
                    ray_vis.end[0] = cx + dx * ray_length;
                    ray_vis.end[1] = cy + dy * ray_length;
                    ray_vis.end[2] = cz + dz * ray_length;
                }
            } else {
                /* ── IVP_Ray_Solver_Min_Hash: all hits, sorted by distance ── */
                IVP_Ray_Solver_Min_Hash solver(&templ);
                solver.check_ray_against_all_objects_in_sim(app->env);

                IVP_U_Min_Hash *results = solver.get_result_min_hash();
                ray_vis.did_hit = (results->counter > 0);

                if (ray_vis.did_hit) {
                    /* Iterate all hits from nearest to farthest */
                    IVP_U_Min_Hash_Enumerator enumerator(results);
                    void *elem;
                    while ((elem = enumerator.get_next_element()) != NULL &&
                           ray_vis.num_hash_hits < MAX_HASH_HITS) {
                        IVP_Ray_Hit *hit = static_cast<IVP_Ray_Hit *>(elem);
                        HitPoint &hp = ray_vis.hash_hits[ray_vis.num_hash_hits++];
                        hp.dist   = (float)hit->hit_distance;
                        hp.pos[0] = cx + dx * hp.dist;
                        hp.pos[1] = cy + dy * hp.dist;
                        hp.pos[2] = cz + dz * hp.dist;
                        /* Use the nearest hit object for highlight */
                        if (!last_hit_obj) last_hit_obj = hit->hit_real_object;
                    }
                    /* Ray end = farthest hit */
                    HitPoint &last_hp = ray_vis.hash_hits[ray_vis.num_hash_hits - 1];
                    ray_vis.end[0] = last_hp.pos[0];
                    ray_vis.end[1] = last_hp.pos[1];
                    ray_vis.end[2] = last_hp.pos[2];
                    ray_vis.hit_dist = last_hp.dist;
                } else {
                    ray_vis.end[0] = cx + dx * ray_length;
                    ray_vis.end[1] = cy + dy * ray_length;
                    ray_vis.end[2] = cz + dz * ray_length;
                }
            }
        }

        ive::app_step(app);
        ive::app_draw_grid(app);

        /* Draw scene objects */
        for (int i = 0; i < NUM_SCENE_OBJECTS; i++) {
            const float *col;
            if (last_hit_obj && scene[i].obj == last_hit_obj)
                col = ive::color::highlight;
            else
                col = scene[i].is_ball ? ive::color::object_b : ive::color::object_a;

            if (scene[i].is_ball)
                ive::draw_object_ball(app->renderer, scene[i].obj, scene[i].size, col);
            else
                ive::draw_object_box(app->renderer, scene[i].obj,
                    scene[i].size, scene[i].size, scene[i].size, col);
        }

        /* Draw ray */
        if (ray_vis.active) {
            const float *rc = ray_vis.did_hit ? ive::color::velocity : ive::color::warning;
            ivp_draw_line(app->renderer, ray_vis.start, ray_vis.end, rc);

            if (ray_vis.did_hit) {
                if (!ray_vis.hash_mode) {
                    /* Min mode: single hit sphere */
                    ivp_draw_wire_sphere(app->renderer, ray_vis.end, 0.1f, ive::color::velocity);
                } else {
                    /* Min_Hash mode: sphere at every intersection point */
                    for (int i = 0; i < ray_vis.num_hash_hits; i++) {
                        /* Vary colour by index: nearest = green, farthest = yellow */
                        float t = (ray_vis.num_hash_hits > 1)
                            ? (float)i / (float)(ray_vis.num_hash_hits - 1) : 0.0f;
                        float col[3] = {
                            ive::color::velocity[0] * (1.0f - t) + ive::color::highlight[0] * t,
                            ive::color::velocity[1] * (1.0f - t) + ive::color::highlight[1] * t,
                            ive::color::velocity[2] * (1.0f - t) + ive::color::highlight[2] * t
                        };
                        ivp_draw_wire_sphere(app->renderer, ray_vis.hash_hits[i].pos, 0.12f, col);
                    }
                }
            }
        }

        ive::app_draw_overlays(app);

        if (ive::app_begin_info_panel(app, "Raycasting", 250, 400)) {
            ive::app_nk_fps(app);
            ive::app_nk_sim_speed(app);

            ive::app_nk_spacing(app);
            if (ray_vis.active) {
                char buf[128];
                const char *mode = ray_vis.hash_mode ? "Min_Hash (all hits)" : "Min (nearest)";
                std::sprintf(buf, "Mode: %s", mode);
                ive::app_nk_label(app, buf);
                if (ray_vis.did_hit) {
                    if (!ray_vis.hash_mode) {
                        std::sprintf(buf, "HIT  dist=%.2f", ray_vis.hit_dist);
                    } else {
                        std::sprintf(buf, "HITS: %d", ray_vis.num_hash_hits);
                        ive::app_nk_label(app, buf);
                        for (int i = 0; i < ray_vis.num_hash_hits; i++) {
                            std::sprintf(buf, "  [%d] dist=%.2f", i, ray_vis.hash_hits[i].dist);
                            ive::app_nk_label(app, buf);
                        }
                        buf[0] = '\0'; /* already printed above */
                    }
                } else {
                    std::sprintf(buf, "MISS");
                }
                if (buf[0]) ive::app_nk_label(app, buf);
            } else {
                ive::app_nk_label(app, "No ray cast yet");
            }

            ive::app_nk_controls(app,
                "R: cast ray (Min -- nearest hit)\n"
                "H: cast ray (Min_Hash -- all hits)");
        }
        ive::app_end_info_panel(app);

        ive::app_end_frame(app);
    }

    ive::app_destroy(app);
    return 0;
}
