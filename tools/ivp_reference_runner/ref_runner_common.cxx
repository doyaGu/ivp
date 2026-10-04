#include "ref_runner.hxx"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

namespace ref_runner {

SceneObjects::SceneObjects() : count(0) {
    for (int i = 0; i < 128; ++i) {
        objects[i] = 0;
        types[i] = 0;
    }
}

ScenarioResources::ScenarioResources()
    : buoyancy_cores(0), buoyancy_attacher(0), liquid_desc(0),
      step_hook(0), cleanup_hook(0), scenario_data(0) {}

void ScenarioResources::cleanup() {
    if (cleanup_hook) {
        cleanup_hook(this);
        cleanup_hook = 0;
    }
    step_hook = 0;
    scenario_data = 0;
    delete buoyancy_cores;
    buoyancy_cores = 0;
    delete liquid_desc;
    liquid_desc = 0;
    buoyancy_attacher = 0;
}

bool ref_debug_enabled() {
    const char *v = std::getenv("IVP_REF_DEBUG");
    return v && *v;
}

void print_usage(const char *exe) {
    std::fprintf(
        stderr,
        "Usage: %s --scenario freefall|cubes|springs|rope|buoyancy|motor|force_actuator|forcefield|stiff_spring|check_distance|collision_filter|motion_controller|phantom|vehicle|car_real_wheels|concave_static|compound_dynamic|convex_hulls|grid_terrain|hull_pile|galton|fast_impacts|spawn_remove|long_range|two_balls|slope_friction|raycast_car_drive|check_dist_events|golem_beam|universe_evict|merge_objects|object_attach|merge_buoyancy|anchor_follow [--steps N] [--dt seconds]\\n"
        "Outputs JSONL snapshots to stdout.\\n",
        exe);
}

static bool parse_int(const char *s, int *out) {
    if (!s || !*s) return false;
    char *end = 0;
    long v = std::strtol(s, &end, 10);
    if (!end || *end != '\0') return false;
    *out = (int)v;
    return true;
}

static bool parse_double(const char *s, double *out) {
    if (!s || !*s) return false;
    char *end = 0;
    double v = std::strtod(s, &end);
    if (!end || *end != '\0') return false;
    *out = v;
    return true;
}

static Scenario parse_scenario(const char *s, bool *ok) {
    *ok = true;
    if (!std::strcmp(s, "freefall")) return SCENARIO_FREEFALL;
    if (!std::strcmp(s, "cubes")) return SCENARIO_CUBES;
    if (!std::strcmp(s, "springs")) return SCENARIO_SPRINGS;
    if (!std::strcmp(s, "rope")) return SCENARIO_ROPE;
    if (!std::strcmp(s, "buoyancy")) return SCENARIO_BUOYANCY;
    if (!std::strcmp(s, "motor")) return SCENARIO_MOTOR;
    if (!std::strcmp(s, "force_actuator")) return SCENARIO_FORCE_ACTUATOR;
    if (!std::strcmp(s, "forcefield")) return SCENARIO_FORCEFIELD;
    if (!std::strcmp(s, "stiff_spring")) return SCENARIO_STIFF_SPRING;
    if (!std::strcmp(s, "check_distance")) return SCENARIO_CHECK_DISTANCE;
    if (!std::strcmp(s, "collision_filter")) return SCENARIO_COLLISION_FILTER;
    if (!std::strcmp(s, "motion_controller")) return SCENARIO_MOTION_CONTROLLER;
    if (!std::strcmp(s, "phantom")) return SCENARIO_PHANTOM;
    if (!std::strcmp(s, "vehicle")) return SCENARIO_VEHICLE;
    if (!std::strcmp(s, "car_real_wheels")) return SCENARIO_CAR_REAL_WHEELS;
    if (!std::strcmp(s, "concave_static")) return SCENARIO_CONCAVE_STATIC;
    if (!std::strcmp(s, "compound_dynamic")) return SCENARIO_COMPOUND_DYNAMIC;
    if (!std::strcmp(s, "convex_hulls")) return SCENARIO_CONVEX_HULLS;
    if (!std::strcmp(s, "grid_terrain")) return SCENARIO_GRID_TERRAIN;
    if (!std::strcmp(s, "hull_pile")) return SCENARIO_HULL_PILE;
    if (!std::strcmp(s, "galton")) return SCENARIO_GALTON;
    if (!std::strcmp(s, "fast_impacts")) return SCENARIO_FAST_IMPACTS;
    if (!std::strcmp(s, "spawn_remove")) return SCENARIO_SPAWN_REMOVE;
    if (!std::strcmp(s, "long_range")) return SCENARIO_LONG_RANGE;
    if (!std::strcmp(s, "torque")) return SCENARIO_TORQUE;
    if (!std::strcmp(s, "stabilizer")) return SCENARIO_STABILIZER;
    if (!std::strcmp(s, "floating")) return SCENARIO_FLOATING;
    if (!std::strcmp(s, "world_friction")) return SCENARIO_WORLD_FRICTION;
    if (!std::strcmp(s, "golem")) return SCENARIO_GOLEM;
    if (!std::strcmp(s, "fixed_keyed")) return SCENARIO_FIXED_KEYED;
    if (!std::strcmp(s, "hinge_limits")) return SCENARIO_HINGE_LIMITS;
    if (!std::strcmp(s, "cardan_tense")) return SCENARIO_CARDAN_TENSE;
    if (!std::strcmp(s, "slider_limits")) return SCENARIO_SLIDER_LIMITS;
    if (!std::strcmp(s, "constraint_break")) return SCENARIO_CONSTRAINT_BREAK;
    if (!std::strcmp(s, "attacher")) return SCENARIO_ATTACHER;
    if (!std::strcmp(s, "actuator_extra")) return SCENARIO_ACTUATOR_EXTRA;
    if (!std::strcmp(s, "airboat")) return SCENARIO_AIRBOAT;
    if (!std::strcmp(s, "fake_jetski")) return SCENARIO_FAKE_JETSKI;
    if (!std::strcmp(s, "raycast_car_drive")) return SCENARIO_RAYCAST_CAR_DRIVE;
    if (!std::strcmp(s, "check_dist_events")) return SCENARIO_CHECK_DIST_EVENTS;
    if (!std::strcmp(s, "golem_beam")) return SCENARIO_GOLEM_BEAM;
    if (!std::strcmp(s, "universe_evict")) return SCENARIO_UNIVERSE_EVICT;
    if (!std::strcmp(s, "merge_objects")) return SCENARIO_MERGE_OBJECTS;
    if (!std::strcmp(s, "object_attach")) return SCENARIO_OBJECT_ATTACH;
    if (!std::strcmp(s, "merge_buoyancy")) return SCENARIO_MERGE_BUOYANCY;
    if (!std::strcmp(s, "anchor_follow")) return SCENARIO_ANCHOR_FOLLOW;

    if (!std::strcmp(s, "two_balls")) return SCENARIO_TWO_BALLS;
    if (!std::strcmp(s, "slope_friction")) return SCENARIO_SLOPE_FRICTION;
    *ok = false;
    return SCENARIO_FREEFALL;
}

bool parse_args(int argc, char **argv, RunnerConfig *cfg) {
    for (int i = 1; i < argc; ++i) {
        const char *arg = argv[i];
        if (!std::strcmp(arg, "--help") || !std::strcmp(arg, "-h")) {
            print_usage(argv[0]);
            return false;
        }
        if (!std::strcmp(arg, "--scenario")) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return false;
            }
            bool ok = false;
            cfg->scenario = parse_scenario(argv[++i], &ok);
            if (!ok) {
                std::fprintf(stderr, "Unknown scenario: %s\\n", argv[i]);
                return false;
            }
            continue;
        }
        if (!std::strcmp(arg, "--steps")) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return false;
            }
            int v = 0;
            if (!parse_int(argv[++i], &v) || v < 0) {
                std::fprintf(stderr, "Invalid --steps\\n");
                return false;
            }
            cfg->steps = v;
            continue;
        }
        if (!std::strcmp(arg, "--dt")) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return false;
            }
            double v = 0.0;
            if (!parse_double(argv[++i], &v) || v <= 0.0) {
                std::fprintf(stderr, "Invalid --dt\\n");
                return false;
            }
            cfg->dt = v;
            continue;
        }

        std::fprintf(stderr, "Unknown arg: %s\\n", arg);
        print_usage(argv[0]);
        return false;
    }

    return true;
}

static void json_vec3_double(const IVP_U_Point *p) {
    std::printf("{\"x\":%.17g,\"y\":%.17g,\"z\":%.17g}", (double)p->k[0], (double)p->k[1], (double)p->k[2]);
}

static void json_vec3_float(const IVP_U_Float_Point *p) {
    std::printf("{\"x\":%.9g,\"y\":%.9g,\"z\":%.9g}", (double)p->k[0], (double)p->k[1], (double)p->k[2]);
}

static void json_quat(const IVP_U_Quat *q) {
    std::printf("{\"x\":%.17g,\"y\":%.17g,\"z\":%.17g,\"w\":%.17g}", (double)q->x, (double)q->y, (double)q->z, (double)q->w);
}

static void snapshot_object(int id, const char *type, IVP_Real_Object *obj) {
    IVP_Core *core = obj->get_core();
    const IVP_U_Point *pos = core->get_position_PSI();
    IVP_U_Quat q_norm = core->q_world_f_core_last_psi;
    q_norm.normize_quat();

    std::printf("{\"id\":%d,\"type\":\"%s\",\"pos\":", id, type);
    json_vec3_double(pos);
    std::printf(",\"quat\":");
    json_quat(&q_norm);

    std::printf(",\"lin_vel\":");
    json_vec3_float(&core->speed);
    std::printf(",\"ang_vel\":");
    json_vec3_float(&core->rot_speed);

    const double energy = core->get_energy_on_test(&core->speed, &core->rot_speed);
    std::printf(",\"energy\":%.17g}", energy);
}

void snapshot_frame(Scenario scenario, int step, double dt, IVP_Environment *env,
                    const SceneObjects *scene) {
    const double t = env->get_current_time().get_time();

    std::printf("{\"scenario\":%d,\"step\":%d,\"dt\":%.17g,\"time\":%.17g,\"objects\":[", (int)scenario, step, dt, t);
    for (int i = 0; i < scene->count; ++i) {
        if (i) std::printf(",");
        snapshot_object(i, scene->types[i], scene->objects[i]);
    }
    std::printf("]}\n");
}

IVP_Compact_Surface *build_box_compact_surface(double hx, double hy, double hz) {
    IVP_U_Vector<IVP_U_Point> points(8);

    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                IVP_U_Point *p = new IVP_U_Point();
                p->set((double)sx * hx, (double)sy * hy, (double)sz * hz);
                points.add(p);
            }
        }
    }

    return IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface(&points);
}

void set_quat_axis_angle(IVP_U_Quat *q, double ax, double ay, double az, double radians) {
    const double half = 0.5 * radians;
    const double s = std::sin(half);
    const double c = std::cos(half);
    q->x = ax * s;
    q->y = ay * s;
    q->z = az * s;
    q->w = c;
}

void configure_dynamic_template(IVP_Template_Real_Object *templ_obj, IVP_Material *mat, double mass) {
    *templ_obj = IVP_Template_Real_Object();
    templ_obj->physical_unmoveable = IVP_FALSE;
    templ_obj->pinned = IVP_FALSE;
    templ_obj->enable_piling_optimization = IVP_FALSE;
    templ_obj->mass = mass;
    templ_obj->material = mat;
    templ_obj->speed_damp_factor = 0.0;
    templ_obj->rot_speed_damp_factor.set(0.0, 0.0, 0.0);
    templ_obj->extra_radius = 0.0f;
}

void configure_static_template(IVP_Template_Real_Object *templ_obj, IVP_Material *mat) {
    *templ_obj = IVP_Template_Real_Object();
    templ_obj->physical_unmoveable = IVP_TRUE;
    templ_obj->pinned = IVP_TRUE;
    templ_obj->enable_piling_optimization = IVP_FALSE;
    templ_obj->mass = 0.0;
    templ_obj->material = mat;
    templ_obj->speed_damp_factor = 0.0;
    templ_obj->rot_speed_damp_factor.set(0.0, 0.0, 0.0);
    templ_obj->extra_radius = 0.0f;
}

void wake_and_enable(IVP_Real_Object *obj) {
    obj->enable_collision_detection(IVP_TRUE);
    obj->ensure_in_simulation_now();
}

void configure_explicit_inertia(IVP_Template_Real_Object *templ_obj, double ix, double iy, double iz) {
    templ_obj->rot_inertia_is_factor = IVP_FALSE;
    templ_obj->rot_inertia.set(ix, iy, iz);
}

IVP_Polygon *create_static_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                               const IVP_U_Quat *q, const IVP_U_Point *pos) {
    IVP_Compact_Surface *compact = build_box_compact_surface(hx, hy, hz);
    IVP_SurfaceManager_Polygon *surman = new IVP_SurfaceManager_Polygon(compact);

    IVP_Template_Real_Object templ_obj;
    configure_static_template(&templ_obj, mat);

    IVP_Polygon *obj = env->create_polygon(surman, &templ_obj, q, pos);
    obj->enable_collision_detection(IVP_TRUE);
    return obj;
}

IVP_Polygon *create_dynamic_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                                double mass, double ix, double iy, double iz,
                                double speed_damp, const IVP_U_Point *rot_speed_damp,
                                const IVP_U_Quat *q, const IVP_U_Point *pos) {
    IVP_Compact_Surface *compact = build_box_compact_surface(hx, hy, hz);
    IVP_SurfaceManager_Polygon *surman = new IVP_SurfaceManager_Polygon(compact);

    IVP_Template_Real_Object templ_obj;
    configure_dynamic_template(&templ_obj, mat, mass);
    configure_explicit_inertia(&templ_obj, ix, iy, iz);
    templ_obj.speed_damp_factor = speed_damp;
    if (rot_speed_damp) {
        templ_obj.rot_speed_damp_factor = *rot_speed_damp;
    }

    IVP_Polygon *obj = env->create_polygon(surman, &templ_obj, q, pos);
    wake_and_enable(obj);
    return obj;
}

IVP_Ball *create_dynamic_ball(IVP_Environment *env, IVP_Material *mat,
                              double radius, double mass, double ix, double iy, double iz,
                              double speed_damp, const IVP_U_Point *rot_speed_damp,
                              const IVP_U_Quat *q, const IVP_U_Point *pos) {
    IVP_Template_Real_Object templ_obj;
    configure_dynamic_template(&templ_obj, mat, mass);
    configure_explicit_inertia(&templ_obj, ix, iy, iz);
    templ_obj.speed_damp_factor = speed_damp;
    if (rot_speed_damp) {
        templ_obj.rot_speed_damp_factor = *rot_speed_damp;
    }

    IVP_Template_Ball templ_ball;
    templ_ball.radius = (IVP_FLOAT)radius;
    IVP_Ball *ball = env->create_ball(&templ_ball, &templ_obj, q, pos);
    wake_and_enable(ball);
    return ball;
}

void add_spring(IVP_Environment *env,
                IVP_Real_Object *objA, double ax, double ay, double az,
                IVP_Real_Object *objB, double bx, double by, double bz,
                double constant, double damp, double spring_len) {
    IVP_Template_Anchor anchorA;
    IVP_Template_Anchor anchorB;
    IVP_Template_Spring spring;
    spring.anchors[0] = &anchorA;
    spring.anchors[1] = &anchorB;

    anchorA.set_anchor_position_os(objA, ax, ay, az);
    anchorB.set_anchor_position_os(objB, bx, by, bz);

    spring.spring_len = (IVP_FLOAT)spring_len;
    spring.spring_constant = (IVP_FLOAT)constant;
    spring.spring_damp = (IVP_FLOAT)damp;
    spring.rel_pos_damp = 0.0f;
    spring.spring_values_are_relative = IVP_FALSE;
    spring.spring_force_only_on_stretch = IVP_FALSE;

    IVP_Controller_Factory::create_spring(env, &spring);
}

void add_ballsocket_constraint_two_anchors(IVP_Environment *env,
                                           IVP_Real_Object *objR, const IVP_U_Point *anchor_Ros,
                                           IVP_Real_Object *objA, const IVP_U_Point *anchor_Aos) {
    IVP_U_Point anchor_ws;
    objR->get_core()->m_world_f_core_last_psi.vmult4(anchor_Ros, &anchor_ws);

    IVP_Template_Constraint tc;
    (void)anchor_Aos;
    tc.set_ballsocket_ws(objR, &anchor_ws, objA);
    IVP_Controller_Factory::create_constraint(env, &tc);
}

void add_force_actuator(IVP_Environment *env, IVP_Real_Object *obj,
                        double ax, double ay, double az,
                        double bx, double by, double bz,
                        double force_n) {
    IVP_Template_Anchor a0;
    IVP_Template_Anchor a1;
    IVP_Template_Force tf;
    tf.anchors[0] = &a0;
    tf.anchors[1] = &a1;
    a0.set_anchor_position_os(obj, ax, ay, az);
    a1.set_anchor_position_os(obj, bx, by, bz);
    tf.force = (IVP_FLOAT)force_n;
    tf.push_first_object = IVP_TRUE;
    tf.push_second_object = IVP_FALSE;
    IVP_Controller_Factory::create_force(env, &tf);
}

} // namespace ref_runner
