#ifndef IVP_REFERENCE_RUNNER_REF_RUNNER_HXX
#define IVP_REFERENCE_RUNNER_REF_RUNNER_HXX

#include <ivp_physics.hxx>
#include <ivp_controller_factory.hxx>

#include <ivp_templates.hxx>
#include <ivp_material.hxx>

#include <ivp_surman_polygon.hxx>
#include <ivp_surbuild_pointsoup.hxx>

#include <ivp_actuator_spring.hxx>
#include <ivp_template_constraint.hxx>
#include <ivp_controller_motion.hxx>
#include <ivp_phantom.hxx>

#include <ivp_controller_buoyancy.hxx>
#include <ivp_liquid_surface_descript.hxx>

namespace ref_runner {

enum Scenario {
    SCENARIO_FREEFALL = 0,
    SCENARIO_TWO_BALLS = 1,
    SCENARIO_SLOPE_FRICTION = 2,
    SCENARIO_SPRINGS = 3,
    SCENARIO_ROPE = 4,
    SCENARIO_MOTOR = 5,
    SCENARIO_BUOYANCY = 6,

    SCENARIO_CUBES = 100,
    SCENARIO_FORCE_ACTUATOR = 101,
    SCENARIO_FORCEFIELD = 102,
    SCENARIO_STIFF_SPRING = 103,
    SCENARIO_CHECK_DISTANCE = 104,
    SCENARIO_COLLISION_FILTER = 105,
    SCENARIO_MOTION_CONTROLLER = 106,
    SCENARIO_PHANTOM = 107,
    SCENARIO_VEHICLE = 108,
    SCENARIO_CAR_REAL_WHEELS = 109,

    SCENARIO_CONCAVE_STATIC = 200,
    SCENARIO_COMPOUND_DYNAMIC = 201,
    SCENARIO_CONVEX_HULLS = 202,
    SCENARIO_GRID_TERRAIN = 203,
    SCENARIO_HULL_PILE = 204,
    SCENARIO_GALTON = 205,
    SCENARIO_FAST_IMPACTS = 206,
    SCENARIO_SPAWN_REMOVE = 207,
    SCENARIO_LONG_RANGE = 208,

    /* controllers2: ref_runner_scenarios_controllers2.cxx */
    SCENARIO_TORQUE = 300,
    SCENARIO_STABILIZER = 301,
    SCENARIO_FLOATING = 302,
    SCENARIO_WORLD_FRICTION = 303,
    SCENARIO_GOLEM = 304,
    SCENARIO_FIXED_KEYED = 305,
    SCENARIO_HINGE_LIMITS = 306,
    SCENARIO_CARDAN_TENSE = 307,
    SCENARIO_SLIDER_LIMITS = 308,
    SCENARIO_CONSTRAINT_BREAK = 309,
    SCENARIO_ATTACHER = 310,
    SCENARIO_ACTUATOR_EXTRA = 311,
    SCENARIO_AIRBOAT = 312,
    SCENARIO_FAKE_JETSKI = 313,

    /* controllers3: ref_runner_scenarios_controllers3.cxx */
    SCENARIO_RAYCAST_CAR_DRIVE = 400,
    SCENARIO_CHECK_DIST_EVENTS = 401,
    SCENARIO_GOLEM_BEAM = 402,

    /* misc: ref_runner_scenarios_misc.cxx */
    SCENARIO_UNIVERSE_EVICT = 500,

    /* shared cores: ref_runner_scenarios_merge.cxx */
    SCENARIO_MERGE_OBJECTS = 600,
    SCENARIO_OBJECT_ATTACH = 601,
    SCENARIO_MERGE_BUOYANCY = 602,
    SCENARIO_ANCHOR_FOLLOW = 603
};

struct RunnerConfig {
    Scenario scenario;
    int steps;
    double dt;

    RunnerConfig() : scenario(SCENARIO_FREEFALL), steps(240), dt(1.0 / 60.0) {}
};

struct SceneObjects {
    IVP_Real_Object *objects[128];
    const char *types[128];
    int count;

    SceneObjects();
};

struct ScenarioResources {
    IVP_U_Set_Active<IVP_Core> *buoyancy_cores;
    IVP_Attacher_To_Cores_Buoyancy *buoyancy_attacher;
    IVP_Liquid_Surface_Descriptor_Simple *liquid_desc;

    /* optional per-step scenario input (called before simulating step N, 1-based)
     * and scenario-owned teardown (called before the environment is deleted) */
    void (*step_hook)(int step, IVP_Environment *env, ScenarioResources *resources);
    void (*cleanup_hook)(ScenarioResources *resources);
    void *scenario_data;

    ScenarioResources();
    void cleanup();
};

bool ref_debug_enabled();
void print_usage(const char *exe);
bool parse_args(int argc, char **argv, RunnerConfig *cfg);

void snapshot_frame(Scenario scenario, int step, double dt, IVP_Environment *env,
                    const SceneObjects *scene);

IVP_Compact_Surface *build_box_compact_surface(double hx, double hy, double hz);
void set_quat_axis_angle(IVP_U_Quat *q, double ax, double ay, double az, double radians);

void configure_dynamic_template(IVP_Template_Real_Object *templ_obj, IVP_Material *mat, double mass);
void configure_static_template(IVP_Template_Real_Object *templ_obj, IVP_Material *mat);
void configure_explicit_inertia(IVP_Template_Real_Object *templ_obj, double ix, double iy, double iz);

void wake_and_enable(IVP_Real_Object *obj);

IVP_Polygon *create_static_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                               const IVP_U_Quat *q, const IVP_U_Point *pos);

IVP_Polygon *create_dynamic_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                                double mass, double ix, double iy, double iz,
                                double speed_damp, const IVP_U_Point *rot_speed_damp,
                                const IVP_U_Quat *q, const IVP_U_Point *pos);

IVP_Ball *create_dynamic_ball(IVP_Environment *env, IVP_Material *mat,
                              double radius, double mass, double ix, double iy, double iz,
                              double speed_damp, const IVP_U_Point *rot_speed_damp,
                              const IVP_U_Quat *q, const IVP_U_Point *pos);

void add_spring(IVP_Environment *env,
                IVP_Real_Object *objA, double ax, double ay, double az,
                IVP_Real_Object *objB, double bx, double by, double bz,
                double constant, double damp, double spring_len);

void add_ballsocket_constraint_two_anchors(IVP_Environment *env,
                                           IVP_Real_Object *objR, const IVP_U_Point *anchor_Ros,
                                           IVP_Real_Object *objA, const IVP_U_Point *anchor_Aos);

void add_force_actuator(IVP_Environment *env, IVP_Real_Object *obj,
                        double ax, double ay, double az,
                        double bx, double by, double bz,
                        double force_n);

bool setup_scenario(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);

bool setup_basic_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);
bool setup_constraint_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);
bool setup_controller_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);
bool setup_car_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);
bool setup_geometry_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);
bool setup_controllers2_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);
bool setup_controllers3_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);
bool setup_misc_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);
bool setup_merge_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources);

} // namespace ref_runner

#endif
