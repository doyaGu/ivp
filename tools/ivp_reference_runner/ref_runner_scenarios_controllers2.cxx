// ref_runner_scenarios_controllers2.cxx -- parity scenarios for controllers
// that no other scenario exercises value by value:
//   torque          IVP_Actuator_Torque (anchors, max_rotation_speed, set_torque)
//   stabilizer      IVP_Actuator_Stabilizer (announced explicitly, see below)
//   floating        IVP_Controller_Floating (analytic ground "ray cast")
//   world_friction  IVP_Controller_World_Friction
//   golem           IVP_Controller_Golem (prime position/orientation, problems)
//   fixed_keyed     IVP_Constraint_Fixed_Keyframed
//   hinge_limits    IVP_Constraint_Local hinge with limited rotation axis
//   cardan_tense    cardan joint, tense ball socket, fixed (weld)
//   slider_limits   2 fixed translation axes + limited one, plane with limits
//   constraint_break IVP_CFE_BREAK / IVP_CFE_CLIP max impulses
//   attacher        IVP_Attacher_To_Cores<floating attachment>, set changes
//   actuator_extra  IVP_Actuator_Extra (float cam + puck force)
//   airboat         IVP_Controller_Raycast_Airboat (beach + water, analytic rays)
//   fake_jetski     IVP_Controller_Raycast_Fake_Jetski (analytic ground rays)
// Mirrored by the run_* functions of the "controllers2" block in libivp's
// tests/test_scenarios.c; keep both in sync.

#include "ref_runner.hxx"

#include <ivp_actuator.hxx>
#include <ivp_controller_floating.hxx>
#include <ivp_controller_world_frict.hxx>
#include <ivp_controller_golem.hxx>
#include <ivp_constraint_fixed_keyed.hxx>
#include <ivp_constraint.hxx>
#include <ivp_attacher_to_cores.hxx>
#include <ivp_car_system.hxx>
#include <ivp_ray_solver.hxx>
#include <ivp_controller_airboat.h>
#include <ivp_controller_fake_jetski.h>

#include <cstdio>

namespace ref_runner {

namespace {

const double C2_GROUND_TOP = 2.5; // ground box (hy 0.5) centred at y = 3

IVP_Polygon *c2_ground(IVP_Environment *env, IVP_Material *mat) {
    IVP_U_Quat q; q.x = q.y = q.z = 0.0; q.w = 1.0;
    IVP_U_Point p; p.set(0.0, 3.0, 0.0);
    return create_static_box(env, mat, 20.0, 0.5, 20.0, &q, &p);
}

IVP_Polygon *c2_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                    double mass, double ix, double iy, double iz,
                    double x, double y, double z, const IVP_U_Quat *q) {
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point rd; rd.set(0.0, 0.0, 0.0);
    IVP_U_Point p; p.set(x, y, z);
    return create_dynamic_box(env, mat, hx, hy, hz, mass, ix, iy, iz, 0.0, &rd, q ? q : &qi, &p);
}

IVP_Ball *c2_ball(IVP_Environment *env, IVP_Material *mat, double r, double mass, double inertia,
                  double x, double y, double z) {
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point rd; rd.set(0.0, 0.0, 0.0);
    IVP_U_Point p; p.set(x, y, z);
    return create_dynamic_ball(env, mat, r, mass, inertia, inertia, inertia, 0.0, &rd, &qi, &p);
}

IVP_Polygon *c2_static(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                       double x, double y, double z) {
    IVP_U_Quat q; q.x = q.y = q.z = 0.0; q.w = 1.0;
    IVP_U_Point p; p.set(x, y, z);
    return create_static_box(env, mat, hx, hy, hz, &q, &p);
}

void c2_add(SceneObjects *scene, IVP_Real_Object *o, const char *type) {
    scene->objects[scene->count] = o;
    scene->types[scene->count] = type;
    scene->count++;
}

/* ===================================================================== */
/* torque                                                                 */
/* ===================================================================== */

struct TorqueScenario {
    IVP_Actuator_Torque *spinner, *roller, *tumbler;
};

IVP_Actuator_Torque *c2_torque(IVP_Environment *env, IVP_Real_Object *obj,
                               double ax, double ay, double az, double torque, double max_speed) {
    IVP_Template_Anchor a0, a1;
    IVP_Template_Torque tt;
    tt.anchors[0] = &a0;
    tt.anchors[1] = &a1;
    a0.set_anchor_position_os(obj, 0.0, 0.0, 0.0);
    a1.set_anchor_position_os(obj, ax, ay, az);
    tt.torque = (IVP_FLOAT)torque;
    tt.max_rotation_speed = (IVP_FLOAT)max_speed;
    return IVP_Controller_Factory::create_torque(env, &tt);
}

void torque_step_hook(int step, IVP_Environment *, ScenarioResources *res) {
    TorqueScenario *s = (TorqueScenario *)res->scenario_data;
    if (step == 241) {
        s->roller->set_torque(-2.5);
        s->spinner->set_max_rotation_speed(8.0);
    }
    if (step == 361) {
        s->tumbler->set_torque(0.0);
    }
}

void torque_cleanup_hook(ScenarioResources *res) {
    delete (TorqueScenario *)res->scenario_data;
    res->scenario_data = 0;
}

void setup_torque(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *spinner = c2_box(env, &mat_dyn, 0.8, 0.3, 0.8, 2.0, 0.5, 0.85, 0.5, -4.0, 2.15, 0.0, 0);
    IVP_Ball *roller = c2_ball(env, &mat_dyn, 0.5, 2.0, 0.2, 0.0, 1.95, -3.0);
    IVP_U_Quat qt; set_quat_axis_angle(&qt, 1.0, 0.0, 0.0, 0.3);
    IVP_Polygon *tumbler = c2_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.0, 0.11, 0.11, 0.11, 4.0, -2.0, 0.0, &qt);

    TorqueScenario *s = new TorqueScenario;
    s->spinner = c2_torque(env, spinner, 0.0, -1.0, 0.0, 12.0, 4.0);
    s->roller = c2_torque(env, roller, 0.0, 0.0, 1.0, 2.5, 6.0);
    s->tumbler = c2_torque(env, tumbler, 0.6, 0.6, 0.2, 1.5, 2.0);
    res->scenario_data = s;
    res->step_hook = torque_step_hook;
    res->cleanup_hook = torque_cleanup_hook;

    c2_add(scene, spinner, "spinner");
    c2_add(scene, roller, "roller");
    c2_add(scene, tumbler, "tumbler");
}

/* ===================================================================== */
/* stabilizer                                                             */
/* ===================================================================== */

struct StabilizerScenario {
    IVP_Actuator_Stabilizer *stab[2];
};

IVP_Actuator_Stabilizer *c2_stabilizer(IVP_Environment *env,
                                       IVP_Real_Object *o0, double x0, double y0, double z0,
                                       IVP_Real_Object *o1, double x1, double y1, double z1,
                                       IVP_Real_Object *o2, double x2, double y2, double z2,
                                       IVP_Real_Object *o3, double x3, double y3, double z3,
                                       double constant) {
    IVP_Template_Anchor a[4];
    IVP_Template_Stabilizer ts;
    for (int i = 0; i < 4; ++i) ts.anchors[i] = &a[i];
    a[0].set_anchor_position_os(o0, x0, y0, z0);
    a[1].set_anchor_position_os(o1, x1, y1, z1);
    a[2].set_anchor_position_os(o2, x2, y2, z2);
    a[3].set_anchor_position_os(o3, x3, y3, z3);
    ts.stabi_constant = (IVP_FLOAT)constant;
    IVP_Actuator_Stabilizer *st = IVP_Controller_Factory::create_stabilizer(env, &ts);
    // The reference IVP_Actuator_Four_Point constructor never announces the
    // controller (a factory stabilizer is inert); announce it here so that
    // IVP_Actuator_Stabilizer::do_simulation_controller actually runs.
    env->get_controller_manager()->announce_controller_to_environment(st);
    return st;
}

void stabilizer_cleanup_hook(ScenarioResources *res) {
    StabilizerScenario *s = (StabilizerScenario *)res->scenario_data;
    if (!s) return;
    delete s->stab[0];
    delete s->stab[1];
    delete s;
    res->scenario_data = 0;
}

void setup_stabilizer(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);

    IVP_Polygon *frame = c2_static(env, &mat_static, 2.0, 0.2, 0.3, 0.0, -8.0, 0.0);
    IVP_Polygon *wl = c2_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, -1.5, -5.5, 0.0, 0);
    IVP_Polygon *wr = c2_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, 1.5, -4.5, 0.0, 0);
    add_spring(env, frame, -1.5, 0.2, 0.0, wl, 0.0, -0.3, 0.0, 60.0, 2.0, 2.0);
    add_spring(env, frame, 1.5, 0.2, 0.0, wr, 0.0, -0.3, 0.0, 60.0, 2.0, 2.0);

    IVP_Polygon *hook = c2_static(env, &mat_static, 0.2, 0.2, 0.2, 0.0, -10.0, 6.0);
    IVP_Polygon *bar = c2_box(env, &mat_dyn, 2.0, 0.15, 0.3, 2.0, 0.05, 0.7, 0.68, 0.0, -8.0, 6.0, 0);
    IVP_Polygon *wl2 = c2_box(env, &mat_dyn, 0.3, 0.3, 0.3, 0.8, 0.05, 0.05, 0.05, -1.8, -5.0, 6.0, 0);
    IVP_Polygon *wr2 = c2_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.2, 0.07, 0.07, 0.07, 1.8, -6.0, 6.0, 0);
    add_spring(env, hook, 0.0, 0.2, 0.0, bar, 0.0, -0.15, 0.0, 200.0, 10.0, 1.6);
    add_spring(env, bar, -1.8, 0.15, 0.0, wl2, 0.0, -0.3, 0.0, 50.0, 1.0, 2.2);
    add_spring(env, bar, 1.8, 0.15, 0.0, wr2, 0.0, -0.3, 0.0, 50.0, 1.0, 1.4);

    StabilizerScenario *s = new StabilizerScenario;
    s->stab[0] = c2_stabilizer(env, frame, -1.5, 0.2, 0.0, wl, 0.0, -0.3, 0.0,
                               frame, 1.5, 0.2, 0.0, wr, 0.0, -0.3, 0.0, 20.0);
    s->stab[1] = c2_stabilizer(env, bar, -1.8, 0.15, 0.0, wl2, 0.0, -0.3, 0.0,
                               bar, 1.8, 0.15, 0.0, wr2, 0.0, -0.3, 0.0, 15.0);
    res->scenario_data = s;
    res->cleanup_hook = stabilizer_cleanup_hook;

    c2_add(scene, frame, "frame");
    c2_add(scene, wl, "weight");
    c2_add(scene, wr, "weight");
    c2_add(scene, hook, "hook");
    c2_add(scene, bar, "bar");
    c2_add(scene, wl2, "weight");
    c2_add(scene, wr2, "weight");
}

/* ===================================================================== */
/* floating                                                               */
/* ===================================================================== */

// do_ray_casting: distance from the anchor along the ray to the ground
// plane y = C2_GROUND_TOP.
class RunnerFloating : public IVP_Controller_Floating {
public:
    RunnerFloating(IVP_Real_Object *obj, const IVP_Template_Controller_Floating *t)
        : IVP_Controller_Floating(obj, t) {}
    IVP_RETURN_TYPE do_ray_casting(IVP_Event_Sim *) {
        IVP_U_Point p;
        object->transform_position_to_world_coords(get_position_os(), &p);
        IVP_DOUBLE dist = (C2_GROUND_TOP - p.k[1]) / ray_direction_ws.k[1];
        set_current_distance(dist);
        return IVP_OK;
    }
};

struct FloatingScenario {
    RunnerFloating *a, *b, *c;
};

void floating_step_hook(int step, IVP_Environment *, ScenarioResources *res) {
    FloatingScenario *s = (FloatingScenario *)res->scenario_data;
    if (step == 241) {
        s->a->set_target_distance(0.4);
        IVP_U_Float_Point d(0.0, 1.0, 0.2);
        s->b->set_ray_direction_ws(&d);
    }
    if (step == 331) {
        s->c->set_target_distance(0.8);
    }
}

void floating_cleanup_hook(ScenarioResources *res) {
    delete (FloatingScenario *)res->scenario_data; // controllers die with their cores
    res->scenario_data = 0;
}

void setup_floating(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *a = c2_box(env, &mat_dyn, 0.5, 0.5, 0.5, 1.0, 0.17, 0.17, 0.17, -4.0, 0.0, 0.0, 0);
    IVP_U_Quat qb; set_quat_axis_angle(&qb, 0.0, 0.0, 1.0, 0.2);
    IVP_Polygon *b = c2_box(env, &mat_dyn, 0.6, 0.25, 0.4, 2.0, 0.15, 0.35, 0.28, 0.0, 0.5, 0.0, &qb);
    IVP_Ball *c = c2_ball(env, &mat_dyn, 0.4, 1.5, 0.096, 4.0, -1.0, 0.0);

    FloatingScenario *s = new FloatingScenario;
    {
        IVP_Template_Controller_Floating t;
        t.max_repulsive_force = 30.0f;
        t.max_adhesive_force = 5.0f;
        t.position_os.set(0.0, 0.5, 0.0);
        t.target_distance = 1.0f;
        s->a = new RunnerFloating(a, &t);
    }
    {
        IVP_Template_Controller_Floating t;
        t.max_repulsive_force = 40.0f;
        t.max_adhesive_force = 40.0f;
        t.position_os.set(0.4, 0.25, 0.2);
        t.target_distance = 0.6f;
        s->b = new RunnerFloating(b, &t);
    }
    {
        IVP_Template_Controller_Floating t;
        t.max_repulsive_force = 15.0f;
        t.max_adhesive_force = 2.0f;
        t.position_os.set(0.0, 0.0, 0.0);
        IVP_U_Point dir; dir.set(0.3, 1.0, 0.0);
        t.set_ray_direction_ws(c, &dir);
        t.target_distance = 1.5f;
        s->c = new RunnerFloating(c, &t);
    }
    res->scenario_data = s;
    res->step_hook = floating_step_hook;
    res->cleanup_hook = floating_cleanup_hook;

    c2_add(scene, a, "hover");
    c2_add(scene, b, "hover");
    c2_add(scene, c, "hover");
}

/* ===================================================================== */
/* world_friction                                                         */
/* ===================================================================== */

struct WorldFrictionScenario {
    IVP_Controller_World_Friction *slider, *floater, *tilt;
};

void world_friction_step_hook(int step, IVP_Environment *, ScenarioResources *res) {
    WorldFrictionScenario *s = (WorldFrictionScenario *)res->scenario_data;
    if (step == 241) {
        IVP_U_Point r; r.set(2.0, 0.1, 5.0);
        s->tilt->set_friction_value_rotation(r);
        IVP_U_Float_Point v(-1.0, -0.5, 0.5);
        s->floater->set_desired_speed_ws(&v);
    }
    if (step == 301) {
        IVP_U_Point t; t.set(-1.0, 3.0, 3.0);
        s->slider->set_friction_value_translation(t);
    }
}

void world_friction_cleanup_hook(ScenarioResources *res) {
    delete (WorldFrictionScenario *)res->scenario_data;
    res->scenario_data = 0;
}

void setup_world_friction(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.6, 0.0);
    static IVP_Material_Simple mat_slider(0.3, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *slider = c2_box(env, &mat_slider, 0.5, 0.5, 0.5, 2.0, 0.33, 0.33, 0.33, -6.0, 1.95, 0.0, 0);
    slider->get_core()->speed.set(6.0f, 0.0f, 2.0f);
    slider->get_core()->rot_speed.set(0.0f, 3.0f, 0.0f);
    IVP_Ball *floater = c2_ball(env, &mat_dyn, 0.5, 1.0, 0.1, 0.0, -2.0, 0.0);
    IVP_U_Quat qt; set_quat_axis_angle(&qt, 0.0, 0.0, 1.0, 0.5);
    IVP_Polygon *tilt = c2_box(env, &mat_dyn, 0.6, 0.3, 0.3, 1.5, 0.09, 0.225, 0.225, 5.0, -3.0, 0.0, &qt);
    tilt->get_core()->speed.set(0.0f, 0.0f, -3.0f);

    WorldFrictionScenario *s = new WorldFrictionScenario;
    {
        IVP_Template_Controller_World_Friction t;
        s->slider = new IVP_Controller_World_Friction(slider, &t);
    }
    {
        IVP_Template_Controller_World_Friction t;
        t.desired_speed_ws.set(1.0, 0.0, 0.0);
        t.desired_rot_speed_cs.set(0.0, 0.0, 2.0);
        t.friction_value_translation.set(2.0, 12.0, 2.0);
        t.friction_value_rotation.set(1.0, 1.0, 1.0);
        s->floater = new IVP_Controller_World_Friction(floater, &t);
    }
    {
        IVP_Template_Controller_World_Friction t;
        t.friction_value_translation.set(3.0, 0.5, 6.0);
        t.friction_value_rotation.set(0.4, 0.4, 0.4);
        s->tilt = new IVP_Controller_World_Friction(tilt, &t);
    }
    res->scenario_data = s;
    res->step_hook = world_friction_step_hook;
    res->cleanup_hook = world_friction_cleanup_hook;

    c2_add(scene, slider, "slider");
    c2_add(scene, floater, "floater");
    c2_add(scene, tilt, "tilt");
}

/* ===================================================================== */
/* golem                                                                  */
/* ===================================================================== */

class RunnerGolem : public IVP_Controller_Golem {
public:
    int problems;
    RunnerGolem(IVP_Real_Object *o, const IVP_Template_Controller_Golem *t)
        : IVP_Controller_Golem(o, t), problems(0) {}
    IVP_RETURN_TYPE resolve_for_problem(IVP_Event_Sim *, IVP_GOLEM_PROBLEM) {
        problems++;
        return IVP_OK;
    }
};

struct GolemScenario {
    RunnerGolem *a, *b, *c;
};

void golem_step_hook(int step, IVP_Environment *env, ScenarioResources *res) {
    GolemScenario *s = (GolemScenario *)res->scenario_data;
    IVP_Time now = env->get_current_time();
    if (step == 121) {
        IVP_U_Quat q; set_quat_axis_angle(&q, 1.0, 0.0, 0.0, 2.0);
        s->c->set_prime_orientation(&q, now);
    }
    if (step == 201) {
        IVP_U_Point p; p.set(2.0, -1.0, 15.0);
        IVP_U_Float_Point v(0.0, 0.0, 0.0);
        s->b->set_prime_position(&p, &v, now);
    }
    if (step == 261) {
        IVP_U_Quat q0; set_quat_axis_angle(&q0, 1.0, 0.0, 0.0, 0.6);
        IVP_U_Quat q1; set_quat_axis_angle(&q1, 0.0, 1.0, 0.0, 0.8);
        s->c->set_prime_orientation(&q0, now, &q1, 1.0f);
    }
    if (step == 331) {
        IVP_U_Point p; p.set(2.0, 0.5, 3.0);
        IVP_U_Float_Point v(0.0, 0.0, -1.0);
        s->b->set_prime_position(&p, &v, now);
    }
}

void golem_cleanup_hook(ScenarioResources *res) {
    delete (GolemScenario *)res->scenario_data;
    res->scenario_data = 0;
}

void setup_golem(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *a = c2_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.5, 0.16, 0.16, 0.16, -4.0, 0.0, 0.0, 0);
    IVP_Ball *b = c2_ball(env, &mat_dyn, 0.5, 1.0, 0.1, 2.0, -1.0, 0.0);
    IVP_Polygon *c = c2_box(env, &mat_dyn, 0.5, 0.3, 0.5, 2.0, 0.2, 0.33, 0.2, 5.0, 2.15, 0.0, 0);

    GolemScenario *s = new GolemScenario;
    IVP_Time now = env->get_current_time();
    {
        IVP_Template_Controller_Golem t;
        t.max_translation_force.set(60.0f, 60.0f, 60.0f);
        t.max_torque = 20.0f;
        s->a = new RunnerGolem(a, &t);
        IVP_U_Point p; p.set(-4.0, 0.0, 0.0);
        IVP_U_Float_Point v(2.0, 0.0, 0.5);
        s->a->set_prime_position(&p, &v, now);
        IVP_U_Quat q0; q0.x = q0.y = q0.z = 0.0; q0.w = 1.0;
        IVP_U_Quat q1; set_quat_axis_angle(&q1, 0.0, 1.0, 0.0, 1.0);
        s->a->set_prime_orientation(&q0, now, &q1, 2.0f);
    }
    {
        IVP_Template_Controller_Golem t;
        t.max_translation_force.set(100.0f, 100.0f, 100.0f);
        t.max_torque = 50.0f;
        s->b = new RunnerGolem(b, &t);
    }
    {
        IVP_Template_Controller_Golem t;
        t.max_translation_force.set(40.0f, 40.0f, 40.0f);
        t.max_torque = 15.0f;
        t.force_factor = 0.6f;
        t.damp_factor = 0.9f;
        t.torque_factor = 0.7f;
        s->c = new RunnerGolem(c, &t);
    }
    res->scenario_data = s;
    res->step_hook = golem_step_hook;
    res->cleanup_hook = golem_cleanup_hook;

    c2_add(scene, a, "golem");
    c2_add(scene, b, "golem");
    c2_add(scene, c, "golem");
}

/* ===================================================================== */
/* fixed_keyed                                                            */
/* ===================================================================== */

struct FixedKeyedScenario {
    IVP_Constraint_Fixed_Keyframed *fk1, *fk2;
};

void fixed_keyed_step_hook(int step, IVP_Environment *env, ScenarioResources *res) {
    FixedKeyedScenario *s = (FixedKeyedScenario *)res->scenario_data;
    IVP_Time now = env->get_current_time();
    if (step == 121) {
        IVP_U_Point p; p.set(2.0, -1.0, 1.0);
        IVP_U_Float_Point v(-1.0, 0.0, 0.0);
        s->fk1->set_prime_position_Ros(&p, &v, now);
        IVP_U_Quat q0; q0.x = q0.y = q0.z = 0.0; q0.w = 1.0;
        IVP_U_Quat q1; set_quat_axis_angle(&q1, 0.0, 0.0, 1.0, 1.2);
        s->fk1->set_prime_orientation_Ros(&q0, now, &q1, 1.5f);
    }
    if (step == 201) {
        IVP_U_Point p; p.set(0.5, -1.5, 0.0);
        IVP_U_Float_Point v(0.0, 0.0, 0.0);
        s->fk2->set_prime_position_Ros(&p, &v, now);
    }
}

void fixed_keyed_cleanup_hook(ScenarioResources *res) {
    FixedKeyedScenario *s = (FixedKeyedScenario *)res->scenario_data;
    if (!s) return;
    delete s->fk1;
    delete s->fk2;
    delete s;
    res->scenario_data = 0;
}

void setup_fixed_keyed(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *post = c2_static(env, &mat_ground, 0.2, 0.2, 0.2, 0.0, -2.0, 0.0);
    IVP_Polygon *fa = c2_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, 2.0, -2.0, 0.0, 0);
    IVP_Polygon *base = c2_box(env, &mat_dyn, 0.8, 0.3, 0.8, 3.0, 0.73, 1.28, 0.73, 4.0, 2.15, 4.0, 0);
    IVP_Polygon *fb = c2_box(env, &mat_dyn, 0.25, 0.25, 0.25, 0.5, 0.02, 0.02, 0.02, 4.0, 1.0, 4.0, 0);

    FixedKeyedScenario *s = new FixedKeyedScenario;
    {
        IVP_Template_Constraint_Fixed_Keyframed t;
        t.max_translation_force.set(200.0f, 200.0f, 200.0f);
        t.max_torque = 50.0f;
        s->fk1 = new IVP_Constraint_Fixed_Keyframed(post, fa, &t);
    }
    {
        IVP_Template_Constraint_Fixed_Keyframed t;
        t.max_translation_force.set(40.0f, 40.0f, 40.0f);
        t.max_torque = 10.0f;
        t.force_factor = 0.5f;
        t.damp_factor = 0.6f;
        t.torque_factor = 0.6f;
        t.angular_damp_factor = 0.8f;
        s->fk2 = new IVP_Constraint_Fixed_Keyframed(base, fb, &t);
    }
    res->scenario_data = s;
    res->step_hook = fixed_keyed_step_hook;
    res->cleanup_hook = fixed_keyed_cleanup_hook;

    c2_add(scene, post, "post");
    c2_add(scene, fa, "keyed");
    c2_add(scene, base, "base");
    c2_add(scene, fb, "keyed");
}

/* ===================================================================== */
/* hinge_limits                                                           */
/* ===================================================================== */

void setup_hinge_limits(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *post = c2_static(env, &mat_ground, 0.2, 0.2, 0.2, 0.0, -8.0, 0.0);
    IVP_Polygon *arm1 = c2_box(env, &mat_dyn, 1.0, 0.1, 0.1, 1.0, 0.05, 0.337, 0.337, 1.2, -6.0, 0.0, 0);
    IVP_Polygon *arm2 = c2_box(env, &mat_dyn, 0.6, 0.1, 0.1, 0.5, 0.03, 0.062, 0.062, 2.9, -6.0, 0.0, 0);
    IVP_Polygon *post2 = c2_static(env, &mat_ground, 0.2, 0.2, 0.2, 0.0, -8.0, 5.0);
    IVP_Polygon *door = c2_box(env, &mat_dyn, 0.1, 0.8, 0.6, 2.0, 0.67, 0.25, 0.43, 0.0, -6.0, 5.8, 0);
    door->get_core()->rot_speed.set(0.0f, 4.0f, 0.0f);

    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(0.2, -6.0, 0.0);
        IVP_U_Point axis; axis.set(0.0, 0.0, 1.0);
        tc.set_hinge_ws(post, &anchor, &axis, arm1);
        tc.limit_rotation_axis(IVP_INDEX_Z, -0.7f, 0.7f);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }
    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(2.25, -6.0, 0.0);
        IVP_U_Point axis; axis.set(0.0, 0.0, 1.0);
        tc.set_hinge_ws(arm1, &anchor, &axis, arm2);
        tc.limit_rotation_axis(IVP_INDEX_Z, -0.2f, 0.4f);
        tc.set_stiffness_for_limited_axis(0.8f);
        tc.force_factor = 0.7f;
        tc.damp_factor = 0.9f;
        IVP_Controller_Factory::create_constraint(env, &tc);
    }
    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(0.0, -6.0, 5.2);
        IVP_U_Point axis; axis.set(0.0, 1.0, 0.0);
        tc.set_hinge_ws(post2, &anchor, &axis, door);
        tc.limit_rotation_axis(IVP_INDEX_Z, -1.0f, 0.5f);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }

    c2_add(scene, post, "post");
    c2_add(scene, arm1, "arm");
    c2_add(scene, arm2, "arm");
    c2_add(scene, post2, "post");
    c2_add(scene, door, "door");
}

/* ===================================================================== */
/* cardan_tense                                                           */
/* ===================================================================== */

void setup_cardan_tense(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *ceiling = c2_static(env, &mat_ground, 3.0, 0.2, 0.3, 0.0, -9.3, 0.0);
    IVP_Polygon *pend = c2_box(env, &mat_dyn, 0.2, 0.8, 0.2, 1.0, 0.227, 0.027, 0.227, 0.0, -8.0, 0.0, 0);
    pend->get_core()->rot_speed.set(1.5f, 3.0f, 0.5f);
    pend->get_core()->speed.set(2.0f, 0.0f, 0.0f);
    IVP_Ball *tense = c2_ball(env, &mat_dyn, 0.3, 0.7, 0.025, 2.5, -7.5, 0.0);
    IVP_Polygon *weld_a = c2_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, -3.0, -1.0, 0.0, 0);
    IVP_U_Quat qw; set_quat_axis_angle(&qw, 0.0, 1.0, 0.0, 0.3);
    IVP_Polygon *weld_b = c2_box(env, &mat_dyn, 0.3, 0.3, 0.3, 0.6, 0.036, 0.036, 0.036, -2.2, -1.0, 0.0, &qw);
    weld_a->get_core()->rot_speed.set(0.0f, 0.0f, 2.0f);

    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(0.0, -8.8, 0.0);
        IVP_U_Point axis; axis.set(0.0, 1.0, 0.0);
        tc.set_cardanjoint_ws(ceiling, &anchor, &axis, pend);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }
    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor_Ros; anchor_Ros.set(2.0, 0.2, 0.0);
        IVP_U_Point dist_Ros; dist_Ros.set(0.0, 1.0, 0.0);
        tc.set_ballsocket_tense_Ros(ceiling, &anchor_Ros, tense, &dist_Ros);
        tc.force_factor = 0.6f;
        IVP_Controller_Factory::create_constraint(env, &tc);
    }
    {
        IVP_Template_Constraint tc;
        tc.set_fixed(weld_a, weld_b);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }

    c2_add(scene, ceiling, "ceiling");
    c2_add(scene, pend, "cardan");
    c2_add(scene, tense, "tense");
    c2_add(scene, weld_a, "weld");
    c2_add(scene, weld_b, "weld");
}

/* ===================================================================== */
/* slider_limits                                                          */
/* ===================================================================== */

void setup_slider_limits(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *rail = c2_static(env, &mat_ground, 0.2, 0.2, 0.2, 0.0, -6.0, 0.0);
    IVP_Polygon *slider = c2_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, -2.0, -4.0, 0.0, 0);
    IVP_Polygon *rail2 = c2_static(env, &mat_ground, 0.2, 0.2, 0.2, 0.0, -6.0, 4.0);
    IVP_Ball *puck = c2_ball(env, &mat_dyn, 0.3, 0.8, 0.029, 2.0, -3.0, 4.0);
    puck->get_core()->speed.set(3.0f, 0.0f, 1.0f);
    puck->get_core()->rot_speed.set(0.0f, 2.0f, 0.0f);

    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(-2.0, -4.0, 0.0);
        IVP_U_Point axis; axis.set(1.0, 1.0, 0.0);
        tc.set_constraint_ws(rail, &anchor, &axis, 2, 3, slider, NULL);
        tc.limit_translation_axis(IVP_INDEX_Z, -1.0f, 1.5f);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }
    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(2.0, -3.0, 4.0);
        IVP_U_Point axis; axis.set(0.0, 1.0, 0.0);
        tc.set_constraint_ws(rail2, &anchor, &axis, 1, 0, puck, NULL);
        tc.limit_translation_axis(IVP_INDEX_Y, -0.5f, 0.5f);
        tc.limit_translation_axis(IVP_INDEX_Z, -1.0f, 1.0f);
        tc.set_stiffness_for_limited_axis(0.5f);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }

    c2_add(scene, rail, "rail");
    c2_add(scene, slider, "slider");
    c2_add(scene, rail2, "rail");
    c2_add(scene, puck, "puck");
}

/* ===================================================================== */
/* constraint_break                                                       */
/* ===================================================================== */

void setup_constraint_break(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    IVP_Polygon *anchor1 = c2_static(env, &mat_ground, 0.2, 0.2, 0.2, -3.0, -10.5, 0.0);
    IVP_U_Quat qh; set_quat_axis_angle(&qh, 0.0, 0.0, 1.0, 1.5707963267948966);
    IVP_Polygon *a1 = c2_box(env, &mat_dyn, 0.15, 0.5, 0.15, 1.0, 0.091, 0.015, 0.091, -2.5, -9.8, 0.0, &qh);
    IVP_Polygon *anchor2 = c2_static(env, &mat_ground, 0.2, 0.2, 0.2, 3.0, -10.5, 0.0);
    IVP_Polygon *b1 = c2_box(env, &mat_dyn, 0.15, 0.5, 0.15, 2.0, 0.18, 0.03, 0.18, 3.0, -9.3, 0.0, 0);
    b1->get_core()->speed.set(0.0f, 0.0f, 4.0f);
    IVP_Polygon *anchor3 = c2_static(env, &mat_ground, 0.2, 0.2, 0.2, 0.0, -11.2, 4.0);
    IVP_Polygon *door = c2_box(env, &mat_dyn, 0.6, 0.6, 0.05, 1.0, 0.12, 0.12, 0.24, 0.8, -10.0, 4.0, 0);
    IVP_Ball *hammer = c2_ball(env, &mat_dyn, 0.3, 3.0, 0.108, 1.1, -13.0, 4.0);

    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(-3.0, -9.8, 0.0);
        tc.set_ballsocket_ws(anchor1, &anchor, a1);
        tc.set_max_translation_impulse(IVP_CFE_BREAK, 0.1f);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }
    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(3.0, -9.8, 0.0);
        tc.set_ballsocket_ws(anchor2, &anchor, b1);
        tc.set_max_translation_impulse(IVP_CFE_CLIP, 0.2f);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }
    {
        IVP_Template_Constraint tc;
        IVP_U_Point anchor; anchor.set(0.2, -10.0, 4.0);
        IVP_U_Point axis; axis.set(0.0, 1.0, 0.0);
        tc.set_hinge_ws(anchor3, &anchor, &axis, door);
        tc.set_max_translation_impulse(IVP_CFE_BREAK, 0.4f);
        tc.set_max_rotation_impulse(IVP_CFE_BREAK, 0.4f);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }

    c2_add(scene, anchor1, "anchor");
    c2_add(scene, a1, "breaker");
    c2_add(scene, anchor2, "anchor");
    c2_add(scene, b1, "clipper");
    c2_add(scene, anchor3, "anchor");
    c2_add(scene, door, "door");
    c2_add(scene, hammer, "hammer");
}

/* ===================================================================== */
/* attacher                                                               */
/* ===================================================================== */

class HoverAttachment : public RunnerFloating {
    IVP_Attacher_To_Cores<HoverAttachment> *attacher;
    IVP_Core *attached_core;

    static const IVP_Template_Controller_Floating *templ() {
        static IVP_Template_Controller_Floating t;
        static bool init = false;
        if (!init) {
            init = true;
            t.max_repulsive_force = 25.0f;
            t.max_adhesive_force = 10.0f;
            t.position_os.set(0.0, 0.4, 0.0);
            t.target_distance = 1.2f;
        }
        return &t;
    }

public:
    HoverAttachment(IVP_Attacher_To_Cores<HoverAttachment> *a, IVP_Core *core)
        : RunnerFloating(core->objects.element_at(0), templ()), attacher(a), attached_core(core) {}
    ~HoverAttachment() { attacher->attachment_is_going_to_be_deleted(this, attached_core); }
};

struct AttacherScenario {
    IVP_U_Set_Active<IVP_Core> *set;
    IVP_Attacher_To_Cores<HoverAttachment> *attacher;
    IVP_Real_Object *box[4];
};

void attacher_step_hook(int step, IVP_Environment *, ScenarioResources *res) {
    AttacherScenario *s = (AttacherScenario *)res->scenario_data;
    if (step == 121) s->set->install_element(s->box[2]->get_core());
    if (step == 241) s->set->remove_element(s->box[0]->get_core());
    if (step == 361) s->set->install_element(s->box[3]->get_core());
}

void attacher_cleanup_hook(ScenarioResources *res) {
    AttacherScenario *s = (AttacherScenario *)res->scenario_data;
    if (!s) return;
    delete s->set; // deletes the attachments and the attacher
    delete s;
    res->scenario_data = 0;
}

void setup_attacher(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    c2_add(scene, c2_ground(env, &mat_ground), "ground");

    AttacherScenario *s = new AttacherScenario;
    for (int i = 0; i < 4; ++i) {
        s->box[i] = c2_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.0, 0.107, 0.107, 0.107,
                           -6.0 + 4.0 * (double)i, 0.0, 0.0, 0);
        c2_add(scene, s->box[i], "box");
    }
    s->set = new IVP_U_Set_Active<IVP_Core>(16);
    s->set->install_element(s->box[0]->get_core());
    s->set->install_element(s->box[1]->get_core());
    s->attacher = new IVP_Attacher_To_Cores<HoverAttachment>(s->set);
    res->scenario_data = s;
    res->step_hook = attacher_step_hook;
    res->cleanup_hook = attacher_cleanup_hook;
}

/* ===================================================================== */
/* actuator_extra                                                         */
/* ===================================================================== */

struct ExtraScenario {
    IVP_Actuator_Extra *cam, *puck;
};

void extra_cleanup_hook(ScenarioResources *res) {
    ExtraScenario *s = (ExtraScenario *)res->scenario_data;
    if (!s) return;
    delete s->cam;
    delete s->puck;
    delete s;
    res->scenario_data = 0;
}

void setup_actuator_extra(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    IVP_Polygon *ground = c2_ground(env, &mat_ground);
    c2_add(scene, ground, "ground");

    IVP_Polygon *cam_box = c2_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.0, 0.107, 0.107, 0.107, -3.0, 1.9, 0.0, 0);
    IVP_Polygon *rest_box = c2_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.0, 0.107, 0.107, 0.107, 3.0, 1.9, 0.0, 0);
    IVP_Ball *p0 = c2_ball(env, &mat_dyn, 0.3, 0.5, 0.018, -1.0, 2.2, 3.0);
    IVP_Ball *p1 = c2_ball(env, &mat_dyn, 0.3, 0.5, 0.018, 1.0, 2.2, 3.0);
    p0->get_core()->speed.set(1.0f, 0.0f, 0.0f);
    p1->get_core()->speed.set(-0.5f, 0.0f, 0.5f);

    ExtraScenario *s = new ExtraScenario;
    {
        IVP_Template_Anchor a0, a1;
        IVP_Template_Extra te;
        te.anchors[0] = &a0;
        te.anchors[1] = &a1;
        a0.set_anchor_position_os(cam_box, 0.0, 0.0, 0.0);
        a1.set_anchor_position_os(ground, 0.0, -0.5, -4.0);
        te.info.is_float_cam = IVP_TRUE;
        s->cam = new IVP_Actuator_Extra(env, &te);
    }
    {
        IVP_Template_Anchor a0, a1;
        IVP_Template_Extra te;
        te.anchors[0] = &a0;
        te.anchors[1] = &a1;
        a0.set_anchor_position_os(p0, 0.0, 0.0, 0.0);
        a1.set_anchor_position_os(p1, 0.0, 0.0, 0.0);
        te.info.is_puck_force = 1;
        s->puck = new IVP_Actuator_Extra(env, &te);
    }
    res->scenario_data = s;
    res->cleanup_hook = extra_cleanup_hook;

    c2_add(scene, cam_box, "cam_box");
    c2_add(scene, rest_box, "rest_box");
    c2_add(scene, p0, "puck");
    c2_add(scene, p1, "puck");
}

/* ===================================================================== */
/* airboat                                                                */
/* ===================================================================== */

const double C2_BEACH_EDGE_X = -2.0; // ground for x < edge, water for x >= edge
const double C2_WATER_Y = 2.5;       // water surface (y points down), level with the beach

// do_raycasts_gameside: the ground plane y = C2_GROUND_TOP on the beach,
// water (surface y = C2_WATER_Y) beyond its edge.
class RunnerAirboat : public IVP_Controller_Raycast_Airboat {
public:
    RunnerAirboat(IVP_Environment *env, const IVP_Template_Car_System *t)
        : IVP_Controller_Raycast_Airboat(env, t) {}
    // IVP_Car_System pure virtuals the reference vehicle class leaves open
    // (never called by its simulation)
    using IVP_Controller_Raycast_Airboat::do_steering;
    void do_steering(IVP_FLOAT angle, bool) { IVP_Controller_Raycast_Airboat::do_steering(angle); }
    void update_wheel_positions() {}
    void set_powerslide(IVP_FLOAT, IVP_FLOAT) {}
    IVP_FLOAT get_booster_time_to_go() { return 0.0f; }
protected:
    void do_raycasts_gameside(int nRaycastCount, IVP_Ray_Solver_Template *pRays, IVP_Raycast_Airboat_Impact *pImpacts) {
        for (int i = 0; i < nRaycastCount; ++i) {
            const IVP_U_Point &start = pRays[i].ray_start_point;
            const IVP_U_Float_Point &dir = pRays[i].ray_normized_direction;
            IVP_DOUBLE len = pRays[i].ray_length;
            IVP_Raycast_Airboat_Impact &imp = pImpacts[i];
            imp.bImpact = IVP_FALSE;
            imp.bImpactWater = IVP_FALSE;
            imp.bInWater = IVP_FALSE;
            imp.vecImpactPointWS.set_to_zero();
            imp.vecImpactNormalWS.set_to_zero();
            imp.flFriction = 0.0f;
            imp.flDampening = 0.0f;
            if (start.k[0] < C2_BEACH_EDGE_X) {
                if (dir.k[1] > 1e-6f) {
                    IVP_DOUBLE t = (C2_GROUND_TOP - start.k[1]) / dir.k[1];
                    if (t >= 0.0 && t <= len) {
                        imp.bImpact = IVP_TRUE;
                        imp.vecImpactPointWS.set(start.k[0] + dir.k[0] * t, start.k[1] + dir.k[1] * t, start.k[2] + dir.k[2] * t);
                        imp.vecImpactNormalWS.set(0.0f, -1.0f, 0.0f);
                        imp.flFriction = 0.8f;
                    }
                }
            } else {
                imp.bInWater = start.k[1] > C2_WATER_Y ? IVP_TRUE : IVP_FALSE;
                if (dir.k[1] > 1e-6f) {
                    IVP_DOUBLE t = (C2_WATER_Y - start.k[1]) / dir.k[1];
                    if (t >= 0.0 && t <= len) imp.bImpactWater = IVP_TRUE;
                }
            }
        }
    }
};

struct AirboatScenario {
    RunnerAirboat *land, *water;
};

void airboat_step_hook(int step, IVP_Environment *, ScenarioResources *res) {
    AirboatScenario *s = (AirboatScenario *)res->scenario_data;
    IVP_Controller_Raycast_Airboat *a = s->land;
    IVP_Controller_Raycast_Airboat *b = s->water;
    if (step == 91) {
        a->update_throttle(0.6f);                               // does not wake the body
        a->IVP_Controller_Raycast_Airboat::do_steering(0.005f); // wakes it (below the 0.01 steering threshold)
        b->update_throttle(0.15f);
    }
    if (step == 301) {
        a->IVP_Controller_Raycast_Airboat::do_steering(0.3f);
        b->IVP_Controller_Raycast_Airboat::do_steering(-0.3f);
    }
    if (step == 341) {
        a->IVP_Controller_Raycast_Airboat::do_steering(0.0f);
        b->IVP_Controller_Raycast_Airboat::do_steering(0.0f);
    }
    if (step == 501) {
        a->IVP_Controller_Raycast_Airboat::do_steering(-0.2f);
        a->update_throttle(-0.3f);
        b->update_throttle(0.0f);
    }
    if (step == 531) a->IVP_Controller_Raycast_Airboat::do_steering(0.0f);
}

void airboat_cleanup_hook(ScenarioResources *res) {
    AirboatScenario *s = (AirboatScenario *)res->scenario_data;
    if (!s) return;
    delete s->land;
    delete s->water;
    delete s;
    res->scenario_data = 0;
}

RunnerAirboat *c2_airboat(IVP_Environment *env, IVP_Real_Object *body) {
    IVP_Template_Car_System tcs(4, 2);
    tcs.car_body = body;
    for (int i = 0; i < 4; ++i) {
        const double wx = (i & 1) ? 0.7 : -0.7;  // odd = right
        const double wz = (i & 2) ? -1.3 : 1.3;  // 2,3 = rear
        tcs.wheel_pos_Bos[i].set((IVP_FLOAT)wx, 0.25f, (IVP_FLOAT)wz);
        tcs.trace_pos_Bos[i].set((IVP_FLOAT)wx, 0.25f, (IVP_FLOAT)wz);
        tcs.wheel_radius[i] = 0.3f;
        tcs.spring_constant[i] = 10000.0f;
        tcs.spring_dampening[i] = 800.0f;
        tcs.spring_dampening_compression[i] = 800.0f;
        tcs.spring_pre_tension[i] = 0.0f;
    }
    tcs.stabilizer_constant[0] = 1000.0f;
    tcs.stabilizer_constant[1] = 1000.0f;
    tcs.wheel_max_rotation_speed[0] = 50.0f;
    tcs.wheel_max_rotation_speed[1] = 50.0f;
    tcs.extra_gravity_force_value = 0.0f;
    tcs.body_down_force_vertical_offset = 0.2f;
    return new RunnerAirboat(env, &tcs);
}

void setup_airboat(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    IVP_Polygon *beach = c2_static(env, &mat_ground, 14.0, 0.5, 20.0, -16.0, 3.0, 0.0);
    c2_add(scene, beach, "beach");

    // land boat on the beach (forward +z), water boat afloat (forward +x)
    IVP_Polygon *land = c2_box(env, &mat_dyn, 0.8, 0.25, 1.5, 300.0, 231.0, 289.0, 70.0, -16.0, 1.95, -6.0, 0);
    IVP_U_Quat q; set_quat_axis_angle(&q, 0.0, 1.0, 0.0, 1.5707963267948966);
    IVP_Polygon *water = c2_box(env, &mat_dyn, 0.8, 0.25, 1.5, 300.0, 231.0, 289.0, 70.0, 6.0, 2.25, 0.0, &q);

    AirboatScenario *s = new AirboatScenario;
    s->land = c2_airboat(env, land);
    s->water = c2_airboat(env, water);
    res->scenario_data = s;
    res->step_hook = airboat_step_hook;
    res->cleanup_hook = airboat_cleanup_hook;

    c2_add(scene, land, "airboat");
    c2_add(scene, water, "airboat");
}

/* ===================================================================== */
/* fake_jetski                                                            */
/* ===================================================================== */

// do_raycasts: the ground plane y = C2_GROUND_TOP (the ground box).
class RunnerJetski : public IVP_Controller_Raycast_Fake_Jetski {
    IVP_Real_Object *ground;
public:
    RunnerJetski(IVP_Environment *env, const IVP_Template_Car_System *t, IVP_Real_Object *g)
        : IVP_Controller_Raycast_Fake_Jetski(env, t), ground(g) {}
    // IVP_Car_System pure virtuals the reference vehicle class leaves open
    // (never called by its simulation)
    using IVP_Controller_Raycast_Fake_Jetski::do_steering;
    void do_steering(IVP_FLOAT angle, bool) { IVP_Controller_Raycast_Fake_Jetski::do_steering(angle); }
    void update_wheel_positions() {}
    void set_powerslide(IVP_FLOAT, IVP_FLOAT) {}
    IVP_FLOAT get_booster_time_to_go() { return 0.0f; }
    void update_throttle(IVP_FLOAT) {}
protected:
    void do_raycasts(IVP_Event_Sim *, int n_wheels_in, IVP_Ray_Solver_Template *t_in,
                     class IVP_Ray_Hit *hits_out, IVP_FLOAT *friction_of_object_out) {
        for (int i = 0; i < n_wheels_in; ++i) {
            const IVP_U_Point &start = t_in[i].ray_start_point;
            const IVP_U_Float_Point &dir = t_in[i].ray_normized_direction;
            IVP_Ray_Hit &hit = hits_out[i];
            hit.hit_real_object = NULL;
            hit.hit_compact_ledge = NULL;
            hit.hit_compact_triangle = NULL;
            hit.hit_surface_direction_os.set_to_zero();
            hit.hit_distance = 0.0f;
            friction_of_object_out[i] = 1.0f;
            if (dir.k[1] > 1e-6f) {
                IVP_DOUBLE t = (C2_GROUND_TOP - start.k[1]) / dir.k[1];
                if (t >= 0.0 && t <= t_in[i].ray_length) {
                    hit.hit_real_object = ground;
                    hit.hit_surface_direction_os.set(0.0f, -1.0f, 0.0f);
                    hit.hit_distance = (IVP_FLOAT)t;
                    friction_of_object_out[i] = 0.8f;
                }
            }
        }
    }
};

struct JetskiScenario {
    RunnerJetski *ski;
};

void jetski_step_hook(int step, IVP_Environment *, ScenarioResources *res) {
    JetskiScenario *s = (JetskiScenario *)res->scenario_data;
    RunnerJetski *k = s->ski;
    if (step == 61) {
        for (int i = 0; i < 4; ++i) k->fix_wheel(IVP_POS_WHEEL(i), IVP_FALSE);
        k->change_wheel_torque(IVP_REAR_LEFT, 600.0f);
        k->change_wheel_torque(IVP_REAR_RIGHT, 600.0f);
    }
    if (step == 241) k->IVP_Controller_Raycast_Fake_Jetski::do_steering(0.3f);
    if (step == 361) k->activate_booster(8.0f, 0.5f, 1.0f);
    if (step == 481) {
        k->change_wheel_torque(IVP_REAR_LEFT, 0.0f);
        k->change_wheel_torque(IVP_REAR_RIGHT, 0.0f);
        k->IVP_Controller_Raycast_Fake_Jetski::do_steering(-0.2f);
    }
    if (step == 601) {
        k->fix_wheel(IVP_REAR_LEFT, IVP_TRUE);
        k->fix_wheel(IVP_REAR_RIGHT, IVP_TRUE);
    }
}

void jetski_cleanup_hook(ScenarioResources *res) {
    JetskiScenario *s = (JetskiScenario *)res->scenario_data;
    if (!s) return;
    delete s->ski;
    delete s;
    res->scenario_data = 0;
}

void setup_fake_jetski(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    IVP_Polygon *ground = c2_ground(env, &mat_ground);
    c2_add(scene, ground, "ground");

    IVP_Polygon *body = c2_box(env, &mat_dyn, 0.5, 0.3, 1.2, 250.0, 132.0, 141.0, 28.0, 0.0, 1.6, -6.0, 0);

    IVP_Template_Car_System tcs(4, 2);
    tcs.car_body = body;
    for (int i = 0; i < 4; ++i) {
        const double wx = (i & 1) ? 0.45 : -0.45;
        const double wz = (i & 2) ? -0.9 : 0.9;
        tcs.wheel_pos_Bos[i].set((IVP_FLOAT)wx, 0.3f, (IVP_FLOAT)wz);
        tcs.trace_pos_Bos[i].set((IVP_FLOAT)wx, 0.3f, (IVP_FLOAT)wz);
        tcs.wheel_radius[i] = 0.3f;
        tcs.spring_constant[i] = 15000.0f;
        tcs.spring_dampening[i] = 1500.0f;
        tcs.spring_dampening_compression[i] = 1500.0f;
        tcs.spring_pre_tension[i] = -0.35f;
    }
    tcs.stabilizer_constant[0] = 2000.0f;
    tcs.stabilizer_constant[1] = 2000.0f;
    tcs.wheel_max_rotation_speed[0] = 50.0f;
    tcs.wheel_max_rotation_speed[1] = 50.0f;
    tcs.extra_gravity_force_value = 500.0f;
    tcs.body_down_force_vertical_offset = 0.2f;

    JetskiScenario *s = new JetskiScenario;
    s->ski = new RunnerJetski(env, &tcs, ground);
    res->scenario_data = s;
    res->step_hook = jetski_step_hook;
    res->cleanup_hook = jetski_cleanup_hook;

    c2_add(scene, body, "jetski");
}

} // namespace

bool setup_controllers2_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    switch (scenario) {
        case SCENARIO_TORQUE: setup_torque(env, scene, resources); return true;
        case SCENARIO_STABILIZER: setup_stabilizer(env, scene, resources); return true;
        case SCENARIO_FLOATING: setup_floating(env, scene, resources); return true;
        case SCENARIO_WORLD_FRICTION: setup_world_friction(env, scene, resources); return true;
        case SCENARIO_GOLEM: setup_golem(env, scene, resources); return true;
        case SCENARIO_FIXED_KEYED: setup_fixed_keyed(env, scene, resources); return true;
        case SCENARIO_HINGE_LIMITS: setup_hinge_limits(env, scene); return true;
        case SCENARIO_CARDAN_TENSE: setup_cardan_tense(env, scene); return true;
        case SCENARIO_SLIDER_LIMITS: setup_slider_limits(env, scene); return true;
        case SCENARIO_CONSTRAINT_BREAK: setup_constraint_break(env, scene); return true;
        case SCENARIO_ATTACHER: setup_attacher(env, scene, resources); return true;
        case SCENARIO_ACTUATOR_EXTRA: setup_actuator_extra(env, scene, resources); return true;
        case SCENARIO_AIRBOAT: setup_airboat(env, scene, resources); return true;
        case SCENARIO_FAKE_JETSKI: setup_fake_jetski(env, scene, resources); return true;
        default: return false;
    }
}

} // namespace ref_runner
