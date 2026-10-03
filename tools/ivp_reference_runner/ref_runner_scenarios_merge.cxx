// ref_runner_scenarios_merge.cxx -- parity scenarios for shared cores:
//   merge_objects   IVP_Environment::merge_objects (IVP_Core_Merged /
//                   IVP_Merge_Core): a box + box + ball compound merged at
//                   setup, two hanging boxes merged mid-run, a landed box that
//                   is frozen with disable_simulation and merged with a
//                   hanging ball mid-run; the merged bodies are pushed / spun
//                   with async pushes and hit the ground and other bodies.
//   object_attach   IVP_Object_Attach: attach_object (current distance and an
//                   explicit one) of a flying box (its spring actuator is
//                   deleted with its core) and a hanging ball to a spinning
//                   parent and of a flying ball to a frozen box,
//                   reposition_object_Ros, detach_object of the attached box
//                   and ball and of the frozen box (whose core keeps the
//                   ball attached to it).
//   merge_buoyancy  IVP_Attacher_To_Cores_Buoyancy on a merged raft (box +
//                   box + ball) and on a box that gets a ball attached:
//                   IVP_Controller_Buoyancy computes buoyancy and dampening
//                   for every object of the core.
// Mirrored by the run_* functions of the "merge" block in libivp's
// tests/test_scenarios.c; keep both in sync.  Merges are only done on
// objects that are not simulated and have no contact points, the inputs on
// which IVP_Environment::merge_objects is safe (simulated cores loop forever
// in fast_normize_quat: the merged core's q_world_f_core_next_psi is never
// set; friction systems keep pointers to the deleted cores).
//
// IVP_Object_Attach (libivp_physics.a: ivp_object_attach.cxx.o) calls
// IVP_Core::inline_calc_at_position / inline_calc_at_quaternion and
// IVP_Hull_Manager::check_hull_synapses without including their inline
// definitions (ivp_core_macros.hxx, ivp_hull_manager_macros.hxx), so the
// library does not link on its own; this file emits out-of-line copies of
// the three reference inline functions (see ref_emit_* below).

#include "ref_runner.hxx"

#include <ivp_object_attach.hxx>
#include <ivp_controller_buoyancy.hxx>
#include <ivp_liquid_surface_descript.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_hull_manager.hxx>
#include <ivp_hull_manager_macros.hxx>


/* out-of-line definitions of the inline functions ivp_object_attach.cxx.o
 * references (taking their address emits them) */
extern void (IVP_Core::*const ref_emit_calc_at_quaternion)(IVP_Time, IVP_U_Quat *) const;
void (IVP_Core::*const ref_emit_calc_at_quaternion)(IVP_Time, IVP_U_Quat *) const = &IVP_Core::inline_calc_at_quaternion;
extern void (IVP_Core::*const ref_emit_calc_at_position)(IVP_Time, IVP_U_Point *) const;
void (IVP_Core::*const ref_emit_calc_at_position)(IVP_Time, IVP_U_Point *) const = &IVP_Core::inline_calc_at_position;
extern void (IVP_Hull_Manager::*const ref_emit_check_hull_synapses)(IVP_Environment *);
void (IVP_Hull_Manager::*const ref_emit_check_hull_synapses)(IVP_Environment *) = &IVP_Hull_Manager::check_hull_synapses;

namespace ref_runner {

namespace {

void mg_add(SceneObjects *scene, IVP_Real_Object *o, const char *type) {
    scene->objects[scene->count] = o;
    scene->types[scene->count] = type;
    scene->count++;
}

/* dynamic template of the runner's create_dynamic_box / create_dynamic_ball */
void mg_template(IVP_Template_Real_Object *t, IVP_Material *mat, double mass, double ix, double iy, double iz) {
    configure_dynamic_template(t, mat, mass);
    configure_explicit_inertia(t, ix, iy, iz);
    t->speed_damp_factor = 0.0;
    t->rot_speed_damp_factor.set(0.0, 0.0, 0.0);
}

/* create_dynamic_box without waking the object when !wake */
IVP_Polygon *mg_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz, double mass,
                    double ix, double iy, double iz, double x, double y, double z, const IVP_U_Quat *q, bool wake) {
    IVP_Compact_Surface *compact = build_box_compact_surface(hx, hy, hz);
    IVP_SurfaceManager_Polygon *surman = new IVP_SurfaceManager_Polygon(compact);
    IVP_Template_Real_Object templ;
    mg_template(&templ, mat, mass, ix, iy, iz);
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point p; p.set(x, y, z);
    IVP_Polygon *o = env->create_polygon(surman, &templ, q ? q : &qi, &p);
    o->enable_collision_detection(IVP_TRUE);
    if (wake) o->ensure_in_simulation_now();
    return o;
}

IVP_Ball *mg_ball(IVP_Environment *env, IVP_Material *mat, double r, double mass, double inertia,
                  double x, double y, double z, bool wake) {
    IVP_Template_Real_Object templ;
    mg_template(&templ, mat, mass, inertia, inertia, inertia);
    IVP_Template_Ball tb;
    tb.radius = (IVP_FLOAT)r;
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point p; p.set(x, y, z);
    IVP_Ball *o = env->create_ball(&tb, &templ, &qi, &p);
    o->enable_collision_detection(IVP_TRUE);
    if (wake) o->ensure_in_simulation_now();
    return o;
}

IVP_Polygon *mg_static(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                       double x, double y, double z, const IVP_U_Quat *q) {
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point p; p.set(x, y, z);
    return create_static_box(env, mat, hx, hy, hz, q ? q : &qi, &p);
}

void mg_merge(IVP_Environment *env, IVP_Real_Object *const *objs, int n) {
    IVP_U_Vector<IVP_Real_Object> v;
    for (int i = 0; i < n; i++) v.add(objs[i]);
    env->merge_objects(&v);
}

void mg_push(IVP_Real_Object *o, float vx, float vy, float vz, float wx, float wy, float wz) {
    IVP_U_Float_Point v; v.set(vx, vy, vz);
    o->async_add_speed_object_ws(&v);
    IVP_U_Float_Point w; w.set(wx, wy, wz);
    o->async_add_rot_speed_object_cs(&w);
}

/* ===================================================================== */
/* merge_objects                                                          */
/* ===================================================================== */

struct MergeScenario {
    IVP_Real_Object *b1, *b2;   /* hanging boxes, merged mid-run */
    IVP_Real_Object *c1, *c2;   /* landed box (frozen mid-run) + hanging ball */
    IVP_Real_Object *heavy;
    bool b_done, c_frozen, c_done;
};

void merge_step_hook(int step, IVP_Environment *env, ScenarioResources *res) {
    MergeScenario *s = (MergeScenario *)res->scenario_data;
    const double t = env->get_current_time().get_time();
    if (step == 1) {
        /* the setup compound: speed and spin (applied at the first PSI) */
        mg_push(s->heavy, 3.0f, -0.5f, 0.6f, 0.3f, 2.5f, -0.8f);
    }
    if (!s->b_done && t >= 0.75) {
        s->b_done = true;
        IVP_Real_Object *objs[2] = { s->b2, s->b1 };
        mg_merge(env, objs, 2);
        mg_push(s->b1, -0.5f, 0.0f, 0.3f, 1.2f, -0.6f, 3.0f);
    }
    if (!s->c_frozen && t >= 0.5) {
        s->c_frozen = true;
        s->c1->disable_simulation();
    }
    if (s->c_frozen && !s->c_done && t >= 0.6) {
        s->c_done = true;
        if (s->c1->get_movement_state() == IVP_MT_NOT_SIM && !s->c1->get_first_friction_synapse()) {
            IVP_Real_Object *objs[2] = { s->c1, s->c2 };
            mg_merge(env, objs, 2);
            mg_push(s->c2, -1.0f, -1.5f, 0.0f, 0.0f, 0.0f, 4.0f);
        }
    }
}

void merge_cleanup_hook(ScenarioResources *res) {
    delete (MergeScenario *)res->scenario_data;
    res->scenario_data = 0;
}

void setup_merge_objects(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.6, 0.2);
    static IVP_Material_Simple mat_dyn(0.4, 0.3);
    MergeScenario *s = new MergeScenario;
    s->b_done = s->c_frozen = s->c_done = false;

    mg_add(scene, mg_static(env, &mat_ground, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, 0), "ground");
    IVP_U_Quat qw; set_quat_axis_angle(&qw, 0.0, 1.0, 0.0, 0.4);
    mg_add(scene, mg_static(env, &mat_ground, 0.3, 0.6, 2.0, 4.5, 1.9, 0.0, &qw), "wall");

    /* compound merged at setup: heavy box (main axis), rotated light box, ball */
    IVP_U_Quat qh; set_quat_axis_angle(&qh, 0.0, 0.0, 1.0, 0.15);
    IVP_Polygon *heavy = mg_box(env, &mat_dyn, 0.5, 0.25, 0.4, 4.0, 0.29, 0.54, 0.41, 0.0, 1.6, 0.0, &qh, false);
    IVP_U_Quat ql; set_quat_axis_angle(&ql, 0.0, 0.0, 1.0, 0.5);
    IVP_Polygon *light = mg_box(env, &mat_dyn, 0.2, 0.2, 0.2, 1.0, 0.027, 0.027, 0.027, 0.85, 1.5, 0.1, &ql, false);
    IVP_Ball *ball = mg_ball(env, &mat_dyn, 0.25, 0.8, 0.02, -0.8, 1.55, -0.2, false);
    {
        IVP_Real_Object *objs[3] = { heavy, light, ball };
        mg_merge(env, objs, 3);
    }
    s->heavy = heavy;
    /* a spring from a static hook to the light box (anchor in a merged object) */
    IVP_Polygon *hook = mg_static(env, &mat_ground, 0.1, 0.1, 0.1, 1.5, -0.5, 0.0, 0);
    add_spring(env, hook, 0.0, 0.1, 0.0, light, 0.1, -0.1, 0.0, 15.0, 1.5, 1.6);
    mg_add(scene, hook, "hook");
    mg_add(scene, heavy, "heavy");
    mg_add(scene, light, "light");
    mg_add(scene, ball, "ball");

    /* free bodies on the ground in the compound's way */
    mg_add(scene, mg_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.2, 0.072, 0.072, 0.072, 2.2, 2.15, 0.5, 0, true), "free_box");
    mg_add(scene, mg_ball(env, &mat_dyn, 0.3, 1.0, 0.036, 1.6, 2.1, -0.6, true), "free_ball");

    /* two hanging boxes, merged mid-run */
    IVP_U_Quat qb; set_quat_axis_angle(&qb, 1.0, 0.0, 0.0, 0.6);
    s->b1 = mg_box(env, &mat_dyn, 0.35, 0.2, 0.3, 1.5, 0.065, 0.11, 0.075, -3.0, 0.5, 0.0, 0, false);
    s->b2 = mg_box(env, &mat_dyn, 0.15, 0.15, 0.35, 0.7, 0.034, 0.034, 0.011, -2.45, 0.35, 0.25, &qb, false);
    mg_add(scene, s->b1, "b1");
    mg_add(scene, s->b2, "b2");

    /* a box sliding on the ground (frozen mid-run with disable_simulation,
     * which removes its contact points) and a hanging ball */
    s->c1 = mg_box(env, &mat_dyn, 0.3, 0.2, 0.3, 1.0, 0.043, 0.06, 0.043, -2.0, 2.25, 3.0, 0, true);
    s->c1->get_core()->speed.set(2.5f, 0.0f, 0.2f);
    s->c2 = mg_ball(env, &mat_dyn, 0.2, 0.5, 0.008, -0.4, 1.6, 3.4, false);
    mg_add(scene, s->c1, "c1");
    mg_add(scene, s->c2, "c2");

    res->scenario_data = s;
    res->step_hook = merge_step_hook;
    res->cleanup_hook = merge_cleanup_hook;
}

/* ===================================================================== */
/* object_attach                                                          */
/* ===================================================================== */

struct AttachScenario {
    IVP_Real_Object *parent, *box, *ball, *free_box, *proj;
    IVP_Material *mat;
    int done;
};

void attach_step_hook(int step, IVP_Environment *env, ScenarioResources *res) {
    (void)step;
    AttachScenario *s = (AttachScenario *)res->scenario_data;
    const double t = env->get_current_time().get_time();
    if (s->done == 0 && t >= 0.2) {
        s->done = 1;
        IVP_Object_Attach::attach_object(s->parent, s->box, -1.0f);
    } else if (s->done == 1 && t >= 0.3) {
        s->done = 2;
        IVP_Object_Attach::attach_object(s->parent, s->ball, 1.5);   /* the hanging ball is revived first */
    } else if (s->done == 2 && t >= 0.5) {
        s->done = 3;
        IVP_U_Quat q; set_quat_axis_angle(&q, 0.0, 1.0, 0.0, 0.35);
        IVP_U_Point shift; shift.set(0.75, -0.1, 0.2);
        IVP_Object_Attach::reposition_object_Ros(s->parent, s->box, &q, &shift, IVP_FALSE);
    } else if (s->done == 3 && t >= 0.55) {
        s->done = 31;
        IVP_Object_Attach::attach_object(s->free_box, s->proj, -1.0f);   /* the frozen parent is revived */
    } else if (s->done == 31 && t >= 1.8) {
        s->done = 4;
        IVP_Template_Real_Object templ;
        mg_template(&templ, s->mat, 1.1, 0.03, 0.03, 0.03);
        IVP_Object_Attach::detach_object(s->box, &templ);
    } else if (s->done == 4 && t >= 2.4) {
        s->done = 5;
        IVP_Template_Real_Object templ;
        mg_template(&templ, s->mat, 2.5, 0.11, 0.11, 0.11);
        IVP_Object_Attach::detach_object(s->free_box, &templ);
    } else if (s->done == 5 && t >= 2.7) {
        s->done = 6;
        IVP_Template_Real_Object templ;
        mg_template(&templ, s->mat, 0.6, 0.015, 0.015, 0.015);
        templ.extra_radius = 0.25f;   /* detach_object: the ball radius comes from extra_radius only */
        IVP_Object_Attach::detach_object(s->ball, &templ);
    }
}

void attach_cleanup_hook(ScenarioResources *res) {
    delete (AttachScenario *)res->scenario_data;
    res->scenario_data = 0;
}

void setup_object_attach(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.7, 0.1);
    static IVP_Material_Simple mat_dyn(0.5, 0.2);
    AttachScenario *s = new AttachScenario;
    s->mat = &mat_dyn;
    s->done = 0;

    mg_add(scene, mg_static(env, &mat_ground, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, 0), "ground");

    IVP_U_Quat qp; set_quat_axis_angle(&qp, 1.0, 0.0, 0.0, 0.2);
    IVP_Polygon *parent = mg_box(env, &mat_dyn, 0.6, 0.2, 0.4, 3.0, 0.2, 0.52, 0.4, 0.0, 0.8, 0.0, &qp, true);
    parent->get_core()->speed.set(1.0f, -1.0f, 0.0f);
    parent->get_core()->rot_speed.set(0.0f, 2.0f, 0.5f);
    IVP_U_Quat qb; set_quat_axis_angle(&qb, 0.0, 0.0, 1.0, 0.3);
    IVP_Polygon *box = mg_box(env, &mat_dyn, 0.2, 0.2, 0.2, 1.0, 0.027, 0.027, 0.027, 1.6, 0.4, 0.3, &qb, true);
    box->get_core()->speed.set(0.5f, -2.0f, 0.0f);
    IVP_Ball *ball = mg_ball(env, &mat_dyn, 0.25, 0.6, 0.015, -1.0, 0.75, -0.5, false);   /* hanging */
    IVP_Polygon *free_box = mg_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.2, 0.072, 0.072, 0.072, 3.0, 2.15, 1.0, 0, true);
    IVP_Ball *proj = mg_ball(env, &mat_dyn, 0.15, 0.4, 0.004, 4.6, 0.2, 1.2, true);
    proj->get_core()->speed.set(-1.2f, -1.5f, 0.0f);
    /* a spring from a static hook to the box: deleted with the box's core at attach_object */
    IVP_Polygon *hook = mg_static(env, &mat_ground, 0.1, 0.1, 0.1, 2.5, -0.6, 0.3, 0);
    add_spring(env, hook, 0.0, 0.1, 0.0, box, 0.0, -0.2, 0.0, 20.0, 1.0, 0.8);

    s->parent = parent;
    s->box = box;
    s->ball = ball;
    s->free_box = free_box;
    s->proj = proj;
    mg_add(scene, parent, "parent");
    mg_add(scene, box, "box");
    mg_add(scene, ball, "ball");
    mg_add(scene, free_box, "free_box");
    mg_add(scene, proj, "proj");
    mg_add(scene, hook, "hook");

    res->scenario_data = s;
    res->step_hook = attach_step_hook;
    res->cleanup_hook = attach_cleanup_hook;
}

/* ===================================================================== */
/* merge_buoyancy                                                         */
/* ===================================================================== */

struct MergeBuoyancyScenario {
    IVP_Real_Object *raft_a, *parent, *child;
    int done;
};

void merge_buoyancy_step_hook(int step, IVP_Environment *env, ScenarioResources *res) {
    MergeBuoyancyScenario *s = (MergeBuoyancyScenario *)res->scenario_data;
    const double t = env->get_current_time().get_time();
    if (step == 1) {
        mg_push(s->raft_a, 0.5f, 0.0f, 0.2f, 0.5f, 1.0f, 0.0f);   /* wakes the merged raft */
    }
    if (s->done == 0 && t >= 0.15) {
        s->done = 1;
        IVP_Object_Attach::attach_object(s->parent, s->child, -1.0f);
    }
}

void merge_buoyancy_cleanup_hook(ScenarioResources *res) {
    delete (MergeBuoyancyScenario *)res->scenario_data;
    res->scenario_data = 0;
}

void setup_merge_buoyancy(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.2);
    MergeBuoyancyScenario *s = new MergeBuoyancyScenario;
    s->done = 0;

    mg_add(scene, mg_static(env, &mat_static, 15.0, 0.5, 15.0, 0.0, 5.0, 0.0, 0), "ground");

    /* a raft of two boxes and a ball merged at setup */
    IVP_Polygon *raft_a = mg_box(env, &mat_dyn, 0.6, 0.15, 0.4, 0.4, 0.025, 0.06, 0.05, -2.0, -2.0, 0.0, 0, false);
    IVP_U_Quat qb; set_quat_axis_angle(&qb, 0.0, 0.0, 1.0, 0.4);
    IVP_Polygon *raft_b = mg_box(env, &mat_dyn, 0.25, 0.25, 0.25, 0.3, 0.0125, 0.0125, 0.0125, -1.3, -2.15, 0.25, &qb, false);
    IVP_Ball *raft_ball = mg_ball(env, &mat_dyn, 0.3, 0.2, 0.0072, -2.8, -2.05, -0.25, false);
    {
        IVP_Real_Object *objs[3] = { raft_a, raft_b, raft_ball };
        mg_merge(env, objs, 3);
    }
    s->raft_a = raft_a;
    mg_add(scene, raft_a, "raft_a");
    mg_add(scene, raft_b, "raft_b");
    mg_add(scene, raft_ball, "raft_ball");

    IVP_Polygon *single = mg_box(env, &mat_dyn, 0.4, 0.4, 0.4, 0.5, 0.053, 0.053, 0.053, 0.5, -3.0, 1.5, 0, true);
    mg_add(scene, single, "single");

    /* a box and a ball falling side by side; the ball is attached to the box */
    IVP_Polygon *parent = mg_box(env, &mat_dyn, 0.5, 0.25, 0.5, 0.6, 0.0375, 0.1, 0.0375, 2.5, -3.5, 0.0, 0, true);
    parent->get_core()->speed.set(0.0f, 1.0f, 0.0f);
    IVP_Ball *child = mg_ball(env, &mat_dyn, 0.25, 0.3, 0.0075, 3.6, -3.4, 0.5, true);
    child->get_core()->speed.set(0.0f, 1.0f, 0.0f);
    s->parent = parent;
    s->child = child;
    mg_add(scene, parent, "parent");
    mg_add(scene, child, "child");

    IVP_Template_Buoyancy templ;
    templ.medium_density = 2.5f;
    templ.pressure_damp_factor = 2.0f;
    templ.friction_damp_factor = 0.1f;
    templ.ball_rot_dampening_factor = 1.0f;
    templ.viscosity_factor = 0.05f;
    templ.use_interpolation = IVP_FALSE;
    IVP_U_Float_Hesse surface(0.0, 1.0, 0.0, 1.0);   /* liquid for y > -1 */
    IVP_U_Float_Point flow(1.0, 0.0, 0.3);
    res->liquid_desc = new IVP_Liquid_Surface_Descriptor_Simple(&surface, &flow);
    res->buoyancy_cores = new IVP_U_Set_Active<IVP_Core>(16);
    res->buoyancy_cores->install_element(raft_a->get_core());   /* the merged core: 3 objects */
    res->buoyancy_cores->install_element(single->get_core());
    res->buoyancy_cores->install_element(parent->get_core());   /* gets the attached ball */
    res->buoyancy_attacher = new IVP_Attacher_To_Cores_Buoyancy(templ, res->buoyancy_cores, res->liquid_desc);

    res->scenario_data = s;
    res->step_hook = merge_buoyancy_step_hook;
    res->cleanup_hook = merge_buoyancy_cleanup_hook;
}

} // namespace

bool setup_merge_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    switch (scenario) {
    case SCENARIO_MERGE_OBJECTS: setup_merge_objects(env, scene, resources); return true;
    case SCENARIO_OBJECT_ATTACH: setup_object_attach(env, scene, resources); return true;
    case SCENARIO_MERGE_BUOYANCY: setup_merge_buoyancy(env, scene, resources); return true;
    default: return false;
    }
}

} // namespace ref_runner
