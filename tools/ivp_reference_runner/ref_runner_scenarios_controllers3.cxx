// ref_runner_scenarios_controllers3.cxx -- parity scenarios for controller
// paths the controllers2 scenes leave out:
//   raycast_car_drive  IVP_Controller_Raycast_Car: a 4 wheel / 2 axis car, a
//                      6 wheel / 3 axis truck and a 3 wheel / 2 axis trike on
//                      analytic ray cast ground (ground box, a ramp, bumps):
//                      throttle, reverse torque, steering left/right,
//                      handbrake (fix_wheel), full brake, booster (incl. a
//                      refused re-ignition and set_booster_acceleration),
//                      wheels in the air (drop at start, ramp jump, bumps),
//                      slow cars without torque (forced fixed wheels)
//   check_dist_events  IVP_Actuator_Check_Dist listeners with effects: a
//                      force switched by a yo-yo's range crossings, a rot
//                      motor reversed / re-powered by a wiper tip, impulses on
//                      a ball sliding between two posts
//   golem_beam         IVP_Controller_Golem resolve_for_problem ->
//                      beam_object_to_target_position (FAR_DISTANCE and
//                      BIG_ANGLE), a shifted core, acos_quat < 0, an
//                      environment reset_time
// The per-step inputs are scheduled in ticks of 1/60 s (tick k = step
// k * (steps per second / 60) + 1), so every dt runs the same program.
// Quaternions come from c3_quat (sin and cos as separate calls).
// Mirrored by the run_* functions of the "controllers3" block in libivp's
// tests/test_scenarios.c; keep both in sync.

#include "ref_runner.hxx"

#include <ivp_actuator.hxx>
#include <ivp_car_system.hxx>
#include <ivp_ray_solver.hxx>
#include <ivp_controller_raycast_car.hxx>
#include <ivp_controller_golem.hxx>

#include <cmath>

// The reference declares IVP_Actuator_Check_Dist::add/remove_listener_
// check_dist_event (ivp_actuator.hxx) but never defines them, so no listener
// can be registered with the legacy library; define them like the other IVP
// listener vectors (IVP_U_Vector add / remove).  libivp's equivalent is
// ivp_check_dist_set_callback.
void IVP_Actuator_Check_Dist::add_listener_check_dist_event(IVP_Listener_Check_Dist_Event *listener) {
    listeners_check_dist_event.add(listener);
}

void IVP_Actuator_Check_Dist::remove_listener_check_dist_event(IVP_Listener_Check_Dist_Event *listener) {
    listeners_check_dist_event.remove(listener);
}

namespace ref_runner {

namespace {

void c3_add(SceneObjects *scene, IVP_Real_Object *o, const char *type) {
    scene->objects[scene->count] = o;
    scene->types[scene->count] = type;
    scene->count++;
}

IVP_Polygon *c3_static(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                       double x, double y, double z, const IVP_U_Quat *q) {
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point p; p.set(x, y, z);
    return create_static_box(env, mat, hx, hy, hz, q ? q : &qi, &p);
}

IVP_Polygon *c3_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                    double mass, double ix, double iy, double iz,
                    double x, double y, double z, const IVP_U_Quat *q) {
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point rd; rd.set(0.0, 0.0, 0.0);
    IVP_U_Point p; p.set(x, y, z);
    return create_dynamic_box(env, mat, hx, hy, hz, mass, ix, iy, iz, 0.0, &rd, q ? q : &qi, &p);
}

// set_quat_axis_angle with sin and cos evaluated by separate calls: clang
// fuses an inline sin/cos pair into __sincos_stret in C++ (1 ulp off for
// some angles, e.g. 0.75), libivp's C test (math-errno) calls sin and cos.
__attribute__((noinline)) double c3_sin(double x) { return std::sin(x); }
__attribute__((noinline)) double c3_cos(double x) { return std::cos(x); }

void c3_quat(IVP_U_Quat *q, double ax, double ay, double az, double radians) {
    const double half = 0.5 * radians;
    const double s = c3_sin(half);
    const double c = c3_cos(half);
    q->x = ax * s;
    q->y = ay * s;
    q->z = az * s;
    q->w = c;
}

// steps per tick of 1/60 s
int c3_steps_per_tick(IVP_Environment *env) {
    const int sps = (int)(1.0 / (double)env->get_delta_PSI_time() + 0.5);
    const int spt = sps / 60;
    return spt < 1 ? 1 : spt;
}

/* ===================================================================== */
/* raycast_car_drive                                                      */
/* ===================================================================== */

// Analytic ray caster: the top faces (object space y = -hy, normal (0,-1,0))
// of a list of static boxes; the closest hit within the ray length wins
// (first box on ties).  The ray is taken to the box's object space with the
// core matrix (static boxes: object == core).
struct C3RaySurface {
    IVP_Real_Object *obj;
    double hx, hy, hz;
    IVP_FLOAT friction;
};

struct C3RayWorld {
    C3RaySurface s[8];
    int n;
};

void c3_cast_rays(const C3RayWorld *w, int n_rays, IVP_Ray_Solver_Template *t_in,
                  IVP_Ray_Hit *hits_out, IVP_FLOAT *friction_out) {
    for (int i = 0; i < n_rays; ++i) {
        const IVP_U_Point &st = t_in[i].ray_start_point;
        const IVP_U_Float_Point &dir = t_in[i].ray_normized_direction;
        const double len = t_in[i].ray_length;
        IVP_Ray_Hit &hit = hits_out[i];
        hit.hit_real_object = NULL;
        hit.hit_compact_ledge = NULL;
        hit.hit_compact_triangle = NULL;
        hit.hit_surface_direction_os.set_to_zero();
        hit.hit_distance = 0.0f;
        friction_out[i] = 1.0f;
        int best = -1;
        double best_t = 0.0;
        for (int j = 0; j < w->n; ++j) {
            const C3RaySurface &s = w->s[j];
            const IVP_U_Matrix *m = s.obj->get_core()->get_m_world_f_core_PSI();
            const double px = st.k[0] - m->vv.k[0];
            const double py = st.k[1] - m->vv.k[1];
            const double pz = st.k[2] - m->vv.k[2];
            const double dx = dir.k[0], dy = dir.k[1], dz = dir.k[2];
            const double ly = m->get_elem(0, 1) * px + m->get_elem(1, 1) * py + m->get_elem(2, 1) * pz;
            const double vy = m->get_elem(0, 1) * dx + m->get_elem(1, 1) * dy + m->get_elem(2, 1) * dz;
            if (!(vy > 1e-6)) continue;
            const double t = (-s.hy - ly) / vy;
            if (t < 0.0 || t > len) continue;
            if (best >= 0 && !(t < best_t)) continue;
            const double lx = m->get_elem(0, 0) * px + m->get_elem(1, 0) * py + m->get_elem(2, 0) * pz;
            const double vx = m->get_elem(0, 0) * dx + m->get_elem(1, 0) * dy + m->get_elem(2, 0) * dz;
            const double hx = lx + vx * t;
            if (hx < -s.hx || hx > s.hx) continue;
            const double lz = m->get_elem(0, 2) * px + m->get_elem(1, 2) * py + m->get_elem(2, 2) * pz;
            const double vz = m->get_elem(0, 2) * dx + m->get_elem(1, 2) * dy + m->get_elem(2, 2) * dz;
            const double hz = lz + vz * t;
            if (hz < -s.hz || hz > s.hz) continue;
            best = j;
            best_t = t;
        }
        if (best >= 0) {
            hit.hit_real_object = w->s[best].obj;
            hit.hit_surface_direction_os.set(0.0f, -1.0f, 0.0f);
            hit.hit_distance = (IVP_FLOAT)best_t;
            friction_out[i] = w->s[best].friction;
        }
    }
}

class RunnerRaycastCar : public IVP_Controller_Raycast_Car {
    const C3RayWorld *world;
public:
    RunnerRaycastCar(IVP_Environment *env, const IVP_Template_Car_System *t, const C3RayWorld *w)
        : IVP_Controller_Raycast_Car(env, t), world(w) {}
    // IVP_Car_System pure virtual the reference class leaves open (never
    // called by its simulation)
    void update_wheel_positions() {}
protected:
    void do_raycasts(IVP_Event_Sim *, int n_wheels_in, IVP_Ray_Solver_Template *t_in,
                     class IVP_Ray_Hit *hits_out, IVP_FLOAT *friction_of_object_out) {
        c3_cast_rays(world, n_wheels_in, t_in, hits_out, friction_of_object_out);
    }
};

struct RaycastCarScenario {
    C3RayWorld world;
    RunnerRaycastCar *car, *truck, *trike;
    int spt;
};

void raycast_car_step_hook(int step, IVP_Environment *, ScenarioResources *res) {
    RaycastCarScenario *s = (RaycastCarScenario *)res->scenario_data;
    RunnerRaycastCar *a = s->car;
    RunnerRaycastCar *b = s->truck;
    RunnerRaycastCar *c = s->trike;
    if ((step - 1) % s->spt) return;
    const int tick = (step - 1) / s->spt;
    switch (tick) {
    case 15:
        a->change_wheel_torque(IVP_REAR_LEFT, 700.0f);
        a->change_wheel_torque(IVP_REAR_RIGHT, 700.0f);
        for (int i = 2; i < 6; ++i) b->change_wheel_torque(IVP_POS_WHEEL(i), 900.0f);
        c->change_wheel_torque(IVP_POS_WHEEL(2), 120.0f);
        break;
    case 40:
        a->do_steering(0.12f);
        break;
    case 70:
        a->do_steering(-0.12f);
        c->do_steering(0.12f);
        break;
    case 95:
        a->do_steering(0.0f);
        b->change_spring_constant(IVP_POS_WHEEL(0), 26000.0f);
        break;
    case 110:
        a->activate_booster(6.0f, 0.4f, 0.8f);
        break;
    case 120:
        b->do_steering(0.1f);
        break;
    case 130:
        c->do_steering(-0.08f);
        break;
    case 140:
        a->activate_booster(9.0f, 0.4f, 0.8f); // refused: not ready
        break;
    case 150:
        a->do_steering(0.15f);
        b->change_stabilizer_constant(IVP_POS_AXIS(1), 0.0f);
        c->change_wheel_torque(IVP_POS_WHEEL(2), 0.0f);
        c->fix_wheel(IVP_POS_WHEEL(2), IVP_TRUE);
        break;
    case 160:
        b->do_steering(-0.1f);
        break;
    case 170:
        a->change_wheel_torque(IVP_REAR_LEFT, -500.0f);
        a->change_wheel_torque(IVP_REAR_RIGHT, -500.0f);
        a->do_steering(-0.12f);
        break;
    case 190:
        a->change_wheel_torque(IVP_REAR_LEFT, 0.0f);
        a->change_wheel_torque(IVP_REAR_RIGHT, 0.0f);
        a->fix_wheel(IVP_REAR_LEFT, IVP_TRUE);
        a->fix_wheel(IVP_REAR_RIGHT, IVP_TRUE);
        a->do_steering(0.2f);
        b->fix_wheel(IVP_POS_WHEEL(4), IVP_TRUE);
        b->fix_wheel(IVP_POS_WHEEL(5), IVP_TRUE);
        b->do_steering(0.0f);
        break;
    case 215:
        a->fix_wheel(IVP_FRONT_LEFT, IVP_TRUE);
        a->fix_wheel(IVP_FRONT_RIGHT, IVP_TRUE);
        break;
    case 230:
        for (int i = 0; i < 4; ++i) a->fix_wheel(IVP_POS_WHEEL(i), IVP_FALSE);
        a->do_steering(0.0f);
        b->fix_wheel(IVP_POS_WHEEL(4), IVP_FALSE);
        b->fix_wheel(IVP_POS_WHEEL(5), IVP_FALSE);
        for (int i = 2; i < 6; ++i) b->change_wheel_torque(IVP_POS_WHEEL(i), 0.0f);
        break;
    case 260:
        a->activate_booster(5.0f, 0.5f, 0.5f);
        c->fix_wheel(IVP_POS_WHEEL(2), IVP_FALSE);
        break;
    case 290:
        c->change_wheel_torque(IVP_POS_WHEEL(2), 180.0f);
        break;
    case 300:
        a->set_booster_acceleration(2.0f);
        b->change_wheel_torque(IVP_POS_WHEEL(2), -600.0f);
        b->change_wheel_torque(IVP_POS_WHEEL(3), -600.0f);
        b->do_steering(0.15f);
        break;
    case 330:
        a->set_booster_acceleration(0.0f);
        a->change_wheel_torque(IVP_REAR_LEFT, 300.0f);
        a->change_wheel_torque(IVP_REAR_RIGHT, 300.0f);
        a->do_steering(-0.1f);
        break;
    case 400:
        a->do_steering(0.08f);
        c->do_steering(0.0f);
        break;
    default:
        break;
    }
}

void raycast_car_cleanup_hook(ScenarioResources *res) {
    RaycastCarScenario *s = (RaycastCarScenario *)res->scenario_data;
    if (!s) return;
    delete s->car;
    delete s->truck;
    delete s->trike;
    delete s;
    res->scenario_data = 0;
}

void setup_raycast_car_drive(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);

    // ground (top y = 2.5), ramp in the car's lane, bumps in the truck's and
    // the trike's lanes (gravity +y: "up" is -y)
    IVP_Polygon *ground = c3_static(env, &mat_ground, 40.0, 0.5, 80.0, 0.0, 3.0, 30.0, 0);
    IVP_U_Quat qr; c3_quat(&qr, 1.0, 0.0, 0.0, 0.11);
    IVP_Polygon *ramp = c3_static(env, &mat_ground, 1.8, 0.5, 2.0, -6.0, 2.79, -5.0, &qr);
    IVP_Polygon *bump0 = c3_static(env, &mat_ground, 0.4, 0.1, 0.3, 6.9, 2.4, -12.5, 0);
    IVP_Polygon *bump1 = c3_static(env, &mat_ground, 0.4, 0.12, 0.5, 5.1, 2.38, -9.5, 0);
    IVP_Polygon *bump2 = c3_static(env, &mat_ground, 0.3, 0.08, 0.6, 0.0, 2.42, -11.0, 0);

    // car: 4 wheels / 2 axes, rear drive
    IVP_Polygon *car_body = c3_box(env, &mat_dyn, 0.9, 0.3, 1.9, 400.0, 493.0, 589.0, 120.0, -6.0, 1.3, -16.0, 0);
    car_body->get_core()->speed.set(0.0f, 0.0f, 5.0f);
    // truck: 6 wheels / 3 axes
    IVP_Polygon *truck_body = c3_box(env, &mat_dyn, 1.0, 0.35, 2.6, 700.0, 1606.0, 1810.0, 262.0, 6.0, 1.2, -18.0, 0);
    truck_body->get_core()->speed.set(0.0f, 0.0f, 4.0f);
    // trike: 3 wheels / 2 axes (front pair, single rear wheel)
    IVP_Polygon *trike_body = c3_box(env, &mat_dyn, 0.6, 0.25, 1.2, 150.0, 75.0, 90.0, 40.0, 0.0, 1.65, -15.0, 0);
    trike_body->get_core()->speed.set(0.0f, 0.0f, 3.0f);

    RaycastCarScenario *s = new RaycastCarScenario;
    s->spt = c3_steps_per_tick(env);
    s->world.n = 0;
    {
        C3RaySurface g = { ground, 40.0, 0.5, 80.0, 0.8f };
        C3RaySurface r = { ramp, 1.8, 0.5, 2.0, 0.9f };
        C3RaySurface b0 = { bump0, 0.4, 0.1, 0.3, 0.7f };
        C3RaySurface b1 = { bump1, 0.4, 0.12, 0.5, 0.7f };
        C3RaySurface b2 = { bump2, 0.3, 0.08, 0.6, 0.7f };
        s->world.s[s->world.n++] = g;
        s->world.s[s->world.n++] = r;
        s->world.s[s->world.n++] = b0;
        s->world.s[s->world.n++] = b1;
        s->world.s[s->world.n++] = b2;
    }

    {
        IVP_Template_Car_System tcs(4, 2);
        tcs.car_body = car_body;
        for (int i = 0; i < 4; ++i) {
            const double wx = (i & 1) ? 0.8 : -0.8;  // odd = right
            const double wz = (i & 2) ? -1.35 : 1.35; // 2,3 = rear
            tcs.wheel_pos_Bos[i].set((IVP_FLOAT)wx, 0.25f, (IVP_FLOAT)wz);
            tcs.trace_pos_Bos[i].set((IVP_FLOAT)wx, 0.25f, (IVP_FLOAT)wz);
            tcs.wheel_radius[i] = 0.35f;
            tcs.spring_constant[i] = 25000.0f;
            tcs.spring_dampening[i] = 2400.0f;
            tcs.spring_dampening_compression[i] = 1800.0f;
            tcs.spring_pre_tension[i] = -0.45f;
        }
        tcs.stabilizer_constant[0] = 3000.0f;
        tcs.stabilizer_constant[1] = 2000.0f;
        tcs.wheel_max_rotation_speed[0] = 50.0f;
        tcs.wheel_max_rotation_speed[1] = 50.0f;
        tcs.extra_gravity_force_value = 0.0f;
        tcs.body_down_force_vertical_offset = 0.2f;
        s->car = new RunnerRaycastCar(env, &tcs, &s->world);
    }
    {
        IVP_Template_Car_System tcs(6, 3);
        tcs.car_body = truck_body;
        for (int i = 0; i < 6; ++i) {
            const double wx = (i & 1) ? 0.9 : -0.9;
            const double wz = 1.9 - 1.9 * (double)(i >> 1); // axes at z = 1.9, 0, -1.9
            tcs.wheel_pos_Bos[i].set((IVP_FLOAT)wx, 0.3f, (IVP_FLOAT)wz);
            tcs.trace_pos_Bos[i].set((IVP_FLOAT)wx, 0.3f, (IVP_FLOAT)wz);
            tcs.wheel_radius[i] = 0.4f;
            tcs.spring_constant[i] = 30000.0f;
            tcs.spring_dampening[i] = 3000.0f;
            tcs.spring_dampening_compression[i] = 2500.0f;
            tcs.spring_pre_tension[i] = -0.4f;
        }
        tcs.stabilizer_constant[0] = 4000.0f;
        tcs.stabilizer_constant[1] = 2500.0f;
        tcs.stabilizer_constant[2] = 2500.0f;
        tcs.wheel_max_rotation_speed[0] = 40.0f;
        tcs.wheel_max_rotation_speed[1] = 40.0f;
        tcs.wheel_max_rotation_speed[2] = 40.0f;
        tcs.extra_gravity_force_value = 0.0f;
        tcs.body_down_force_vertical_offset = 0.3f;
        s->truck = new RunnerRaycastCar(env, &tcs, &s->world);
    }
    {
        IVP_Template_Car_System tcs(3, 2);
        tcs.car_body = trike_body;
        tcs.wheel_pos_Bos[0].set(-0.8f, 0.25f, 0.8f);
        tcs.wheel_pos_Bos[1].set(0.8f, 0.25f, 0.8f);
        tcs.wheel_pos_Bos[2].set(0.0f, 0.25f, -0.9f);
        for (int i = 0; i < 3; ++i) {
            tcs.trace_pos_Bos[i] = tcs.wheel_pos_Bos[i];
            tcs.wheel_radius[i] = 0.3f;
            tcs.spring_constant[i] = 12000.0f;
            tcs.spring_dampening[i] = 1100.0f;
            tcs.spring_dampening_compression[i] = 900.0f;
            tcs.spring_pre_tension[i] = -0.35f;
        }
        tcs.stabilizer_constant[0] = 1500.0f;
        tcs.stabilizer_constant[1] = 1500.0f;
        tcs.wheel_max_rotation_speed[0] = 60.0f;
        tcs.wheel_max_rotation_speed[1] = 60.0f;
        tcs.extra_gravity_force_value = 300.0f;
        tcs.body_down_force_vertical_offset = 0.1f;
        s->trike = new RunnerRaycastCar(env, &tcs, &s->world);
    }
    res->scenario_data = s;
    res->step_hook = raycast_car_step_hook;
    res->cleanup_hook = raycast_car_cleanup_hook;

    c3_add(scene, ground, "ground");
    c3_add(scene, ramp, "ramp");
    c3_add(scene, bump0, "bump");
    c3_add(scene, bump1, "bump");
    c3_add(scene, bump2, "bump");
    c3_add(scene, car_body, "car");
    c3_add(scene, truck_body, "truck");
    c3_add(scene, trike_body, "trike");
}

/* ===================================================================== */
/* check_dist_events                                                      */
/* ===================================================================== */

// Three IVP_Actuator_Check_Dist setups whose events change the simulation:
//   yo-yo     ball on a spring under a hook: outside -> an upward
//             IVP_Actuator_Force on the ball is switched on, inside -> off
//             (bang-bang oscillation around the range)
//   wiper     hinged blade driven by a rot motor; its tip against two markers:
//             inside -> the motor's max_rotation_speed is reversed, outside ->
//             the motor power alternates
//   ping-pong frictionless ball sliding between two posts: inside -> a fixed
//             impulse (async_push_object_ws) towards the other post
// The listener receives the reference argument as fired (IVP_TRUE = outside).

struct CheckDistScenario;

class RunnerCheckDistListener : public IVP_Listener_Check_Dist_Event {
public:
    CheckDistScenario *s;
    int which;
    RunnerCheckDistListener() : s(0), which(0) {}
    void check_dist_event(IVP_Actuator_Check_Dist *, IVP_BOOL is_outside);
    void check_dist_is_going_to_be_deleted_event(IVP_Actuator_Check_Dist *cd) {
        cd->remove_listener_check_dist_event(this);
    }
};

struct CheckDistScenario {
    IVP_Actuator_Force *yoyo_force;
    IVP_Actuator_Rot_Mot *wiper_motor;
    IVP_Real_Object *pong;
    IVP_Actuator_Check_Dist *cd[5];
    RunnerCheckDistListener listener[5];
    float wiper_speed;
    int wiper_power_toggle;
    int events;
};

void RunnerCheckDistListener::check_dist_event(IVP_Actuator_Check_Dist *, IVP_BOOL is_outside) {
    s->events++;
    switch (which) {
    case 0:
        s->yoyo_force->set_force(is_outside ? 14.0 : 0.0);
        break;
    case 1:
    case 2:
        if (!is_outside) {
            s->wiper_speed = -s->wiper_speed;
            s->wiper_motor->set_max_rotation_speed(s->wiper_speed);
        } else {
            s->wiper_power_toggle ^= 1;
            s->wiper_motor->set_power(s->wiper_power_toggle ? 4.5 : 6.0);
        }
        break;
    case 3:
    case 4:
        if (!is_outside) {
            IVP_U_Point pos;
            pos.set(s->pong->get_core()->get_position_PSI());
            IVP_U_Float_Point impulse(which == 3 ? 2.0f : -2.0f, 0.0f, 0.0f);
            s->pong->async_push_object_ws(&pos, &impulse);
        }
        break;
    default:
        break;
    }
}

IVP_Actuator_Check_Dist *c3_check_dist(IVP_Environment *env, IVP_Real_Object *o0, double x0, double y0, double z0,
                                       IVP_Real_Object *o1, double x1, double y1, double z1, double range,
                                       RunnerCheckDistListener *listener) {
    IVP_Template_Check_Dist t;
    t.objects[0] = o0;
    t.objects[1] = o1;
    t.position_world_space[0].set(x0, y0, z0);
    t.position_world_space[1].set(x1, y1, z1);
    t.range = (IVP_FLOAT)range;
    IVP_Actuator_Check_Dist *cd = IVP_Controller_Factory::create_check_dist(env, &t);
    cd->add_listener_check_dist_event(listener);
    return cd;
}

void check_dist_cleanup_hook(ScenarioResources *res) {
    CheckDistScenario *s = (CheckDistScenario *)res->scenario_data;
    if (!s) return;
    for (int i = 0; i < 5; ++i) delete s->cd[i]; // listeners remove themselves
    delete s;
    res->scenario_data = 0;
}

void setup_check_dist_events(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    static IVP_Material_Simple mat_slick(0.0, 0.0);
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point rd; rd.set(0.0, 0.0, 0.0);

    IVP_Polygon *ground = c3_static(env, &mat_ground, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, 0);
    IVP_Polygon *hook = c3_static(env, &mat_ground, 0.2, 0.2, 0.2, -6.0, -3.0, 0.0, 0);
    IVP_U_Point p_yoyo; p_yoyo.set(-6.0, -1.0, 0.0);
    IVP_Ball *yoyo = create_dynamic_ball(env, &mat_dyn, 0.4, 1.0, 0.064, 0.064, 0.064, 0.0, &rd, &qi, &p_yoyo);
    IVP_Polygon *pedestal = c3_static(env, &mat_ground, 0.15, 0.6, 0.15, 4.0, -4.0, 0.0, 0);
    IVP_Polygon *blade = c3_box(env, &mat_dyn, 2.0, 0.1, 0.3, 1.0, 0.04, 1.36, 1.34, 4.0, -5.2, 0.0, 0);
    IVP_Polygon *post_l = c3_static(env, &mat_ground, 0.2, 0.5, 0.2, -2.5, 2.0, 6.0, 0);
    IVP_Polygon *post_r = c3_static(env, &mat_ground, 0.2, 0.5, 0.2, 2.5, 2.0, 6.0, 0);
    IVP_U_Point p_pong; p_pong.set(0.0, 2.18, 6.0);
    IVP_Ball *pong = create_dynamic_ball(env, &mat_slick, 0.3, 0.5, 0.018, 0.018, 0.018, 0.0, &rd, &qi, &p_pong);
    pong->get_core()->speed.set(2.0f, 0.0f, 0.0f);

    CheckDistScenario *s = new CheckDistScenario;
    s->wiper_speed = 3.0f;
    s->wiper_power_toggle = 0;
    s->events = 0;
    s->pong = pong;

    add_spring(env, hook, 0.0, 0.2, 0.0, yoyo, 0.0, 0.0, 0.0, 40.0, 0.5, 1.8);
    {
        IVP_Template_Anchor a0, a1;
        IVP_Template_Force tf;
        tf.anchors[0] = &a0;
        tf.anchors[1] = &a1;
        a0.set_anchor_position_os(yoyo, 0.0, 0.0, 0.0);
        a1.set_anchor_position_os(yoyo, 0.0, 1.0, 0.0);
        tf.force = 0.0f;
        tf.push_first_object = IVP_TRUE;
        tf.push_second_object = IVP_FALSE;
        s->yoyo_force = IVP_Controller_Factory::create_force(env, &tf);
    }
    {
        IVP_U_Point anchor_ws; anchor_ws.set(4.0, -5.2, 0.0);
        IVP_U_Point axis_ws; axis_ws.set(0.0, 1.0, 0.0);
        IVP_Template_Constraint tc;
        tc.set_hinge_ws(pedestal, &anchor_ws, &axis_ws, blade);
        IVP_Controller_Factory::create_constraint(env, &tc);
    }
    {
        IVP_Template_Anchor a0, a1;
        IVP_Template_Rot_Mot tm;
        tm.anchors[0] = &a0;
        tm.anchors[1] = &a1;
        a0.set_anchor_position_os(blade, 0.0, 0.0, 0.0);
        a1.set_anchor_position_os(blade, 0.0, 1.0, 0.0);
        tm.power = 6.0f;
        tm.max_torque = 8.0f;
        tm.max_rotation_speed = 3.0f;
        s->wiper_motor = IVP_Controller_Factory::create_rotmot(env, &tm);
    }
    for (int i = 0; i < 5; ++i) {
        s->listener[i].s = s;
        s->listener[i].which = i;
    }
    s->cd[0] = c3_check_dist(env, hook, -6.0, -3.0, 0.0, yoyo, -6.0, -1.0, 0.0, 2.15, &s->listener[0]);
    s->cd[1] = c3_check_dist(env, blade, 6.0, -5.2, 0.0, pedestal, 4.0, -5.2, 2.0, 0.6, &s->listener[1]);
    s->cd[2] = c3_check_dist(env, blade, 6.0, -5.2, 0.0, pedestal, 4.0, -5.2, -2.0, 0.6, &s->listener[2]);
    s->cd[3] = c3_check_dist(env, pong, 0.0, 2.2, 6.0, post_l, -2.5, 2.2, 6.0, 1.0, &s->listener[3]);
    s->cd[4] = c3_check_dist(env, pong, 0.0, 2.2, 6.0, post_r, 2.5, 2.2, 6.0, 1.0, &s->listener[4]);

    res->scenario_data = s;
    res->cleanup_hook = check_dist_cleanup_hook;

    c3_add(scene, ground, "ground");
    c3_add(scene, hook, "hook");
    c3_add(scene, yoyo, "yoyo");
    c3_add(scene, pedestal, "pedestal");
    c3_add(scene, blade, "wiper");
    c3_add(scene, post_l, "post");
    c3_add(scene, post_r, "post");
    c3_add(scene, pong, "pong");
}

/* ===================================================================== */
/* golem_beam                                                             */
/* ===================================================================== */

// IVP_Controller_Golem whose resolve_for_problem beams the object to the
// target (beam_object_to_target_position): FAR_DISTANCE (lagging target,
// prime position jumps) and BIG_ANGLE (prime orientation jumps), a golem on
// an object with a shifted core (shift_core_f_object != 0: delta position
// and beam_object_to_new_position shift paths), a negated target quaternion
// (acos_quat < 0), interpolated orientations and an environment reset_time
// (IVP_Controller_Golem::reset_time).

IVP_Compact_Surface *c3_offset_box_surface(double hx, double hy, double hz, double ox, double oy, double oz) {
    IVP_U_Vector<IVP_U_Point> points(8);
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                IVP_U_Point *p = new IVP_U_Point();
                p->set((double)sx * hx + ox, (double)sy * hy + oy, (double)sz * hz + oz);
                points.add(p);
            }
        }
    }
    IVP_Compact_Surface *cs = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface(&points);
    for (int i = points.len() - 1; i >= 0; --i) delete points.element_at(i);
    return cs;
}

class RunnerBeamGolem : public IVP_Controller_Golem {
public:
    int far_count, angle_count;
    RunnerBeamGolem(IVP_Real_Object *o, const IVP_Template_Controller_Golem *t)
        : IVP_Controller_Golem(o, t), far_count(0), angle_count(0) {}
    IVP_RETURN_TYPE resolve_for_problem(IVP_Event_Sim *es, IVP_GOLEM_PROBLEM problem) {
        if (problem == IVP_GP_FAR_DISTANCE) far_count++;
        else angle_count++;
        beam_object_to_target_position(es);
        return IVP_OK;
    }
};

struct GolemBeamScenario {
    RunnerBeamGolem *a, *b, *c;
    int spt;
};

void golem_beam_step_hook(int step, IVP_Environment *env, ScenarioResources *res) {
    GolemBeamScenario *s = (GolemBeamScenario *)res->scenario_data;
    if ((step - 1) % s->spt) return;
    const int tick = (step - 1) / s->spt;
    IVP_Time now = env->get_current_time();
    switch (tick) {
    case 30: {
        IVP_U_Point p; p.set(1.0, 0.5, 1.0);
        IVP_U_Float_Point v(0.0, 0.0, 0.0);
        s->b->set_prime_position(&p, &v, now);
        break;
    }
    case 60: {
        IVP_U_Quat q; c3_quat(&q, 1.0, 0.0, 0.0, 1.2);
        s->b->set_prime_orientation(&q, now);
        break;
    }
    case 90: {
        IVP_U_Point p; p.set(4.0, -1.0, 8.0);
        IVP_U_Float_Point v(0.0, 0.0, -3.0);
        s->c->set_prime_position(&p, &v, now);
        break;
    }
    case 120: {
        IVP_U_Quat q0; c3_quat(&q0, 0.0, 0.0, 1.0, 0.3);
        IVP_U_Quat q1; c3_quat(&q1, 0.0, 0.0, 1.0, 1.0);
        s->b->set_prime_orientation(&q0, now, &q1, 1.5f);
        break;
    }
    case 130: {
        IVP_U_Quat q; c3_quat(&q, 0.0, 1.0, 0.0, -1.5);
        s->c->set_prime_orientation(&q, now);
        break;
    }
    case 150: {
        // the same rotation as +0.5 rad about y, negated (w < 0)
        IVP_U_Quat q; c3_quat(&q, 0.0, 1.0, 0.0, 0.5);
        q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
        s->a->set_prime_orientation(&q, now);
        break;
    }
    case 180: {
        IVP_U_Point p; p.set(1.0, 0.5, 6.0);
        IVP_U_Float_Point v(0.0, 0.0, -1.0);
        s->b->set_prime_position(&p, &v, now);
        break;
    }
    case 200:
        env->reset_time();
        break;
    case 210: {
        // 0.3 rad from the ball's orientation, negated: acos_quat < 0 without a problem
        IVP_U_Quat q; c3_quat(&q, 0.0, 1.0, 0.0, -1.2);
        q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
        s->c->set_prime_orientation(&q, now);
        break;
    }
    case 260: {
        IVP_U_Point p; p.set(-2.0, 0.0, -2.0);
        IVP_U_Float_Point v(1.5, 0.0, 0.5);
        s->a->set_prime_position(&p, &v, now);
        break;
    }
    case 330: {
        IVP_U_Quat q; c3_quat(&q, 1.0, 0.0, 0.0, 0.5);
        s->c->set_prime_orientation(&q, now);
        break;
    }
    default:
        break;
    }
}

void golem_beam_cleanup_hook(ScenarioResources *res) {
    delete (GolemBeamScenario *)res->scenario_data; // controllers die with their cores
    res->scenario_data = 0;
}

void setup_golem_beam(IVP_Environment *env, SceneObjects *scene, ScenarioResources *res) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);
    IVP_U_Quat qi; qi.x = qi.y = qi.z = 0.0; qi.w = 1.0;
    IVP_U_Point rd; rd.set(0.0, 0.0, 0.0);

    IVP_Polygon *ground = c3_static(env, &mat_ground, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, 0);
    IVP_Polygon *a = c3_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.5, 0.16, 0.16, 0.16, -4.0, 1.0, -3.0, 0);
    IVP_Polygon *b;
    {
        IVP_Compact_Surface *cs = c3_offset_box_surface(0.3, 0.25, 0.5, 0.0, 0.0, 0.6);
        IVP_SurfaceManager_Polygon *sm = new IVP_SurfaceManager_Polygon(cs);
        IVP_Template_Real_Object t;
        configure_dynamic_template(&t, &mat_dyn, 2.0);
        configure_explicit_inertia(&t, 0.21, 0.23, 0.1);
        IVP_U_Point p; p.set(0.0, 1.0, 0.0);
        b = env->create_polygon(sm, &t, &qi, &p);
        wake_and_enable(b);
    }
    IVP_U_Point pc; pc.set(4.0, 1.0, 2.0);
    IVP_Ball *c = create_dynamic_ball(env, &mat_dyn, 0.5, 1.0, 0.1, 0.1, 0.1, 0.0, &rd, &qi, &pc);

    GolemBeamScenario *s = new GolemBeamScenario;
    s->spt = c3_steps_per_tick(env);
    IVP_Time now = env->get_current_time();
    {
        IVP_Template_Controller_Golem t;
        t.max_translation_force.set(30.0f, 30.0f, 30.0f);
        t.max_torque = 10.0f;
        t.max_delta_position = 1.5f;
        t.max_delta_orientation = 1.0f;
        s->a = new RunnerBeamGolem(a, &t);
        IVP_U_Point p; p.set(-4.0, 1.0, -3.0);
        IVP_U_Float_Point v(2.5, 0.0, 0.0);
        s->a->set_prime_position(&p, &v, now);
        IVP_U_Quat q1; c3_quat(&q1, 0.0, 1.0, 0.0, 1.5);
        s->a->set_prime_orientation(&qi, now, &q1, 2.0f);
    }
    {
        IVP_Template_Controller_Golem t;
        t.max_translation_force.set(60.0f, 60.0f, 60.0f);
        t.max_torque = 20.0f;
        t.force_factor = 0.7f;
        t.damp_factor = 0.9f;
        t.torque_factor = 0.6f;
        t.max_delta_position = 3.0f;
        t.max_delta_orientation = 0.8f;
        s->b = new RunnerBeamGolem(b, &t);
    }
    {
        IVP_Template_Controller_Golem t;
        t.max_translation_force.set(40.0f, 40.0f, 40.0f);
        t.max_torque = 30.0f;
        t.max_delta_position = 2.5f;
        s->c = new RunnerBeamGolem(c, &t);
    }
    res->scenario_data = s;
    res->step_hook = golem_beam_step_hook;
    res->cleanup_hook = golem_beam_cleanup_hook;

    c3_add(scene, ground, "ground");
    c3_add(scene, a, "golem");
    c3_add(scene, b, "golem_shifted");
    c3_add(scene, c, "golem");
}

} // namespace

bool setup_controllers3_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    switch (scenario) {
        case SCENARIO_RAYCAST_CAR_DRIVE: setup_raycast_car_drive(env, scene, resources); return true;
        case SCENARIO_CHECK_DIST_EVENTS: setup_check_dist_events(env, scene, resources); return true;
        case SCENARIO_GOLEM_BEAM: setup_golem_beam(env, scene, resources); return true;
        default: return false;
    }
}

} // namespace ref_runner
