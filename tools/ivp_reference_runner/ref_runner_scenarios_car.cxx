#include "ref_runner.hxx"

#include <ivp_car_system.hxx>

#include <cstdio>

namespace ref_runner {

/* "car_real_wheels": IVP_Car_System_Real_Wheels (constraint car solver +
 * suspensions + wheel torques + extra gravity / down force on the static
 * object) driving on a static ground box.  Mirrored by run_car_real_wheels in
 * ivp-c17/tests/test_scenarios.c; keep both in sync. */

namespace {

struct CarScenario {
    IVP_Car_System_Real_Wheels *car;
};

const double CAR_BODY_HX = 0.9, CAR_BODY_HY = 0.3, CAR_BODY_HZ = 2.0;
const double CAR_WHEEL_X = 0.85, CAR_WHEEL_Y = 0.4, CAR_WHEEL_Z = 1.4;
const double CAR_WHEEL_RADIUS = 0.35;
const double CAR_BODY_Y = 1.7; /* wheels start 5 cm above the ground top (y = 2.5) */

void car_step_hook(int step, IVP_Environment * /*env*/, ScenarioResources *resources) {
    CarScenario *cs = (CarScenario *)resources->scenario_data;
    if (!cs || !cs->car) return;
    IVP_Car_System_Real_Wheels *car = cs->car;

    if (step == 181) { /* t = 1.0 s: rear wheel drive, steer left, down force */
        car->change_wheel_torque(IVP_REAR_LEFT, 700.0f);
        car->change_wheel_torque(IVP_REAR_RIGHT, 700.0f);
        car->update_body_countertorque();
        car->do_steering(0.25f);
        car->change_body_downforce(1500.0f);
    }
    if (step == 361) { /* t = 2.0 s: steer the other way, more throttle */
        car->change_wheel_torque(IVP_REAR_LEFT, 900.0f);
        car->change_wheel_torque(IVP_REAR_RIGHT, 900.0f);
        car->update_body_countertorque();
        car->do_steering(-0.15f);
    }
    if (step == 541) { /* t = 3.0 s: release throttle, handbrake on the rear wheels */
        car->change_wheel_torque(IVP_REAR_LEFT, 0.0f);
        car->change_wheel_torque(IVP_REAR_RIGHT, 0.0f);
        car->update_body_countertorque();
        car->do_steering(0.0f);
        car->fix_wheel(IVP_REAR_LEFT, IVP_TRUE);
        car->fix_wheel(IVP_REAR_RIGHT, IVP_TRUE);
    }
}

void car_cleanup_hook(ScenarioResources *resources) {
    CarScenario *cs = (CarScenario *)resources->scenario_data;
    if (!cs) return;
    delete cs->car;
    cs->car = 0;
    delete cs;
    resources->scenario_data = 0;
}

void setup_car_real_wheels(IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_ground(0.9, 0.0);
    static IVP_Material_Simple mat_body(0.6, 0.1);
    static IVP_Material_Simple mat_wheel(0.9, 0.1);
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 3.0, 0.0);
    IVP_Polygon *ground = create_static_box(env, &mat_ground, 30.0, 0.5, 30.0, &q_ident, &pos_ground);
    scene->objects[idx] = ground; scene->types[idx++] = "ground";

    /* body and wheels share a collision group (the wheels overlap the body box) */
    IVP_U_Point pos_body; pos_body.set(0.0, CAR_BODY_Y, 0.0);
    IVP_Polygon *body;
    {
        IVP_Compact_Surface *compact = build_box_compact_surface(CAR_BODY_HX, CAR_BODY_HY, CAR_BODY_HZ);
        IVP_SurfaceManager_Polygon *surman = new IVP_SurfaceManager_Polygon(compact);
        IVP_Template_Real_Object templ_obj;
        configure_dynamic_template(&templ_obj, &mat_body, 800.0);
        configure_explicit_inertia(&templ_obj, 1100.0, 1300.0, 250.0);
        templ_obj.speed_damp_factor = 0.0;
        templ_obj.rot_speed_damp_factor = rot_damp_zero;
        templ_obj.set_nocoll_group_ident("car");
        body = env->create_polygon(surman, &templ_obj, &q_ident, &pos_body);
        wake_and_enable(body);
    }

    IVP_Template_Car_System tcs(4, 2);
    tcs.car_body = body;
    for (int i = 0; i < 4; ++i) {
        const double wx = (i & 1) ? CAR_WHEEL_X : -CAR_WHEEL_X; /* odd = right */
        const double wz = (i & 2) ? -CAR_WHEEL_Z : CAR_WHEEL_Z; /* 2,3 = rear  */
        IVP_U_Point p; p.set(wx, CAR_BODY_Y + CAR_WHEEL_Y, wz);
        IVP_Ball *wheel;
        {
            IVP_Template_Real_Object templ_obj;
            configure_dynamic_template(&templ_obj, &mat_wheel, 25.0);
            configure_explicit_inertia(&templ_obj, 1.225, 1.225, 1.225);
            templ_obj.speed_damp_factor = 0.0;
            templ_obj.rot_speed_damp_factor = rot_damp_zero;
            templ_obj.set_nocoll_group_ident("car");
            IVP_Template_Ball templ_ball;
            templ_ball.radius = (IVP_FLOAT)CAR_WHEEL_RADIUS;
            wheel = env->create_ball(&templ_ball, &templ_obj, &q_ident, &p);
            wake_and_enable(wheel);
        }
        scene->objects[idx] = wheel; scene->types[idx++] = "wheel";

        tcs.car_wheel[i] = wheel;
        tcs.wheel_radius[i] = (IVP_FLOAT)CAR_WHEEL_RADIUS;
        tcs.wheel_reversed_sign[i] = 1.0f;
        tcs.friction_of_wheel[i] = 0.9f;
        tcs.wheel_pos_Bos[i].set((IVP_FLOAT)wx, (IVP_FLOAT)CAR_WHEEL_Y, (IVP_FLOAT)wz);
        tcs.trace_pos_Bos[i].set((IVP_FLOAT)wx, (IVP_FLOAT)CAR_WHEEL_Y, (IVP_FLOAT)wz);
        tcs.spring_constant[i] = 40000.0f;
        tcs.spring_dampening[i] = 3000.0f;
        tcs.spring_dampening_compression[i] = 3000.0f;
        tcs.max_body_force[i] = 60000.0f;
        tcs.spring_pre_tension[i] = 0.05f;
    }
    tcs.stabilizer_constant[0] = 1500.0f;
    tcs.stabilizer_constant[1] = 1500.0f;
    tcs.wheel_max_rotation_speed[0] = 60.0f;
    tcs.wheel_max_rotation_speed[1] = 60.0f;
    tcs.body_counter_torque_factor = 0.2f;
    tcs.extra_gravity_force_value = 2000.0f;
    tcs.extra_gravity_height_offset = 0.0f;
    tcs.body_down_force_vertical_offset = 0.2f;
    tcs.fast_turn_factor = 1.0f;

    CarScenario *cs = new CarScenario;
    cs->car = new IVP_Car_System_Real_Wheels(env, &tcs);
    resources->scenario_data = cs;
    resources->step_hook = car_step_hook;
    resources->cleanup_hook = car_cleanup_hook;

    scene->objects[idx] = body; scene->types[idx++] = "car_body";
    scene->count = idx;
}

} // namespace

bool setup_car_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    if (scenario == SCENARIO_CAR_REAL_WHEELS) {
        setup_car_real_wheels(env, scene, resources);
        return true;
    }
    return false;
}

} // namespace ref_runner
