#include "ref_runner.hxx"

namespace ref_runner {

static void setup_freefall(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_dynamic(0.8, 0.0);

    IVP_Template_Real_Object templ_obj;
    configure_dynamic_template(&templ_obj, &mat_dynamic, 1.0);
    IVP_Template_Ball templ_ball;
    templ_ball.radius = 0.5f;

    IVP_U_Quat q;
    q.x = q.y = q.z = 0.0;
    q.w = 1.0;

    IVP_U_Point pos;
    pos.set(0.0, -5.0, 0.0);

    IVP_Ball *ball = env->create_ball(&templ_ball, &templ_obj, &q, &pos);
    ball->get_core()->speed.set(0.0f, 0.0f, 0.0f);
    ball->get_core()->rot_speed.set(0.0f, 0.0f, 0.0f);
    wake_and_enable(ball);

    scene->objects[0] = ball;
    scene->types[0] = "ball";
    scene->count = 1;
}

static void setup_two_balls(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_dynamic(0.8, 0.0);

    IVP_U_Quat q;
    q.x = q.y = q.z = 0.0;
    q.w = 1.0;
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    IVP_U_Point posA; posA.set(-2.0, -2.0, 0.0);
    IVP_U_Point posB; posB.set(2.0, -2.0, 0.0);

    IVP_Ball *a = create_dynamic_ball(env, &mat_dynamic, 0.5, 1.0, 0.1, 0.1, 0.1,
                                      0.0, &rot_damp_zero, &q, &posA);
    IVP_Ball *b = create_dynamic_ball(env, &mat_dynamic, 0.5, 1.0, 0.1, 0.1, 0.1,
                                      0.0, &rot_damp_zero, &q, &posB);

    a->get_core()->speed.set(2.0f, 0.0f, 0.0f);
    b->get_core()->speed.set(-2.0f, 0.0f, 0.0f);
    a->get_core()->rot_speed.set(0.0f, 0.0f, 0.0f);
    b->get_core()->rot_speed.set(0.0f, 0.0f, 0.0f);

    scene->objects[0] = a; scene->types[0] = "ball";
    scene->objects[1] = b; scene->types[1] = "ball";
    scene->count = 2;
}

static void setup_slope_friction(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_static(0.8, 0.0);

    IVP_Compact_Surface *compact = build_box_compact_surface(5.0, 0.25, 2.5);
    IVP_SurfaceManager_Polygon *surman = new IVP_SurfaceManager_Polygon(compact);

    IVP_Template_Real_Object templ_ramp;
    configure_static_template(&templ_ramp, &mat_static);

    IVP_U_Quat q_ramp;
    set_quat_axis_angle(&q_ramp, 0.0, 0.0, 1.0, 30.0 * (3.14159265358979323846 / 180.0));

    IVP_U_Point pos_ramp;
    pos_ramp.set(0.0, 0.0, 0.0);

    IVP_Polygon *ramp = env->create_polygon(surman, &templ_ramp, &q_ramp, &pos_ramp);
    ramp->get_core()->speed.set(0.0f, 0.0f, 0.0f);
    ramp->get_core()->rot_speed.set(0.0f, 0.0f, 0.0f);
    ramp->enable_collision_detection(IVP_TRUE);

    static IVP_Material_Simple mat_ball(0.4, 0.0);
    IVP_U_Quat q_ball;
    q_ball.x = q_ball.y = q_ball.z = 0.0;
    q_ball.w = 1.0;
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    IVP_U_Point pos_ball;
    pos_ball.set(-4.0, -4.0, 0.0);

    IVP_Ball *ball = create_dynamic_ball(env, &mat_ball, 0.5, 1.0, 0.1, 0.1, 0.1,
                                         0.0, &rot_damp_zero, &q_ball, &pos_ball);

    ball->get_core()->speed.set(0.0f, 0.0f, 0.0f);
    ball->get_core()->rot_speed.set(0.0f, 0.0f, 0.0f);

    scene->objects[0] = ramp; scene->types[0] = "poly";
    scene->objects[1] = ball; scene->types[1] = "ball";
    scene->count = 2;
}

static void setup_cubes(IVP_Environment *env, SceneObjects *scene) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_cube(0.6, 0.05);
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_ground, 20.0, 0.5, 20.0, &q_ident, &pos_ground); scene->types[idx++] = "ground";

    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            IVP_U_Point pos; pos.set(-2.25 + x * 1.5, -2.0 - y * 1.1, 0.0);
            scene->objects[idx] = create_dynamic_box(env, &mat_cube, 0.5, 0.5, 0.5,
                                                     0.6, 0.1, 0.1, 0.1,
                                                     0.02, &rot_damp_zero,
                                                     &q_ident, &pos);
            scene->types[idx++] = "cube";
        }
    }
    scene->count = idx;
}

static void setup_collision_filter(IVP_Environment *env, SceneObjects *scene) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.6, 0.1);

    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_static, 20.0, 0.5, 20.0, &q_ident, &pos_ground); scene->types[idx++] = "ground";

    IVP_Template_Real_Object t;
    configure_dynamic_template(&t, &mat_dyn, 1.0);
    t.set_nocoll_group_ident("grpA");
    IVP_Template_Ball tb;
    tb.radius = 0.5f;
    IVP_U_Point p0; p0.set(-3.0, -3.0, 0.0);
    IVP_U_Point p1; p1.set(3.0, -3.0, 0.0);
    IVP_Ball *a = env->create_ball(&tb, &t, &q_ident, &p0);
    IVP_Ball *b = env->create_ball(&tb, &t, &q_ident, &p1);
    wake_and_enable(a);
    wake_and_enable(b);
    a->get_core()->speed.set(3.0f, 0.0f, 0.0f);
    b->get_core()->speed.set(-3.0f, 0.0f, 0.0f);

    scene->objects[idx] = a; scene->types[idx++] = "filter_ball";
    scene->objects[idx] = b; scene->types[idx++] = "filter_ball";
    scene->count = idx;
}

static void setup_vehicle_proxy(IVP_Environment *env, SceneObjects *scene) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_static(0.9, 0.0);
    static IVP_Material_Simple mat_dyn(0.6, 0.05);
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 3.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_static, 30.0, 0.5, 30.0, &q_ident, &pos_ground); scene->types[idx++] = "ground";

    IVP_U_Point pos_body; pos_body.set(0.0, -4.0, 0.0);
    IVP_Polygon *body = create_dynamic_box(env, &mat_dyn, 1.6, 0.35, 0.8,
                                           2.2, 0.9, 0.9, 0.9,
                                           0.0, &rot_damp_zero,
                                           &q_ident, &pos_body);
    body->get_core()->speed.set(2.0f, 0.0f, 0.0f);

    const double wheel_y = -3.2;
    const double wheel_z = 0.95;
    for (int i = 0; i < 4; ++i) {
        const double wx = (i < 2) ? -1.1 : 1.1;
        const double wz = (i % 2 == 0) ? -wheel_z : wheel_z;
        IVP_U_Point p; p.set(wx, wheel_y, wz);
        scene->objects[idx] = create_dynamic_ball(env, &mat_dyn, 0.35, 0.3, 0.02, 0.02, 0.02,
                                                  0.0, &rot_damp_zero, &q_ident, &p);
        scene->types[idx++] = "wheel";
    }

    scene->objects[idx] = body; scene->types[idx++] = "vehicle_body";
    scene->count = idx;
}

bool setup_basic_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources * /*resources*/) {
    if (scenario == SCENARIO_FREEFALL) {
        setup_freefall(env, scene);
        return true;
    }
    if (scenario == SCENARIO_TWO_BALLS) {
        setup_two_balls(env, scene);
        return true;
    }
    if (scenario == SCENARIO_SLOPE_FRICTION) {
        setup_slope_friction(env, scene);
        return true;
    }
    if (scenario == SCENARIO_CUBES) {
        setup_cubes(env, scene);
        return true;
    }
    if (scenario == SCENARIO_COLLISION_FILTER) {
        setup_collision_filter(env, scene);
        return true;
    }
    if (scenario == SCENARIO_VEHICLE) {
        setup_vehicle_proxy(env, scene);
        return true;
    }
    return false;
}

} // namespace ref_runner
