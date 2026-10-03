#include "ref_runner.hxx"

namespace ref_runner {

static void setup_buoyancy(IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;

    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.2);

    IVP_U_Point pos_ground; pos_ground.set(0.0, 5.0, 0.0);
    IVP_Polygon *ground = create_static_box(env, &mat_static, 15.0, 0.5, 15.0, &q_ident, &pos_ground);
    IVP_U_Point pos_wall_l; pos_wall_l.set(-12.0, 0.0, 0.0);
    IVP_Polygon *wall_l = create_static_box(env, &mat_static, 0.3, 6.0, 15.0, &q_ident, &pos_wall_l);
    IVP_U_Point pos_wall_r; pos_wall_r.set(12.0, 0.0, 0.0);
    IVP_Polygon *wall_r = create_static_box(env, &mat_static, 0.3, 6.0, 15.0, &q_ident, &pos_wall_r);

    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    IVP_U_Point pos_cork; pos_cork.set(-6.0, -6.0, 0.0);
    IVP_Polygon *cork = create_dynamic_box(env, &mat_dyn, 0.5, 0.5, 0.5,
                                           0.3, 0.072, 0.072, 0.072,
                                           0.0, &rot_damp_zero,
                                           &q_ident, &pos_cork);

    IVP_U_Point pos_wood; pos_wood.set(-3.0, -5.0, 0.0);
    IVP_Polygon *wood = create_dynamic_box(env, &mat_dyn, 0.6, 0.4, 0.6,
                                           1.0, 0.24, 0.24, 0.24,
                                           0.0, &rot_damp_zero,
                                           &q_ident, &pos_wood);

    IVP_U_Point pos_ball; pos_ball.set(0.0, -7.0, 0.0);
    IVP_Ball *ball = create_dynamic_ball(env, &mat_dyn, 0.55, 1.5, 0.36, 0.36, 0.36,
                                         0.0, &rot_damp_zero, &q_ident, &pos_ball);

    IVP_U_Point pos_steel; pos_steel.set(3.0, -4.0, 0.0);
    IVP_Polygon *steel = create_dynamic_box(env, &mat_dyn, 0.5, 0.5, 0.5,
                                            8.0, 1.92, 1.92, 1.92,
                                            0.0, &rot_damp_zero,
                                            &q_ident, &pos_steel);

    IVP_U_Point pos_brick; pos_brick.set(6.0, -6.0, 0.0);
    IVP_Polygon *brick = create_dynamic_box(env, &mat_dyn, 0.7, 0.35, 0.5,
                                            3.0, 0.72, 0.72, 0.72,
                                            0.0, &rot_damp_zero,
                                            &q_ident, &pos_brick);

    IVP_U_Point pos_foam; pos_foam.set(-4.5, -9.0, 3.0);
    IVP_Polygon *foam = create_dynamic_box(env, &mat_dyn, 0.8, 0.8, 0.8,
                                           0.2, 0.048, 0.048, 0.048,
                                           0.0, &rot_damp_zero,
                                           &q_ident, &pos_foam);

    IVP_U_Point pos_lead; pos_lead.set(4.5, -8.0, -3.0);
    IVP_Polygon *lead = create_dynamic_box(env, &mat_dyn, 0.4, 0.4, 0.4,
                                           12.0, 2.88, 2.88, 2.88,
                                           0.0, &rot_damp_zero,
                                           &q_ident, &pos_lead);

    IVP_Template_Buoyancy templ;
    // medium_density 1.0 (was 1000): the object masses (0.2..12 kg) and sizes
    // give densities of 0.05..23 kg/m^3, so 1000 launched every object out of
    // the liquid and the scene diverged.  With 1.0 cork/wood/foam float and
    // ball/steel/brick/lead sink, as intended.  Kept identical in libivp's
    // tests/test_scenarios.c (run_buoyancy).
    templ.medium_density = 1.0f;
    templ.pressure_damp_factor = 3.0f;
    templ.friction_damp_factor = 0.0f;
    templ.ball_rot_dampening_factor = 2.0f;
    templ.viscosity_factor = 0.0f;
    templ.use_interpolation = IVP_FALSE;

    IVP_U_Float_Hesse surface(0.0, 1.0, 0.0, 1.0);
    IVP_U_Float_Point flow(1.5, 0.0, 0.0);
    resources->liquid_desc = new IVP_Liquid_Surface_Descriptor_Simple(&surface, &flow);
    resources->buoyancy_cores = new IVP_U_Set_Active<IVP_Core>(16);
    resources->buoyancy_cores->install_element(cork->get_core());
    resources->buoyancy_cores->install_element(wood->get_core());
    resources->buoyancy_cores->install_element(ball->get_core());
    resources->buoyancy_cores->install_element(steel->get_core());
    resources->buoyancy_cores->install_element(brick->get_core());
    resources->buoyancy_cores->install_element(foam->get_core());
    resources->buoyancy_cores->install_element(lead->get_core());
    resources->buoyancy_attacher = new IVP_Attacher_To_Cores_Buoyancy(templ, resources->buoyancy_cores, resources->liquid_desc);

    int idx = 0;
    scene->objects[idx] = ground; scene->types[idx++] = "ground";
    scene->objects[idx] = wall_l; scene->types[idx++] = "wall";
    scene->objects[idx] = wall_r; scene->types[idx++] = "wall";
    scene->objects[idx] = cork;   scene->types[idx++] = "cork";
    scene->objects[idx] = wood;   scene->types[idx++] = "wood";
    scene->objects[idx] = ball;   scene->types[idx++] = "ball";
    scene->objects[idx] = steel;  scene->types[idx++] = "steel";
    scene->objects[idx] = brick;  scene->types[idx++] = "brick";
    scene->objects[idx] = foam;   scene->types[idx++] = "foam";
    scene->objects[idx] = lead;   scene->types[idx++] = "lead";
    scene->count = idx;
}

static void setup_force_actuator_like(IVP_Environment *env, SceneObjects *scene, IVP_BOOL stronger) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_box(0.5, 0.05);
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_ground, 20.0, 0.5, 20.0, &q_ident, &pos_ground); scene->types[idx++] = "ground";

    IVP_U_Point pos_box; pos_box.set(0.0, -2.0, 0.0);
    IVP_Polygon *box = create_dynamic_box(env, &mat_box, 0.6, 0.6, 0.6,
                                          1.0, 0.24, 0.24, 0.24,
                                          0.12, &rot_damp_zero,
                                          &q_ident, &pos_box);
    add_force_actuator(env, box, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, stronger ? 2.5 : 1.2);

    scene->objects[idx] = box; scene->types[idx++] = stronger ? "forcefield_box" : "force_box";
    scene->count = idx;
}

static void setup_motion_controller(IVP_Environment *env, SceneObjects *scene) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.05);
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 3.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_static, 20.0, 0.5, 20.0, &q_ident, &pos_ground); scene->types[idx++] = "ground";

    IVP_U_Point pos_box; pos_box.set(-6.0, -4.0, 0.0);
    IVP_Polygon *box = create_dynamic_box(env, &mat_dyn, 0.7, 0.7, 0.7,
                                          1.2, 0.32, 0.32, 0.32,
                                          0.0, &rot_damp_zero,
                                          &q_ident, &pos_box);

    IVP_Template_Controller_Motion tcm;
    tcm.force_factor = 0.8f;
    tcm.damp_factor = 1.2f;
    tcm.torque_factor = 0.8f;
    tcm.angular_damp_factor = 1.2f;
    tcm.max_translation_force.set(200.0f, 200.0f, 200.0f);
    tcm.max_torque = 80.0f;
    IVP_Controller_Motion *cm = IVP_Controller_Factory::create_controller_motion(env, box, &tcm);
    IVP_U_Point target; target.set(6.0, -4.0, 0.0);
    cm->set_target_position_ws(&target);

    scene->objects[idx] = box; scene->types[idx++] = "motion_box";
    scene->count = idx;
}

static void setup_phantom(IVP_Environment *env, SceneObjects *scene) {
    // Zero gravity (as libivp's test_scenarios run_phantom): with gravity the
    // ball hit the ground before reaching the trigger volume.
    IVP_U_Point zero_gravity;
    zero_gravity.set(0.0, 0.0, 0.0);
    env->set_gravity(&zero_gravity);

    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.05);

    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_static, 20.0, 0.5, 20.0, &q_ident, &pos_ground); scene->types[idx++] = "ground";

    IVP_Compact_Surface *trigger_compact = build_box_compact_surface(2.5, 2.5, 2.5);
    IVP_SurfaceManager_Polygon *trigger_surman = new IVP_SurfaceManager_Polygon(trigger_compact);
    IVP_Template_Real_Object trigger_templ;
    configure_static_template(&trigger_templ, &mat_static);
    trigger_templ.set_nocoll_group_ident("phntm");
    IVP_U_Point pos_trigger; pos_trigger.set(0.0, -6.0, 0.0);
    IVP_Polygon *trigger = env->create_polygon(trigger_surman, &trigger_templ, &q_ident, &pos_trigger);
    trigger->enable_collision_detection(IVP_TRUE);

    IVP_Template_Real_Object ball_templ;
    configure_dynamic_template(&ball_templ, &mat_dyn, 1.0);
    ball_templ.set_nocoll_group_ident("phntm");
    configure_explicit_inertia(&ball_templ, 0.1, 0.1, 0.1);
    IVP_Template_Ball ball_shape;
    ball_shape.radius = 0.5f;
    IVP_U_Point pos_ball; pos_ball.set(-9.0, -6.0, 0.0);
    IVP_Ball *ball = env->create_ball(&ball_shape, &ball_templ, &q_ident, &pos_ball);
    wake_and_enable(ball);
    ball->get_core()->speed.set(5.0f, 0.0f, 0.0f);

    scene->objects[idx] = trigger; scene->types[idx++] = "phantom";
    scene->objects[idx] = ball;    scene->types[idx++] = "ball";
    scene->count = idx;
}

bool setup_controller_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    if (scenario == SCENARIO_BUOYANCY) {
        setup_buoyancy(env, scene, resources);
        return true;
    }
    if (scenario == SCENARIO_FORCE_ACTUATOR) {
        setup_force_actuator_like(env, scene, IVP_FALSE);
        return true;
    }
    if (scenario == SCENARIO_FORCEFIELD) {
        setup_force_actuator_like(env, scene, IVP_TRUE);
        return true;
    }
    if (scenario == SCENARIO_MOTION_CONTROLLER) {
        setup_motion_controller(env, scene);
        return true;
    }
    if (scenario == SCENARIO_PHANTOM) {
        setup_phantom(env, scene);
        return true;
    }
    return false;
}

} // namespace ref_runner
