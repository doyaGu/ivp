#include "ref_runner.hxx"

#include <cmath>
#include <cstdio>

namespace ref_runner {

static void setup_springs(IVP_Environment *env, SceneObjects *scene) {
    IVP_U_Point gravity;
    gravity.set(0.0, 3.0, 0.0);
    env->set_gravity(&gravity);

    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;

    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_node(0.7, 0.05);
    static IVP_Material_Simple mat_ball(0.5, 0.0);
    static IVP_Material_Simple mat_bungee(0.6, 0.0);

    IVP_U_Point pos_ground; pos_ground.set(0.0, 0.0, 0.0);
    IVP_Polygon *ground = create_static_box(env, &mat_static, 15.0, 0.5, 15.0, &q_ident, &pos_ground);

    IVP_U_Point pos_pillar_l; pos_pillar_l.set(-6.0, -7.0, 0.0);
    IVP_Polygon *pillar_l = create_static_box(env, &mat_static, 0.5, 2.0, 0.5, &q_ident, &pos_pillar_l);
    IVP_U_Point pos_pillar_r; pos_pillar_r.set(6.0, -7.0, 0.0);
    IVP_Polygon *pillar_r = create_static_box(env, &mat_static, 0.5, 2.0, 0.5, &q_ident, &pos_pillar_r);

    static const int N_BRIDGE = 4;
    const double node_spacing = 12.0 / (N_BRIDGE + 1.0);
    double bridge_rest_len = node_spacing - 1.0;
    if (bridge_rest_len < 0.3) bridge_rest_len = 0.3;

    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);
    IVP_Real_Object *nodes[N_BRIDGE] = {0, 0, 0, 0};
    for (int i = 0; i < N_BRIDGE; ++i) {
        const double x = -6.0 + node_spacing * (i + 1.0);
        IVP_U_Point pos; pos.set(x, -7.0, 0.0);
        nodes[i] = create_dynamic_box(env, &mat_node, 0.5, 0.5, 0.5,
                                      0.6, 0.07, 0.07, 0.07,
                                      0.15, &rot_damp_zero,
                                      &q_ident, &pos);
    }

    IVP_U_Point pos_ball; pos_ball.set(0.0, -10.0, 0.0);
    IVP_Ball *ball = create_dynamic_ball(env, &mat_ball, 0.8, 0.5, 0.48, 0.48, 0.48,
                                         0.0, &rot_damp_zero, &q_ident, &pos_ball);

    IVP_U_Point pos_bungee; pos_bungee.set(10.0, -6.0, 0.0);
    IVP_Polygon *bungee = create_dynamic_box(env, &mat_bungee, 0.6, 0.6, 0.6,
                                             3.0, 0.36, 0.36, 0.36,
                                             0.0, &rot_damp_zero,
                                             &q_ident, &pos_bungee);

    IVP_U_Point pos_anchor; pos_anchor.set(10.0, -14.0, 0.0);
    IVP_Polygon *anchor = create_static_box(env, &mat_static, 0.3, 0.3, 0.3, &q_ident, &pos_anchor);

    add_spring(env, pillar_l, 0.5, 0.0, 0.0, nodes[0], -0.5, 0.0, 0.0, 120.0, 30.0, bridge_rest_len);
    for (int i = 1; i < N_BRIDGE; ++i) {
        add_spring(env, nodes[i - 1], 0.5, 0.0, 0.0, nodes[i], -0.5, 0.0, 0.0, 120.0, 30.0, bridge_rest_len);
    }
    add_spring(env, nodes[N_BRIDGE - 1], 0.5, 0.0, 0.0, pillar_r, -0.5, 0.0, 0.0, 120.0, 30.0, bridge_rest_len);
    add_spring(env, anchor, 0.0, 0.3, 0.0, bungee, 0.0, -0.6, 0.0, 8.0, 6.0, 8.0);

    int idx = 0;
    scene->objects[idx] = ground;    scene->types[idx++] = "ground";
    scene->objects[idx] = pillar_l;  scene->types[idx++] = "pillar";
    scene->objects[idx] = pillar_r;  scene->types[idx++] = "pillar";
    for (int i = 0; i < N_BRIDGE; ++i) {
        scene->objects[idx] = nodes[i]; scene->types[idx++] = "node";
    }
    scene->objects[idx] = ball;      scene->types[idx++] = "ball";
    scene->objects[idx] = bungee;    scene->types[idx++] = "bungee";
    scene->objects[idx] = anchor;    scene->types[idx++] = "anchor";
    scene->count = idx;
}

static void setup_rope(IVP_Environment *env, SceneObjects *scene) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;

    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_link(0.6, 0.05);
    static IVP_Material_Simple mat_weight(0.8, 0.1);

    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    IVP_Polygon *ground = create_static_box(env, &mat_static, 12.0, 0.5, 12.0, &q_ident, &pos_ground);

    IVP_U_Point pos_ceiling; pos_ceiling.set(0.0, -15.0, 0.0);
    IVP_Polygon *ceiling = create_static_box(env, &mat_static, 1.5, 0.3, 0.3, &q_ident, &pos_ceiling);

    static const int CHAIN_LEN = 8;
    const double step_y = 2.0 * 0.55 + 0.05;

    const double m = 0.6;
    const double hx = 0.2, hy = 0.55, hz = 0.2;
    const double ix = m / 3.0 * (hy * hy + hz * hz);
    const double iy = m / 3.0 * (hx * hx + hz * hz);
    const double iz = m / 3.0 * (hx * hx + hy * hy);

    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);
    IVP_Real_Object *links[CHAIN_LEN] = {0};
    for (int i = 0; i < CHAIN_LEN; ++i) {
        const double y = -15.0 + step_y * (i + 1.0);
        IVP_U_Point pos; pos.set(0.0, y, 0.0);
        links[i] = create_dynamic_box(env, &mat_link, 0.2, 0.55, 0.2,
                                      0.6, ix, iy, iz,
                                      0.03, &rot_damp_zero,
                                      &q_ident, &pos);
    }

    const double weight_y = -15.0 + step_y * (CHAIN_LEN + 1.0);
    IVP_U_Point pos_weight; pos_weight.set(0.0, weight_y, 0.0);
    IVP_Polygon *weight = create_dynamic_box(env, &mat_weight, 0.6, 0.6, 0.6,
                                             2.0, 0.48, 0.48, 0.48,
                                             0.02, &rot_damp_zero,
                                             &q_ident, &pos_weight);

    {
        IVP_U_Point a; a.set(0.0, 0.3, 0.0);
        IVP_U_Point b; b.set(0.0, -0.55, 0.0);
        add_ballsocket_constraint_two_anchors(env, ceiling, &a, links[0], &b);
    }
    for (int i = 1; i < CHAIN_LEN; ++i) {
        IVP_U_Point a; a.set(0.0, 0.55, 0.0);
        IVP_U_Point b; b.set(0.0, -0.55, 0.0);
        add_ballsocket_constraint_two_anchors(env, links[i - 1], &a, links[i], &b);
    }
    {
        IVP_U_Point a; a.set(0.0, 0.55, 0.0);
        IVP_U_Point b; b.set(0.0, -0.6, 0.0);
        add_ballsocket_constraint_two_anchors(env, links[CHAIN_LEN - 1], &a, weight, &b);
    }

    int idx = 0;
    scene->objects[idx] = ground;   scene->types[idx++] = "ground";
    scene->objects[idx] = ceiling;  scene->types[idx++] = "ceiling";
    for (int i = 0; i < CHAIN_LEN; ++i) {
        scene->objects[idx] = links[i]; scene->types[idx++] = "link";
    }
    scene->objects[idx] = weight;   scene->types[idx++] = "weight";
    scene->count = idx;
}

static void setup_motor(IVP_Environment *env, SceneObjects *scene) {
    const char *motor_internal = "motint";

    if (ref_debug_enabled()) {
        std::fprintf(stderr, "setup_motor: begin\n");
        std::fflush(stderr);
    }

    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_blade(0.5, 0.35);
    static IVP_Material_Simple mat_target(0.5, 0.2);

    IVP_U_Point pos_ground; pos_ground.set(0.0, 0.0, 0.0);
    IVP_Polygon *ground = create_static_box(env, &mat_static, 15.0, 0.5, 15.0, &q_ident, &pos_ground);

    IVP_Compact_Surface *ped_compact = build_box_compact_surface(0.15, 0.8, 0.15);
    IVP_SurfaceManager_Polygon *ped_surman = new IVP_SurfaceManager_Polygon(ped_compact);
    IVP_Template_Real_Object templ_ped;
    configure_static_template(&templ_ped, &mat_static);
    templ_ped.set_nocoll_group_ident(motor_internal);
    IVP_U_Point pos_ped; pos_ped.set(0.0, -5.6, 0.0);
    IVP_Polygon *pedestal = env->create_polygon(ped_surman, &templ_ped, &q_ident, &pos_ped);
    pedestal->enable_collision_detection(IVP_TRUE);

    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);
    IVP_Compact_Surface *blade_compact = build_box_compact_surface(4.0, 0.25, 0.4);
    IVP_SurfaceManager_Polygon *blade_surman = new IVP_SurfaceManager_Polygon(blade_compact);
    IVP_Template_Real_Object templ_blade;
    configure_dynamic_template(&templ_blade, &mat_blade, 1.2);
    configure_explicit_inertia(&templ_blade, 0.089, 6.464, 6.425);
    templ_blade.speed_damp_factor = 0.0;
    templ_blade.rot_speed_damp_factor = rot_damp_zero;
    templ_blade.set_nocoll_group_ident(motor_internal);
    IVP_U_Point pos_blade; pos_blade.set(0.0, -6.8, 0.0);
    IVP_Polygon *blade = env->create_polygon(blade_surman, &templ_blade, &q_ident, &pos_blade);
    wake_and_enable(blade);

    static const int N_TARGETS = 8;
    IVP_Real_Object *targets[N_TARGETS] = {0};
    for (int i = 0; i < N_TARGETS; ++i) {
        const double angle = 2.0 * 3.14159265358979323846 * (double)i / (double)N_TARGETS;
        const double x = 3.5 * std::cos(angle);
        const double z = 3.5 * std::sin(angle);
        IVP_U_Point pos; pos.set(x, -7.9, z); // above the blade (synced with libivp test_scenarios motor)
        targets[i] = create_dynamic_box(env, &mat_target, 0.4, 0.4, 0.4,
                                        0.06, 0.05, 0.05, 0.05,
                                        0.0, &rot_damp_zero,
                                        &q_ident, &pos);
    }

    {
        IVP_U_Point anchor_ws; anchor_ws.set(0.0, -6.8, 0.0);
        IVP_U_Point axis_ws; axis_ws.set(0.0, 1.0, 0.0);
        IVP_Template_Constraint tc;
        tc.set_hinge_ws(pedestal, &anchor_ws, &axis_ws, blade);
        tc.force_factor = 0.25f;
        tc.damp_factor = 0.95f;
        IVP_Controller_Factory::create_constraint(env, &tc);
    }

    {
        IVP_Template_Anchor a0;
        IVP_Template_Anchor a1;
        IVP_Template_Rot_Mot tm;
        tm.anchors[0] = &a0;
        tm.anchors[1] = &a1;

        a0.set_anchor_position_os(blade, 0.0, 0.0, 0.0);
        a1.set_anchor_position_os(blade, 0.0, 1.0, 0.0);

        tm.power = 3.0f;
        tm.max_torque = 4.0f;
        tm.max_rotation_speed = 1.6f;
        IVP_Controller_Factory::create_rotmot(env, &tm);
    }

    int idx = 0;
    scene->objects[idx] = ground;    scene->types[idx++] = "ground";
    scene->objects[idx] = pedestal;  scene->types[idx++] = "pedestal";
    scene->objects[idx] = blade;     scene->types[idx++] = "blade";
    for (int i = 0; i < N_TARGETS; ++i) {
        scene->objects[idx] = targets[i]; scene->types[idx++] = "target";
    }
    scene->count = idx;
}

static void setup_stiff_spring(IVP_Environment *env, SceneObjects *scene) {
    IVP_U_Quat q_ident; q_ident.x = q_ident.y = q_ident.z = 0.0; q_ident.w = 1.0;
    static IVP_Material_Simple mat_static(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.6, 0.05);
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);

    int idx = 0;
    IVP_U_Point pos_anchor_l; pos_anchor_l.set(-4.0, -8.0, 0.0);
    IVP_Polygon *anchor_l = create_static_box(env, &mat_static, 0.4, 0.4, 0.4, &q_ident, &pos_anchor_l);
    IVP_U_Point pos_anchor_r; pos_anchor_r.set(4.0, -8.0, 0.0);
    IVP_Polygon *anchor_r = create_static_box(env, &mat_static, 0.4, 0.4, 0.4, &q_ident, &pos_anchor_r);

    IVP_U_Point pos_soft; pos_soft.set(-2.0, -10.5, 0.0);
    IVP_Polygon *soft = create_dynamic_box(env, &mat_dyn, 0.5, 0.5, 0.5,
                                           0.8, 0.12, 0.12, 0.12,
                                           0.0, &rot_damp_zero,
                                           &q_ident, &pos_soft);

    IVP_U_Point pos_stiff; pos_stiff.set(2.0, -10.5, 0.0);
    IVP_Polygon *stiff = create_dynamic_box(env, &mat_dyn, 0.5, 0.5, 0.5,
                                            0.8, 0.12, 0.12, 0.12,
                                            0.0, &rot_damp_zero,
                                            &q_ident, &pos_stiff);

    add_spring(env, anchor_l, 0, 0, 0, soft, 0, 0, 0, 25.0, 6.0, 2.5);
    add_spring(env, anchor_r, 0, 0, 0, stiff, 0, 0, 0, 320.0, 45.0, 2.5);

    scene->objects[idx] = anchor_l; scene->types[idx++] = "anchor";
    scene->objects[idx] = anchor_r; scene->types[idx++] = "anchor";
    scene->objects[idx] = soft;     scene->types[idx++] = "soft";
    scene->objects[idx] = stiff;    scene->types[idx++] = "stiff";
    scene->count = idx;
}

bool setup_constraint_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources * /*resources*/) {
    if (scenario == SCENARIO_SPRINGS) {
        setup_springs(env, scene);
        return true;
    }
    if (scenario == SCENARIO_ROPE) {
        setup_rope(env, scene);
        return true;
    }
    if (scenario == SCENARIO_MOTOR || scenario == SCENARIO_CHECK_DISTANCE) {
        setup_motor(env, scene);
        return true;
    }
    if (scenario == SCENARIO_STIFF_SPRING) {
        setup_stiff_spring(env, scene);
        return true;
    }
    return false;
}

} // namespace ref_runner
