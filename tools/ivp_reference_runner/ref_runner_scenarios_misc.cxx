#include "ref_runner.hxx"

#include <ivp_universe_manager.hxx>

/* Miscellaneous parity scenarios (libivp tests/test_scenarios.c keeps the
 * same literals):
 *   universe_evict  a custom IVP_Universe_Manager streams static tiles in
 *                   (ensure_objects_in_environment) and out
 *                   (object_no_longer_needed) while balls and boxes cross the
 *                   tile field; the tile heights depend on the order in which
 *                   IVP_Cluster_Manager::check_for_unused_objects visits the
 *                   objects (cluster tree order). */

namespace ref_runner {

namespace {

/* IVP_Environment::universe_manager is private and only set from the
 * IVP_Application_Environment; this scenario installs its manager after the
 * scene is built (member pointer obtained through an explicit template
 * instantiation, which is exempt from access checking). */
template <typename Tag, typename Tag::type M>
struct MemberAccess {
    friend typename Tag::type member_pointer(Tag) { return M; }
};
struct EnvUniverseManager {
    typedef IVP_Universe_Manager *IVP_Environment::*type;
    friend type member_pointer(EnvUniverseManager);
};
template struct MemberAccess<EnvUniverseManager, &IVP_Environment::universe_manager>;

void set_universe_manager(IVP_Environment *env, IVP_Universe_Manager *um) {
    env->*member_pointer(EnvUniverseManager()) = um;
}

/* ── universe_evict ──────────────────────────────────────────────────── */

enum { UE_N = 5, UE_SLOTS = UE_N * UE_N, UE_HEIGHTS = 5 };

struct UeSlot {
    IVP_Real_Object *obj;
    int generation; /* index of the eviction that removed the tile last */
    int touches;    /* second level checks (one collision partner) */
    double x, z;
};

class UniverseEvictManager : public IVP_Universe_Manager {
public:
    IVP_Environment *env;
    IVP_Material *mat;
    IVP_SurfaceManager_Polygon *surmans[UE_HEIGHTS];
    UeSlot slots[UE_SLOTS];
    IVP_Universe_Manager_Settings settings;
    int evict_counter;

    UniverseEvictManager() : env(0), mat(0), evict_counter(0) {
        for (int i = 0; i < UE_HEIGHTS; ++i) surmans[i] = 0;
        settings.num_objects_in_environment_threshold_0 = 1;
        settings.check_objects_per_second_threshold_0 = 450; /* int(450 / 180) + 1 = 3 checks per PSI */
        settings.num_objects_in_environment_threshold_1 = 14;
        settings.check_objects_per_second_threshold_1 = 630; /* 4 checks per PSI */
    }

    static double half_height(int h) { return 0.02 + 0.02 * (double)h; }

    void create_tile(int i) {
        UeSlot *s = &slots[i];
        int h = (s->generation + s->touches) % UE_HEIGHTS;
        IVP_Template_Real_Object templ;
        configure_static_template(&templ, mat);
        IVP_U_Quat q;
        q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
        IVP_U_Point pos;
        pos.set(s->x, -half_height(h), s->z);
        IVP_Polygon *o = env->create_polygon(surmans[h], &templ, &q, &pos);
        o->enable_collision_detection(IVP_TRUE);
        s->obj = o;
    }

    void ensure_objects_in_environment(IVP_Real_Object *, IVP_U_Float_Point *center, IVP_DOUBLE radius) {
        const double reach = radius + 1.0; /* tile half size 0.9 */
        for (int i = 0; i < UE_SLOTS; ++i) {
            UeSlot *s = &slots[i];
            if (s->obj) continue;
            const double dx = s->x - (double)center->k[0];
            const double dz = s->z - (double)center->k[2];
            if (dx * dx + dz * dz > reach * reach) continue;
            create_tile(i);
        }
    }

    void object_no_longer_needed(IVP_Real_Object *object) {
        for (int i = 0; i < UE_SLOTS; ++i) {
            UeSlot *s = &slots[i];
            if (s->obj != object) continue;
            if (object->get_collision_check_reference_count() == 0) {
                s->obj = 0;
                s->generation = evict_counter++;
                object->delete_and_check_vicinity();
            } else {
                s->touches++;
            }
            return;
        }
    }

    void event_object_deleted(IVP_Real_Object *object) {
        for (int i = 0; i < UE_SLOTS; ++i) {
            if (slots[i].obj == object) slots[i].obj = 0;
        }
    }

    const IVP_Universe_Manager_Settings *provide_universe_settings() { return &settings; }
};

struct UniverseEvictScenario {
    IVP_Environment *env;
    UniverseEvictManager manager;
};

void universe_evict_cleanup_hook(ScenarioResources *resources) {
    UniverseEvictScenario *ue = (UniverseEvictScenario *)resources->scenario_data;
    if (!ue) return;
    set_universe_manager(ue->env, 0); /* the environment outlives the manager */
    delete ue;
    resources->scenario_data = 0;
}

IVP_Ball *ue_ball(IVP_Environment *env, IVP_Material *mat, double radius, double mass, double inertia,
                  double x, double y, double z, float vx, float vy, float vz) {
    IVP_U_Quat q;
    q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);
    IVP_U_Point p; p.set(x, y, z);
    IVP_Ball *b = create_dynamic_ball(env, mat, radius, mass, inertia, inertia, inertia, 0.0, &rot_damp_zero, &q, &p);
    b->get_core()->speed.set(vx, vy, vz);
    return b;
}

IVP_Polygon *ue_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz, double mass,
                    double ix, double iy, double iz, const IVP_U_Quat *q,
                    double x, double y, double z, float vx, float vy, float vz) {
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);
    IVP_U_Point p; p.set(x, y, z);
    IVP_Polygon *b = create_dynamic_box(env, mat, hx, hy, hz, mass, ix, iy, iz, 0.0, &rot_damp_zero, q, &p);
    b->get_core()->speed.set(vx, vy, vz);
    return b;
}

void setup_universe_evict(IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    static IVP_Material_Simple mat_ground(0.3, 0.3);
    static IVP_Material_Simple mat_tile(0.3, 0.5);
    static IVP_Material_Simple mat_dyn(0.2, 0.5);

    UniverseEvictScenario *ue = new UniverseEvictScenario;
    ue->env = env;
    UniverseEvictManager *um = &ue->manager;
    um->env = env;
    um->mat = &mat_tile;
    for (int h = 0; h < UE_HEIGHTS; ++h) {
        um->surmans[h] = new IVP_SurfaceManager_Polygon(
            build_box_compact_surface(0.9, UniverseEvictManager::half_height(h), 0.9));
    }

    IVP_U_Quat q_ident;
    q_ident.x = 0.0; q_ident.y = 0.0; q_ident.z = 0.0; q_ident.w = 1.0;
    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 0.5, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_ground, 12.0, 0.5, 12.0, &q_ident, &pos_ground);
    scene->types[idx++] = "ground";

    /* the full tile field; initial heights by slot */
    for (int i = 0; i < UE_SLOTS; ++i) {
        UeSlot *s = &um->slots[i];
        s->obj = 0;
        s->generation = i;
        s->touches = 0;
        s->x = -6.0 + 3.0 * (double)(i % UE_N);
        s->z = -6.0 + 3.0 * (double)(i / UE_N);
        um->create_tile(i);
    }

    scene->objects[idx] = ue_ball(env, &mat_dyn, 0.3, 1.0, 0.036, -7.5, -0.6, -6.0, 8.0f, 0.0f, 1.5f);
    scene->types[idx++] = "ball";
    scene->objects[idx] = ue_ball(env, &mat_dyn, 0.25, 0.6, 0.015, 7.0, -0.8, -3.0, -8.5f, -2.0f, 0.75f);
    scene->types[idx++] = "ball";
    scene->objects[idx] = ue_ball(env, &mat_dyn, 0.35, 1.5, 0.0735, 0.0, -1.5, 7.5, 0.5f, 0.0f, -9.0f);
    scene->types[idx++] = "ball";
    scene->objects[idx] = ue_box(env, &mat_dyn, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, &q_ident,
                                 -6.0, -0.5, 6.0, 7.0f, -1.0f, -6.0f);
    scene->types[idx++] = "box";
    IVP_U_Quat q5;
    q5.x = 0.0; q5.y = 0.25881904510252074; q5.z = 0.0; q5.w = 0.9659258262890683;
    scene->objects[idx] = ue_box(env, &mat_dyn, 0.4, 0.2, 0.3, 1.2, 0.052, 0.1, 0.08, &q5,
                                 6.0, -0.6, 6.0, -6.5f, -1.5f, -6.5f);
    scene->types[idx++] = "box";
    scene->count = idx;

    set_universe_manager(env, um);
    resources->scenario_data = ue;
    resources->cleanup_hook = universe_evict_cleanup_hook;
}

} // namespace

bool setup_misc_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    if (scenario == SCENARIO_UNIVERSE_EVICT) {
        setup_universe_evict(env, scene, resources);
        return true;
    }
    return false;
}

} // namespace ref_runner
