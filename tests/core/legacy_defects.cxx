#include <ivp_physics.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_3d_solver.hxx>
#include <ivp_ball.hxx>
#include <ivp_constraint_car.hxx>
#include <ivp_environment.hxx>
#include <ivp_gridbuild_array.hxx>
#include <ivp_material.hxx>
#include <ivp_merge_core.hxx>
#include <ivp_object_attach.hxx>
#include <ivp_sphere_solver.hxx>
#include <ivp_surman_grid.hxx>
#include <ivp_templates.hxx>
#include <ivp_time.hxx>
#include <ivu_memory.hxx>

#include <cmath>
#include <cstdio>
#include <cstring>

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
    return 1; } } while (0)

static IVP_Environment *make_environment(const char *name)
{
    IVP_Application_Environment application_environment;
    return IVP_Environment_Manager::get_environment_manager()->create_environment(
        &application_environment, name, 0);
}

static IVP_Ball *make_ball(IVP_Environment *environment,
                           IVP_Material *material,
                           IVP_DOUBLE x,
                           IVP_DOUBLE y,
                           IVP_DOUBLE z,
                           IVP_BOOL simulated)
{
    IVP_Template_Ball ball_template;
    ball_template.radius = 0.5f;

    IVP_Template_Real_Object object_template;
    object_template.material = material;
    object_template.mass = 1.0;
    object_template.rot_inertia_is_factor = IVP_FALSE;
    object_template.rot_inertia.set(0.1f, 0.1f, 0.1f);

    IVP_U_Quat rotation;
    rotation.init();
    IVP_U_Point position(x, y, z);
    IVP_Ball *ball = environment->create_ball(
        &ball_template, &object_template, &rotation, &position);
    if (simulated)
    {
        ball->ensure_in_simulation_now();
    }
    return ball;
}

class Recording_Time_Event : public IVP_Time_Event
{
public:
    int fired;
    IVP_DOUBLE fired_at;

    Recording_Time_Event() : fired(0), fired_at(-1.0) {}

    virtual void simulate_time_event(IVP_Environment *environment)
    {
        ++fired;
        fired_at = environment->get_current_time().get_time();
    }
};

static int test_time_event()
{
    IVP_Environment *environment = make_environment("legacy-time-event-test");
    environment->simulate_until(IVP_Time(3.0 / 66.0));

    Recording_Time_Event event;
    IVP_Time due = environment->get_current_time() + 0.1;
    environment->get_time_manager()->insert_event(&event, due);
    environment->simulate_until(due + 0.05);

    CHECK(event.fired == 1);
    CHECK(std::fabs(event.fired_at - due.get_time()) < 1e-6);
    delete environment;
    return 0;
}

static int test_car_builder()
{
    IVP_Environment *environment = make_environment("legacy-car-builder-test");
    IVP_Material_Simple material(0.5, 0.0);
    IVP_Real_Object *body = make_ball(environment, &material, 0.0, 0.0, 0.0, IVP_FALSE);

    IVP_U_Vector<IVP_Real_Object> wheels;
    IVP_U_Vector<IVP_U_Float_Point> hardpoints;
    IVP_U_Float_Point hardpoint_storage[4];
    const IVP_DOUBLE positions[4][3] = {
        {-1.0, 0.5, 1.0},
        { 1.0, 0.5, 1.0},
        {-1.0, 0.5, -1.0},
        { 1.0, 0.5, -1.0}
    };
    for (int i = 0; i < 4; ++i)
    {
        wheels.add(make_ball(environment, &material,
                             positions[i][0], positions[i][1], positions[i][2],
                             IVP_FALSE));
        hardpoint_storage[i].set(
            (IVP_FLOAT)positions[i][0],
            (IVP_FLOAT)positions[i][1],
            (IVP_FLOAT)positions[i][2]);
        hardpoints.add(&hardpoint_storage[i]);
    }

    int result = 1;
    {
        IVP_Constraint_Solver_Car solver(
            IVP_INDEX_X, IVP_INDEX_Y, IVP_INDEX_Z, IVP_FALSE);
        result = solver.init_constraint_system(
            environment, body, wheels, hardpoints) == IVP_OK ? 0 : 1;
    }
    delete environment;
    CHECK(result == 0);
    return 0;
}

static int test_merge_duplicate()
{
    IVP_Environment *environment = make_environment("legacy-merge-test");
    IVP_Material_Simple material(0.5, 0.0);

    IVP_U_Vector<IVP_Real_Object> empty;
    environment->merge_objects(&empty);

    IVP_Real_Object *object = make_ball(
        environment, &material, 0.0, 0.0, 0.0, IVP_FALSE);
    IVP_Core *original_core = object->get_core();

    IVP_U_Vector<IVP_Real_Object> objects;
    objects.add(object);
    objects.add(object);
    environment->merge_objects(&objects);

    CHECK(object->get_core() == original_core);
    CHECK(original_core->objects.len() == 1);

    IVP_Real_Object *moving = make_ball(
        environment, &material, 2.0, 0.0, 0.0, IVP_TRUE);
    IVP_Core *moving_core = moving->get_core();
    IVP_U_Vector<IVP_Real_Object> moving_objects;
    moving_objects.add(moving);
    environment->merge_objects(&moving_objects);
    CHECK(moving->get_core() == moving_core);
    CHECK(moving_core->objects.len() == 1);

    IVP_Real_Object *shared_first = make_ball(
        environment, &material, 4.0, 0.0, 0.0, IVP_FALSE);
    IVP_Real_Object *shared_second = make_ball(
        environment, &material, 6.0, 0.0, 0.0, IVP_FALSE);
    IVP_U_Vector<IVP_Real_Object> pair;
    pair.add(shared_first);
    pair.add(shared_second);
    environment->merge_objects(&pair);
    IVP_Core *shared_core = shared_first->get_core();
    CHECK(shared_second->get_core() == shared_core);
    CHECK(shared_core->objects.len() == 2);

    IVP_U_Vector<IVP_Real_Object> shared_objects;
    shared_objects.add(shared_first);
    environment->merge_objects(&shared_objects);
    CHECK(shared_first->get_core() == shared_core);
    CHECK(shared_second->get_core() == shared_core);
    CHECK(shared_core->objects.len() == 2);

    IVP_U_Vector<IVP_Real_Object> many_objects;
    for (int i = 0; i < 7; ++i)
    {
        many_objects.add(make_ball(
            environment, &material, 10.0 + i * 2.0, 0.0, 0.0, IVP_FALSE));
    }
    environment->merge_objects(&many_objects);
    IVP_Core *many_core = many_objects.element_at(0)->get_core();
    CHECK(many_core->objects.len() == 7);
    for (int i = 1; i < many_objects.len(); ++i)
    {
        CHECK(many_objects.element_at(i)->get_core() == many_core);
    }

    delete environment;
    return 0;
}

static int test_attach_self()
{
    IVP_Environment *environment = make_environment("legacy-attach-test");
    IVP_Material_Simple material(0.5, 0.0);
    IVP_Real_Object *object = make_ball(
        environment, &material, 0.0, 0.0, 0.0, IVP_FALSE);
    IVP_Core *original_core = object->get_core();

    IVP_Object_Attach::attach_object(object, object, -1.0);

    CHECK(object->get_core() == original_core);
    CHECK(original_core->objects.len() == 1);
    delete environment;
    return 0;
}

static int test_matrix_cache()
{
    IVP_Environment *environment = make_environment("legacy-cache-test");
    IVP_Material_Simple material(0.5, 0.0);
    IVP_Real_Object *object = make_ball(
        environment, &material, 0.0, 0.0, 0.0, IVP_FALSE);
    IVP_Cache_Object *cache_object = object->get_cache_object();
    IVP_U_Matrix_Cache cache(cache_object);
    IVP_U_Matrix *matrix = cache.calc_matrix_at(
        environment->get_current_time() + 1.0,
        IVP_3D_SOLVER_MAX_STEPS_PER_PSI + 1);
    CHECK(matrix != NULL);
    matrix = cache.calc_matrix_at(environment->get_current_time() - 1.0, -1);
    CHECK(matrix != NULL);
    cache_object->remove_reference();
    delete environment;
    return 0;
}

static int test_sphere_grid()
{
    IVP_Environment *environment = make_environment("legacy-sphere-grid-test");
    IVP_Material_Simple material(0.5, 0.0);
    IVP_U_Memory memory;
    memory.init_mem();

    IVP_FLOAT heights[100];
    for (int i = 0; i < 100; ++i)
    {
        heights[i] = 0.0f;
    }
    IVP_Template_Compact_Grid grid_template;
    grid_template.row_info.n_points = 10;
    grid_template.row_info.maps_to = IVP_INDEX_X;
    grid_template.row_info.invert_axis = IVP_FALSE;
    grid_template.column_info.n_points = 10;
    grid_template.column_info.maps_to = IVP_INDEX_Z;
    grid_template.column_info.invert_axis = IVP_FALSE;
    grid_template.height_maps_to = IVP_INDEX_Y;
    grid_template.height_invert_axis = IVP_TRUE;
    grid_template.grid_field_size = 1.0f;
    grid_template.position_origin_os.set(0.0f, 0.0f, 0.0f);

    IVP_Compact_Grid *grid = IVP_GridBuilder_Array::convert_array_to_compact_grid(
        &memory, &grid_template, heights);
    CHECK(grid != NULL);

    IVP_Template_Real_Object object_template;
    object_template.material = &material;
    object_template.physical_unmoveable = IVP_TRUE;
    object_template.rot_inertia_is_factor = IVP_FALSE;
    object_template.rot_inertia.set(1.0f, 1.0f, 1.0f);
    IVP_U_Quat rotation;
    rotation.init();
    IVP_U_Point position(0.0, 0.0, 0.0);
    IVP_Real_Object *object = environment->create_polygon(
        new IVP_SurfaceManager_Grid(grid), &object_template, &rotation, &position);

    IVP_Sphere_Solver_Template sphere_template;
    sphere_template.center.set(4.5, 0.0, 4.5);
    sphere_template.radius = 0.75;
    sphere_template.max_traversal_depth = 2;
    IVP_Sphere_Solver solver;
    CHECK(solver.check_sphere_against_object(&sphere_template, object));

    sphere_template.center.set(20.0, 0.0, 20.0);
    CHECK(!solver.check_sphere_against_object(&sphere_template, object));

    delete environment;
    ivp_free_aligned(grid);
    return 0;
}

static int test_merged_core()
{
    IVP_Environment *environment = make_environment("legacy-merged-core-test");
    IVP_Material_Simple material(0.5, 0.0);
    IVP_Real_Object *first = make_ball(
        environment, &material, -1.0, 0.0, 0.0, IVP_FALSE);
    IVP_Real_Object *second = make_ball(
        environment, &material, 1.0, 0.0, 0.0, IVP_FALSE);

    IVP_Core_Merged *merged = new IVP_Core_Merged(
        first->get_core(), second->get_core());
    CHECK(std::fabs(merged->q_world_f_core_next_psi.w) > 0.5);
    delete merged;
    delete environment;
    return 0;
}

static int test_zero_inertia()
{
    IVP_Environment *environment = make_environment("legacy-zero-inertia-test");
    IVP_Material_Simple material(0.5, 0.0);
    IVP_Template_Ball ball_template;
    ball_template.radius = 0.5f;
    IVP_Template_Real_Object object_template;
    object_template.material = &material;
    object_template.mass = 1.0;
    object_template.rot_inertia_is_factor = IVP_FALSE;
    object_template.rot_inertia.set_to_zero();
    IVP_U_Quat rotation;
    rotation.init();
    IVP_U_Point position(0.0, 0.0, 0.0);

    IVP_Real_Object *object = environment->create_ball(
        &ball_template, &object_template, &rotation, &position);
    const IVP_U_Float_Point *inertia = object->get_core()->get_rot_inertia();
    CHECK(inertia->k[0] > 0.0f);
    CHECK(inertia->k[1] > 0.0f);
    CHECK(inertia->k[2] > 0.0f);
    delete environment;
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "usage: %s <defect>\n", argv[0]);
        return 2;
    }
    if (std::strcmp(argv[1], "time_event") == 0) return test_time_event();
    if (std::strcmp(argv[1], "car_builder") == 0) return test_car_builder();
    if (std::strcmp(argv[1], "merge_duplicate") == 0) return test_merge_duplicate();
    if (std::strcmp(argv[1], "attach_self") == 0) return test_attach_self();
    if (std::strcmp(argv[1], "matrix_cache") == 0) return test_matrix_cache();
    if (std::strcmp(argv[1], "sphere_grid") == 0) return test_sphere_grid();
    if (std::strcmp(argv[1], "merged_core") == 0) return test_merged_core();
    if (std::strcmp(argv[1], "zero_inertia") == 0) return test_zero_inertia();
    std::fprintf(stderr, "unknown defect: %s\n", argv[1]);
    return 2;
}
