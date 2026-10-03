#include <ivp_ball.hxx>
#include <ivp_environment.hxx>
#include <ivp_listener_object.hxx>
#include <ivp_material.hxx>
#include <ivp_real_object.hxx>
#include <ivp_templates.hxx>

#include <cstdio>

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
    return 1; } } while (0)

class Cascading_Deletion_Listener : public IVP_Listener_Object
{
public:
    IVP_Real_Object *first;
    IVP_Real_Object *second;
    int deleted_count;

    Cascading_Deletion_Listener()
        : first(NULL), second(NULL), deleted_count(0)
    {
    }

    virtual void event_object_deleted(IVP_Event_Object *event)
    {
        ++deleted_count;
        if (event->real_object == first && second)
        {
            IVP_Real_Object *to_delete = second;
            second = NULL;
            to_delete->delete_silently();
        }
    }

    virtual void event_object_created(IVP_Event_Object *) {}
    virtual void event_object_revived(IVP_Event_Object *) {}
    virtual void event_object_frozen(IVP_Event_Object *) {}
};

static IVP_Ball *make_ball(IVP_Environment *environment,
                           IVP_Material *material,
                           IVP_DOUBLE x)
{
    IVP_Template_Ball ball_template;
    ball_template.radius = 0.5f;

    IVP_Template_Real_Object object_template;
    object_template.material = material;
    object_template.mass = 1.0;
    object_template.rot_inertia_is_factor = IVP_FALSE;
    object_template.rot_inertia.set(0.1, 0.1, 0.1);

    IVP_U_Quat rotation;
    rotation.init();
    IVP_U_Point position(x, 0.0, 0.0);
    return environment->create_ball(&ball_template, &object_template,
                                    &rotation, &position);
}

int main()
{
    IVP_Application_Environment application_environment;
    IVP_Environment *environment =
        IVP_Environment_Manager::get_environment_manager()->create_environment(
            &application_environment, "deferred-delete-test", 0);
    IVP_Material_Simple material(0.2, 0.5);

    Cascading_Deletion_Listener listener;
    listener.first = make_ball(environment, &material, 0.0);
    listener.second = make_ball(environment, &material, 2.0);
    environment->add_listener_object_global(&listener);

    IVP_Real_Object::begin_deferred_deletion();
    listener.first->delete_silently();
    listener.second->delete_silently();
    IVP_Real_Object::end_deferred_deletion();

    CHECK(listener.deleted_count == 2);
    environment->remove_listener_object_global(&listener);
    delete environment;
    return 0;
}
