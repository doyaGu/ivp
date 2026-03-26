// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.
// Factory methods for IVP_Environment that instantiate concrete controller types.
// Extracted from ivp_environment.cxx to break ivp_physics → ivp_controller cycle.

#include <ivp_controller_factory.hxx>
#include <ivp_physics.hxx>
#include <ivp_actuator.hxx>
#include <ivp_actuator_spring.hxx>
#include <ivp_controller_motion.hxx>
#include <ivp_controller_buoyancy.hxx>
#include <ivp_attacher_to_cores.hxx>
#include <ivu_set.hxx>

class IVP_Mindist_Base;

IVP_Actuator_Rot_Mot *IVP_Controller_Factory::create_rotmot(IVP_Environment *env, IVP_Template_Rot_Mot *templ)
{
    if (templ->active_float_max_torque ||
        templ->active_float_max_rotation_speed ||
        templ->active_float_power)
    {
        return new IVP_Actuator_Rot_Mot_Active(env, templ);
    }
    return new IVP_Actuator_Rot_Mot(env, templ);
}

IVP_Actuator_Torque *IVP_Controller_Factory::create_torque(IVP_Environment *env, IVP_Template_Torque *templ)
{
    if (templ->active_float_torque || templ->active_float_max_rotation_speed)
    {
        return new IVP_Actuator_Torque_Active(env, templ);
    }
    return new IVP_Actuator_Torque(env, templ);
}

IVP_Controller_Motion *IVP_Controller_Factory::create_controller_motion(IVP_Environment *, IVP_Real_Object *obj, const IVP_Template_Controller_Motion *templ)
{
    return new IVP_Controller_Motion(obj, templ);
}

IVP_Actuator_Force *IVP_Controller_Factory::create_force(IVP_Environment *env, IVP_Template_Force *templ)
{
    if (templ->active_float_force)
    {
        return new IVP_Actuator_Force_Active(env, templ);
    }
    else
    {
        return new IVP_Actuator_Force(env, templ);
    }
}

IVP_Actuator_Spring *IVP_Controller_Factory::create_spring(IVP_Environment *env, IVP_Template_Spring *templ)
{
    if (templ->active_float_spring_len || templ->active_float_spring_constant || templ->active_float_spring_damp || templ->active_float_spring_rel_pos_damp)
    {
        return new IVP_Actuator_Spring_Active(env, templ);
    }
    else
    {
        return new IVP_Actuator_Spring(env, templ, IVP_ACTUATOR_TYPE_SPRING);
    }
}

IVP_Actuator_Suspension *IVP_Controller_Factory::create_suspension(IVP_Environment *env, IVP_Template_Suspension *templ)
{
    return new IVP_Actuator_Suspension(env, templ);
}

IVP_Actuator_Stabilizer *IVP_Controller_Factory::create_stabilizer(IVP_Environment *env, IVP_Template_Stabilizer *templ)
{
    return new IVP_Actuator_Stabilizer(env, templ);
}

IVP_Actuator_Check_Dist *IVP_Controller_Factory::create_check_dist(IVP_Environment *env, IVP_Template_Check_Dist *templ)
{
    return new IVP_Actuator_Check_Dist(env, templ);
}

// Force template instantiation for buoyancy attacher
void ivp_dummy_func()
{
    IVP_U_Set_Active<IVP_Core> ivp_class_dummy1(16);
    IVP_U_Set_Active<IVP_Real_Object> ivp_class_dummy2(16);
    IVP_U_Set_Active<IVP_Mindist_Base> ivp_class_dummy3(16);
    IVP_Attacher_To_Cores<IVP_Controller_Buoyancy> *ivp_class_dummy4 = NULL;
    (void)ivp_class_dummy1;
    (void)ivp_class_dummy2;
    (void)ivp_class_dummy3;
    (void)ivp_class_dummy4;
}
