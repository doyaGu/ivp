// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.
// Factory methods for IVP_Environment that instantiate concrete controller types.
// Extracted from ivp_environment.cxx to break ivp_physics → ivp_controller cycle.

#include <ivp_physics.hxx>
#include <ivp_actuator.hxx>
#include <ivp_actuator_spring.hxx>
#include <ivp_controller_motion.hxx>
#include <ivp_controller_buoyancy.hxx>
#include <ivp_attacher_to_cores.hxx>
#include <ivu_set.hxx>
#include <ivp_mindist_intern.hxx>

IVP_Actuator_Rot_Mot *IVP_Environment::create_rotmot(IVP_Template_Rot_Mot *templ)
{
    if (templ->active_float_max_torque ||
        templ->active_float_max_rotation_speed ||
        templ->active_float_power)
    {
        return new IVP_Actuator_Rot_Mot_Active(this, templ);
    }
    return new IVP_Actuator_Rot_Mot(this, templ);
}

IVP_Actuator_Torque *IVP_Environment::create_torque(IVP_Template_Torque *templ)
{
    if (templ->active_float_torque || templ->active_float_max_rotation_speed)
    {
        return new IVP_Actuator_Torque_Active(this, templ);
    }
    return new IVP_Actuator_Torque(this, templ);
}

IVP_Controller_Motion *IVP_Environment::create_controller_motion(IVP_Real_Object *obj, const IVP_Template_Controller_Motion *templ)
{
    return new IVP_Controller_Motion(obj, templ);
}

IVP_Actuator_Force *IVP_Environment::create_force(IVP_Template_Force *templ)
{
    if (templ->active_float_force)
    {
        return new IVP_Actuator_Force_Active(this, templ);
    }
    else
    {
        return new IVP_Actuator_Force(this, templ);
    }
}

IVP_Actuator_Spring *IVP_Environment::create_spring(IVP_Template_Spring *templ)
{
    if (templ->active_float_spring_len || templ->active_float_spring_constant || templ->active_float_spring_damp || templ->active_float_spring_rel_pos_damp)
    {
        return new IVP_Actuator_Spring_Active(this, templ);
    }
    else
    {
        return new IVP_Actuator_Spring(this, templ, IVP_ACTUATOR_TYPE_SPRING);
    }
}

IVP_Actuator_Suspension *IVP_Environment::create_suspension(IVP_Template_Suspension *templ)
{
    return new IVP_Actuator_Suspension(this, templ);
}

IVP_Actuator_Stabilizer *IVP_Environment::create_stabilizer(IVP_Template_Stabilizer *templ)
{
    return new IVP_Actuator_Stabilizer(this, templ);
}

IVP_Actuator_Check_Dist *IVP_Environment::create_check_dist(IVP_Template_Check_Dist *templ)
{
    return new IVP_Actuator_Check_Dist(this, templ);
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
