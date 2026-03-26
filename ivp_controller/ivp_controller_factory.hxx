// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef IVP_CONTROLLER_FACTORY_INCLUDED
#define IVP_CONTROLLER_FACTORY_INCLUDED

class IVP_Environment;
class IVP_Real_Object;

class IVP_Template_Rot_Mot;
class IVP_Template_Torque;
class IVP_Template_Force;
class IVP_Template_Spring;
class IVP_Template_Suspension;
class IVP_Template_Stabilizer;
class IVP_Template_Check_Dist;
class IVP_Template_Constraint;
class IVP_Template_Controller_Motion;

class IVP_Actuator_Rot_Mot;
class IVP_Actuator_Torque;
class IVP_Controller_Motion;
class IVP_Actuator_Force;
class IVP_Actuator_Spring;
class IVP_Actuator_Suspension;
class IVP_Actuator_Stabilizer;
class IVP_Actuator_Check_Dist;
class IVP_Constraint;

class IVP_Controller_Factory
{
public:
    static IVP_Actuator_Rot_Mot *create_rotmot(IVP_Environment *env, IVP_Template_Rot_Mot *templ);
    static IVP_Actuator_Torque *create_torque(IVP_Environment *env, IVP_Template_Torque *templ);
    static IVP_Controller_Motion *create_controller_motion(IVP_Environment *env, IVP_Real_Object *obj, const IVP_Template_Controller_Motion *templ);
    static IVP_Actuator_Force *create_force(IVP_Environment *env, IVP_Template_Force *templ);
    static IVP_Actuator_Spring *create_spring(IVP_Environment *env, IVP_Template_Spring *templ);
    static IVP_Actuator_Suspension *create_suspension(IVP_Environment *env, IVP_Template_Suspension *templ);
    static IVP_Actuator_Stabilizer *create_stabilizer(IVP_Environment *env, IVP_Template_Stabilizer *templ);
    static IVP_Actuator_Check_Dist *create_check_dist(IVP_Environment *env, IVP_Template_Check_Dist *templ);
    static IVP_Constraint *create_constraint(IVP_Environment *env, const IVP_Template_Constraint *tmpl);
};

#endif
