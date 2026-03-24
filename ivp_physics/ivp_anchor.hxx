// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.
// Extracted from ivp_actuator.hxx to break ivp_physics → ivp_controller cycle.

#ifndef IVP_ANCHOR_INCLUDED
#define IVP_ANCHOR_INCLUDED

#include <ivu_linear.hxx>

class IVP_Real_Object;
class IVP_Actuator;
class IVP_Actuator_Two_Point;
class IVP_Actuator_Four_Point;
class IVP_Template_Anchor;

/********************************************************************************
 *	Name:	      	IVP_Anchor
 *	Description:	An anchor is a position in an objects space
 *			It's used by actuators, which can share one anchor.
 ********************************************************************************/
class IVP_Anchor
{
    friend class IVP_Real_Object;
    friend class IVP_Actuator_Two_Point;
    friend class IVP_Actuator_Four_Point;
    ~IVP_Anchor();

protected:
    IVP_Anchor *anchor_next_in_object; // linked list of anchors per object
    IVP_Anchor *anchor_prev_in_object;

public:
    IVP_Real_Object *l_anchor_object; // might be zero
    IVP_U_Float_Point object_pos;     // original position of object space
    IVP_U_Float_Point core_pos;       // core position of anchor
    class IVP_Actuator *l_actuator;

public:
    IVP_Anchor() = default;
    void object_is_going_to_be_deleted_event(IVP_Real_Object *obj);

    void init_anchor(IVP_Actuator *, IVP_Template_Anchor *);
    IVP_Anchor *move_anchor(IVP_U_Point *coords_ws);

    IVP_Anchor *get_next_anchor() { return anchor_next_in_object; };
    IVP_Anchor *get_prev_anchor() { return anchor_prev_in_object; };

    IVP_Real_Object *anchor_get_real_object() { return l_anchor_object; };
};

#endif // IVP_ANCHOR_INCLUDED
