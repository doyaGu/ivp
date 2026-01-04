// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC
#ifndef IVP_LIQUID_SURFACE_DESCRIPTOR_INCLUDED
#define IVP_LIQUID_SURFACE_DESCRIPTOR_INCLUDED

#include <ivu_linear.hxx>

class IVP_Environment;
class IVP_Core;

class IVP_Liquid_Surface_Descriptor
{
public:
    virtual void calc_liquid_surface(IVP_Environment *environment,
                                     IVP_Core *core,
                                     IVP_U_Float_Hesse *surface_normal_out,
                                     IVP_U_Float_Point *abs_speed_of_current_out) = 0;
};

class IVP_Liquid_Surface_Descriptor_Simple : public IVP_Liquid_Surface_Descriptor
{
public:
    IVP_U_Float_Hesse surface;
    IVP_U_Float_Point abs_speed_of_current;

public:
    void calc_liquid_surface(IVP_Environment *environment,
                             IVP_Core *core,
                             IVP_U_Float_Hesse *surface_normal_out,
                             IVP_U_Float_Point *abs_speed_of_current_out);

    IVP_Liquid_Surface_Descriptor_Simple(const IVP_U_Float_Hesse *surface_in,
                                         const IVP_U_Float_Point *abs_speed_of_current_in);
};

#endif // IVP_LIQUID_SURFACE_DESCRIPTOR_INCLUDED