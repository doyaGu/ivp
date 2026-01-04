// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_physics.hxx
 *	Description:	The main include file for the physics API.
 *	Attention:	This file should be included prior to other 'ivp*.hxx' files.
 ********************************************************************************/

#ifndef IVP_PHYSICS_INCLUDED
#define IVP_PHYSICS_INCLUDED

#define IVP_MAX_DELTA_PSI_TIME (1.0f / 10.0f)
#define IVP_MIN_DELTA_PSI_TIME (1.0f / 200.0f)

#define IVP_BLOCKING_EVERY_MIN

#include <stdio.h>

#include <math.h>
#if defined(LINUX)
#include <string.h>
#endif

#include <ivu_types.hxx>
#include <ivu_vector.hxx>
#include <ivu_bigvector.hxx>
#include <ivu_linear.hxx>
#include <ivu_linear_macros.hxx>
#include <ivp_surface_manager.hxx>

#include <ivp_object.hxx>
#include <ivp_real_object.hxx>
#include <ivp_ball.hxx>
#include <ivp_polygon.hxx>
#include <ivp_environment.hxx>
#include <ivp_core.hxx>
#include <ivu_string.hxx>

#define IVP_NO_MD_INTERPOLATION

#endif // IVP_PHYSICS_INCLUDED
