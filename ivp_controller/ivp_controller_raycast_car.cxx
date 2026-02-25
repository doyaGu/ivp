// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PROTECTED

/********************************************************************************
 *	Name:	     	IVP_Car_System
 *	Description:
 ********************************************************************************/
#include <ivp_physics.hxx>
#ifndef WIN32
#pragma implementation "ivp_controller_raycast_car.hxx"
#endif

// includes for API

#include <ivp_material.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_actuator.hxx>
#include <ivp_constraint_car.hxx>
#include <ivp_car_system.hxx>
#include <ivp_ray_solver.hxx>
#include <ivp_controller_raycast_car.hxx>
#include <ivp_solver_core_reaction.hxx>

int IVP_Controller_Raycast_Car::GetAxisForWheelIndex(int iWheel) const
{
	if (n_wheels <= 0 || n_axis <= 0)
	{
		return 0;
	}

	int iAxis = (iWheel * n_axis) / n_wheels;
	if (iAxis < 0)
	{
		iAxis = 0;
	}
	else if (iAxis >= n_axis)
	{
		iAxis = n_axis - 1;
	}
	return iAxis;
}

void IVP_Controller_Raycast_Car::BuildAxisWheelBounds(int *pFirstWheelPerAxis, int *pLastWheelPerAxis) const
{
	IVP_ASSERT(pFirstWheelPerAxis);
	IVP_ASSERT(pLastWheelPerAxis);
	if (!pFirstWheelPerAxis || !pLastWheelPerAxis)
	{
		return;
	}

	IVP_ASSERT(n_axis >= 0);
	IVP_ASSERT(n_axis <= IVP_CAR_SYSTEM_MAX_AXIS);

	for (int iAxis = 0; iAxis < n_axis; ++iAxis)
	{
		pFirstWheelPerAxis[iAxis] = -1;
		pLastWheelPerAxis[iAxis] = -1;
	}

	for (int iWheel = 0; iWheel < n_wheels; ++iWheel)
	{
		const int iAxis = GetAxisForWheelIndex(iWheel);
		if (pFirstWheelPerAxis[iAxis] < 0)
		{
			pFirstWheelPerAxis[iAxis] = iWheel;
		}
		pLastWheelPerAxis[iAxis] = iWheel;
	}
}

IVP_BOOL IVP_Controller_Raycast_Car::GetFrontAndRearAxes(int *pFrontAxisOut, int *pRearAxisOut,
														 int *pFirstWheelPerAxis, int *pLastWheelPerAxis) const
{
	IVP_ASSERT(pFrontAxisOut);
	IVP_ASSERT(pRearAxisOut);
	IVP_ASSERT(pFirstWheelPerAxis);
	IVP_ASSERT(pLastWheelPerAxis);
	if (!pFrontAxisOut || !pRearAxisOut || !pFirstWheelPerAxis || !pLastWheelPerAxis)
	{
		return IVP_FALSE;
	}

	*pFrontAxisOut = -1;
	*pRearAxisOut = -1;
	BuildAxisWheelBounds(pFirstWheelPerAxis, pLastWheelPerAxis);

	for (int iAxis = 0; iAxis < n_axis; ++iAxis)
	{
		if (pFirstWheelPerAxis[iAxis] >= 0)
		{
			if (*pFrontAxisOut < 0)
			{
				*pFrontAxisOut = iAxis;
			}
			*pRearAxisOut = iAxis;
		}
	}

	if (*pFrontAxisOut < 0 || *pRearAxisOut < 0)
	{
		return IVP_FALSE;
	}
	return IVP_TRUE;
}

//-----------------------------------------------------------------------------
// Purpose: Main raycast car simulation.
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::do_simulation_controller(IVP_Event_Sim *es, IVP_U_Vector<IVP_Core> *)
{
	IVP_Raycast_Car_Wheel_Temp wt[IVP_CONSTRAINT_CAR_MAX_WHEELS];
	IVP_Ray_Solver_Template rst[IVP_CONSTRAINT_CAR_MAX_WHEELS];
	IVP_Ray_Hit ray_hits[IVP_CONSTRAINT_CAR_MAX_WHEELS];
	IVP_FLOAT frictions[IVP_CONSTRAINT_CAR_MAX_WHEELS];

	const IVP_U_Matrix *m_world_f_core = car_body->get_core()->get_m_world_f_core_PSI();

	IVP_Core *car_core = car_body->get_core();

	// Raycasts.
	SetupWheelRaycasts(rst, m_world_f_core, wt);
	do_raycasts(es, n_wheels, rst, ray_hits, frictions);

	// Wheels.
	if (!DoSimulationWheels(rst, m_world_f_core, wt, ray_hits, frictions, car_core))
		return;

	// Stabilizers.
	DoSimulationStabilizers(wt);

	// Extra gravity.
	{
		car_core->speed.add_multiple(&normized_gravity_ws, extra_gravity * car_core->get_inv_mass() * es->delta_time);
	}

	// Springs.
	DoSimulationShocks(wt, ray_hits, es, car_core);

	// Booster.
	DoSimulationBooster(es, car_core);

	// Steering.
	DoSimulationSteering(wt, car_core, es);
}

IVP_CONTROLLER_PRIORITY IVP_Controller_Raycast_Car::get_controller_priority()
{
	return IVP_CP_CONSTRAINTS_MAX;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::InitRaycastCarEnvironment(IVP_Environment *pEnvironment,
									   const IVP_Template_Car_System *pCarSystemTemplate)
{
	// Copies of the car system template component indices and handedness.
	index_x = pCarSystemTemplate->index_x;
	index_y = pCarSystemTemplate->index_y;
	index_z = pCarSystemTemplate->index_z;
	is_left_handed = pCarSystemTemplate->is_left_handed;

	// Add this controller to the physics environment and setup the objects gravity.
	pEnvironment->get_controller_manager()->announce_controller_to_environment(this);
	extra_gravity = pCarSystemTemplate->extra_gravity_force_value;
	if (pEnvironment->get_gravity()->k[index_y] > 0)
	{
		gravity_y_direction = 1.0f;
	}
	else
	{
		gravity_y_direction = -1.0f;
	}
	normized_gravity_ws.set(pEnvironment->get_gravity());
	normized_gravity_ws.normize();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::InitRaycastCarBody(const IVP_Template_Car_System *pCarSystemTemplate)
{
	// Car body attributes.
	int template_wheels = pCarSystemTemplate->n_wheels;
	if (template_wheels < 0)
	{
		template_wheels = 0;
	}
	if (template_wheels > IVP_CAR_SYSTEM_MAX_WHEELS)
	{
		template_wheels = IVP_CAR_SYSTEM_MAX_WHEELS;
	}
	if (template_wheels > IVP_RAYCAST_CAR_MAX_WHEELS)
	{
		template_wheels = IVP_RAYCAST_CAR_MAX_WHEELS;
	}
	n_wheels = (short)template_wheels;

	int template_axis = pCarSystemTemplate->n_axis;
	if (template_axis < 1)
	{
		template_axis = 1;
	}
	const int max_axis_supported = (IVP_CAR_SYSTEM_MAX_AXIS < (IVP_RAYCAST_CAR_MAX_WHEELS / 2)) ? IVP_CAR_SYSTEM_MAX_AXIS : (IVP_RAYCAST_CAR_MAX_WHEELS / 2);
	if (template_axis > max_axis_supported)
	{
		template_axis = max_axis_supported;
	}
	if (template_axis > n_wheels)
	{
		template_axis = (n_wheels > 0) ? n_wheels : 1;
	}
	n_axis = (short)template_axis;
	wheels_per_axis = n_wheels / n_axis;

	// Add the car body "core" to the list of raycast car controller "cores."
	car_body = pCarSystemTemplate->car_body;
	this->vector_of_cores.add(car_body->get_core());

	// Initialize the car's booster system.
	booster_force = 0.0f;
	booster_seconds_until_ready = 0.0f;
	booster_seconds_to_go = 0.0f;

	// Init extra downward force applied to car.
	down_force_vertical_offset = pCarSystemTemplate->body_down_force_vertical_offset;
	down_force = 0.0f;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::InitRaycastCarWheels(const IVP_Template_Car_System *pCarSystemTemplate)
{
	IVP_U_Matrix m_core_f_object;
	car_body->calc_m_core_f_object(&m_core_f_object);

	// Initialize the car wheel system.
	for (int iWheel = 0; iWheel < n_wheels; iWheel++)
	{
		// Get and clear out memory for the current raycast wheel.
		IVP_Raycast_Car_Wheel *pRaycastWheel = get_wheel(IVP_POS_WHEEL(iWheel));
		P_MEM_CLEAR(pRaycastWheel);

		// Put the wheel in car space.
		m_core_f_object.vmult4(&pCarSystemTemplate->wheel_pos_Bos[iWheel], &pRaycastWheel->hp_cs);

		// Spring (Shocks) data.
		//		pRaycastWheel->distance_orig_hp_to_hp = pRaycastWheel->hp_cs.k[index_y] + ( gravity_y_direction * pCarSystemTemplate->raycast_startpoint_height_offset );
		//		pRaycastWheel->spring_len = gravity_y_direction * ( pRaycastWheel->distance_orig_hp_to_hp - pCarSystemTemplate->spring_pre_tension[iWheel] );
		pRaycastWheel->spring_len = -pCarSystemTemplate->spring_pre_tension[iWheel];
		//		pRaycastWheel->hp_cs.k[index_y] = -gravity_y_direction * pCarSystemTemplate->raycast_startpoint_height_offset;

		pRaycastWheel->spring_direction_cs.set_to_zero();
		pRaycastWheel->spring_direction_cs.k[index_y] = gravity_y_direction;

		pRaycastWheel->spring_constant = pCarSystemTemplate->spring_constant[iWheel];
		pRaycastWheel->spring_damp_relax = pCarSystemTemplate->spring_dampening[iWheel];
		pRaycastWheel->spring_damp_compress = pCarSystemTemplate->spring_dampening_compression[iWheel];

		// Wheel data.
		pRaycastWheel->friction_of_wheel = 1.0f; // pCarSystemTemplate->friction_of_wheel[iWheel];
		pRaycastWheel->wheel_radius = pCarSystemTemplate->wheel_radius[iWheel];
		if (pRaycastWheel->wheel_radius < P_FLOAT_EPS)
		{
			IVP_ASSERT(0);
			pRaycastWheel->wheel_radius = P_FLOAT_EPS;
		}
		pRaycastWheel->inv_wheel_radius = 1.0f / pRaycastWheel->wheel_radius;

		do_steering_wheel(IVP_POS_WHEEL(iWheel), 0.0f);

		pRaycastWheel->wheel_is_fixed = IVP_FALSE;
		pRaycastWheel->max_rotation_speed = pCarSystemTemplate->wheel_max_rotation_speed[iWheel >> 1];
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::InitRaycastCarAxes(const IVP_Template_Car_System *pCarSystemTemplate)
{
	this->steering_angle = -1.0f; // make sure next call is not optimized
	this->do_steering(0.0f);	  // make sure next call gets through

	for (int iAxis = 0; iAxis < n_axis; iAxis++)
	{
		IVP_Raycast_Car_Axis *pAxis = get_axis(IVP_POS_AXIS(iAxis));
		pAxis->stabilizer_constant = pCarSystemTemplate->stabilizer_constant[iAxis];
	}
}

//-----------------------------------------------------------------------------
// Purpose: Initialize the rays to be cast from the vehicle wheel positions to
//          the "ground."
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::SetupWheelRaycasts(IVP_Ray_Solver_Template *pRaySolverTemplates,
													const IVP_U_Matrix *m_world_f_core,
													IVP_Raycast_Car_Wheel_Temp *pTempWheels)
{
	for (int iWheel = 0; iWheel < n_wheels; ++iWheel)
	{
		IVP_Raycast_Car_Wheel *pWheel = get_wheel(IVP_POS_WHEEL(iWheel));
		{
			// Fill the in the ray solver template for the current wheel.
			IVP_Ray_Solver_Template &raySolverTemplate = pRaySolverTemplates[iWheel];

			// Transform the wheel "start" position from vehicle core-space to world-space.  This is
			// the raycast starting position.
			m_world_f_core->vmult4(&pWheel->hp_cs, &raySolverTemplate.ray_start_point);

			// Transform the shock (spring) direction from vehicle core-space to world-space.  This is
			// the raycast direction.
			m_world_f_core->vmult3(&pWheel->spring_direction_cs, &pTempWheels[iWheel].spring_direction_ws);
			raySolverTemplate.ray_normized_direction.set(&pTempWheels[iWheel].spring_direction_ws);

			// Set the length of the ray cast.
			raySolverTemplate.ray_length = pWheel->spring_len + pWheel->wheel_radius;

			// Set the ray solver template flags.  This defines wish objects you wish to
			// collide against in the physics environment.
			raySolverTemplate.ray_flags = IVP_RAY_SOLVER_ALL;
		}
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool IVP_Controller_Raycast_Car::DoSimulationWheels(IVP_Ray_Solver_Template *pRaySolverTemplates,
													const IVP_U_Matrix *m_world_f_core,
													IVP_Raycast_Car_Wheel_Temp *pTempWheels,
													IVP_Ray_Hit *pRayHits, IVP_FLOAT *pFrictions,
													IVP_Core *pCarCore)
{
	for (int iWheel = 0; iWheel < n_wheels; ++iWheel)
	{
		// Get "hit" data.
		IVP_Raycast_Car_Wheel *pWheel = get_wheel(IVP_POS_WHEEL(iWheel));
		IVP_Raycast_Car_Wheel_Temp *pTempWheel = &pTempWheels[iWheel];
		IVP_Ray_Hit *pRayHit = &pRayHits[iWheel];

		if (pRayHit->hit_real_object)
		{
			// Get the hit object.
			IVP_Cache_Object *pCacheObject = pRayHit->hit_real_object->get_cache_object_no_lock();
			if (pCacheObject)
			{
				// Transform the impact normal from object-space to world-space.
				pCacheObject->transform_vector_to_world_coords(&pRayHit->hit_surface_direction_os, &pTempWheel->ground_normal_ws);

				// Set the wheel's distance to move along the ray to reach the impact surface.
				pWheel->raycast_dist = pRayHit->hit_distance;
				// IVP_ASSERT( wheel->raycast_dist <= wheel->spring_len + wheel->wheel_radius);

				// Get the inverse portion of the surface normal in the direction of the ray cast (shock - used in the shock simulation code for the sign
				// and percentage of force applied to the shock).
				pTempWheel->inv_normal_dot_dir = 1.1f / (IVP_Inline_Math::fabsd(pTempWheel->spring_direction_ws.dot_product(&pTempWheel->ground_normal_ws)) + 0.1f);
			}
		}
		else
		{
			// The wheel is in air (i.e. no pressure on the wheel and it is at full extension).
			pWheel->pressure = 0.0f;
			pWheel->raycast_dist = pWheel->spring_len + pWheel->wheel_radius;

			// Set default non-impact data.
			pTempWheel->inv_normal_dot_dir = 1.0f;
			pTempWheel->moveable_object_hit_by_ray = NULL;
			pTempWheel->ground_normal_ws.set_multiple(&pTempWheel->spring_direction_ws, -1);
		}

		// Set the wheel friciton - ground friction (if any) + wheel friction.
		pTempWheel->friction_value = pFrictions[iWheel] * pWheel->friction_of_wheel;

		// Set the new wheel position (the impact point or the full ray distance).
		pTempWheel->ground_hit_ws.add_multiple(&pRaySolverTemplates[iWheel].ray_start_point, &pTempWheel->spring_direction_ws, pWheel->raycast_dist);

		// Get the speed (velocity) at the impact point.
		pCarCore->get_surface_speed_ws(&pTempWheel->ground_hit_ws, &pTempWheel->surface_speed_wheel_ws);
		pTempWheel->projected_surface_speed_wheel_ws.set_orthogonal_part(&pTempWheel->surface_speed_wheel_ws, &pTempWheel->ground_normal_ws);

		m_world_f_core->vmult3(&pWheel->axis_direction_cs, &pTempWheel->axis_direction_ws);
		pTempWheel->projected_axis_direction_ws.set_orthogonal_part(&pTempWheel->axis_direction_ws, &pTempWheel->ground_normal_ws);
		if (pTempWheel->projected_axis_direction_ws.normize() == IVP_FAULT)
		{
			IVP_IF(1)
			{
				printf("IVP_Controller_Raycast_Car::do_simulation_controller projected_axis_direction_ws.normize failed\n");
			}

			return false;
		}
	}

	return true;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::DoSimulationStabilizers(IVP_Raycast_Car_Wheel_Temp *pTempWheels)
{
	for (int iWheel = 0; iWheel < n_wheels; ++iWheel)
	{
		pTempWheels[iWheel].stabilizer_force = 0.0f;
	}

	if (n_wheels < 2 || n_axis < 1)
	{
		return;
	}

	int firstWheelPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	int lastWheelPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	BuildAxisWheelBounds(firstWheelPerAxis, lastWheelPerAxis);

	for (int iAxis = 0; iAxis < n_axis; ++iAxis)
	{
		const int iWheel0 = firstWheelPerAxis[iAxis];
		const int iWheel1 = lastWheelPerAxis[iAxis];
		if (iWheel0 < 0 || iWheel1 < 0 || iWheel0 == iWheel1)
		{
			continue;
		}

		IVP_Raycast_Car_Wheel *pWheel0 = get_wheel(IVP_POS_WHEEL(iWheel0));
		IVP_Raycast_Car_Wheel *pWheel1 = get_wheel(IVP_POS_WHEEL(iWheel1));

		const IVP_DOUBLE flDiff0 = pWheel0->raycast_dist - pWheel0->spring_len - pWheel0->wheel_radius;
		const IVP_DOUBLE flDiff1 = pWheel1->raycast_dist - pWheel1->spring_len - pWheel1->wheel_radius;

		// Raycast vehicles tend to be a bit more shaky than real wheeled vehicles; keep historical scaling.
		const IVP_FLOAT flStabilizerForce = (flDiff1 - flDiff0) * (get_axis(IVP_POS_AXIS(iAxis))->stabilizer_constant * 0.5f);
		pTempWheels[iWheel0].stabilizer_force += flStabilizerForce;
		pTempWheels[iWheel1].stabilizer_force -= flStabilizerForce;
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::DoSimulationShocks(IVP_Raycast_Car_Wheel_Temp *pTempWheels,
													IVP_Ray_Hit *pRayHits,
													IVP_Event_Sim *pEventSim,
													IVP_Core *pCarCore)
{
	// simulate springs
	for (int iWheel = 0; iWheel < n_wheels; ++iWheel)
	{
		IVP_Raycast_Car_Wheel *pWheel = get_wheel(IVP_POS_WHEEL(iWheel));

		// Check to see if we hit anything, otherwise the shocks just go to rest.
		if (!pRayHits[iWheel].hit_real_object)
			continue;

		// Check to see if we hit anything, otherwise the shocks just go to rest.
		IVP_DOUBLE flDiff = pWheel->raycast_dist - pWheel->spring_len - pWheel->wheel_radius;
		if (flDiff >= 0)
			continue;

#if 0
		// Since the front and back wheels are not the same distance from the center of mass we need to adjust the spring
		// constant accordingly.
	    const IVP_U_Matrix *m_world_f_core = car_body->get_core()->get_m_world_f_core_PSI();

		IVP_Raycast_Car_Wheel *pFrontWheel = get_wheel( IVP_POS_WHEEL( 0 ) );
		IVP_Raycast_Car_Wheel *pBackWheel = get_wheel( IVP_POS_WHEEL( 3 ) );

		IVP_U_Point frontPoint, backPoint;
		m_world_f_core->vmult4( &pFrontWheel->hp_cs, &frontPoint );
		m_world_f_core->vmult4( &pBackWheel->hp_cs, &backPoint );

		const IVP_U_Point pCenterOfMass = pCarCore->get_position_PSI();

		IVP_DOUBLE flZDeltaFront = frontPoint.k[2] - pCenterOfMass.k[2];
		IVP_DOUBLE flZDeltaBack = backPoint.k[2] - pCenterOfMass.k[2];

		IVP_DOUBLE flAlpha = fabs( flZDeltaFront / flZDeltaBack );

		IVP_FLOAT flSpringConstant, flSpringRelax, flSpringCompress;
		if ( iWheel == 0 || iWheel == 1 )
		{
			flSpringConstant = pWheel->spring_constant;
			flSpringRelax = pWheel->spring_damp_relax;
			flSpringCompress = pWheel->spring_damp_compress;
		}
		else
		{
			flSpringConstant = pWheel->spring_constant / flAlpha;
			flSpringRelax = pWheel->spring_damp_relax / flAlpha;
			flSpringCompress = pWheel->spring_damp_compress / flAlpha;
		}
#endif

		IVP_FLOAT flSpringConstant, flSpringRelax, flSpringCompress;
		flSpringConstant = pWheel->spring_constant;
		flSpringRelax = pWheel->spring_damp_relax;
		flSpringCompress = pWheel->spring_damp_compress;

		// Static force.
		IVP_DOUBLE flForce = -flDiff * flSpringConstant + pTempWheels[iWheel].stabilizer_force;
		IVP_FLOAT flInvNormalDotDir = pTempWheels[iWheel].inv_normal_dot_dir;
		if (flInvNormalDotDir < 0.0f)
		{
			flInvNormalDotDir = 0.0f;
		}
		if (flInvNormalDotDir > 3.0f)
		{
			flInvNormalDotDir = 3.0f;
		}
		flForce *= flInvNormalDotDir;

		// Dynamic force.
		IVP_U_Float_Point diff_speed;
		diff_speed.subtract(&pTempWheels[iWheel].projected_surface_speed_wheel_ws, &pTempWheels[iWheel].surface_speed_wheel_ws);

		IVP_DOUBLE flSpeed = diff_speed.dot_product(&pTempWheels[iWheel].spring_direction_ws);
		if (flSpeed > 0)
		{
			flForce -= flSpringRelax * flSpeed;
		}
		else
		{
			flForce -= flSpringCompress * flSpeed;
		}

		if (flForce < 0)
		{
			flForce = 0.0f;
		}

		pWheel->pressure = flForce;

		IVP_DOUBLE flImpulse = flForce * pEventSim->delta_time;

		IVP_U_Float_Point impulse_ws;
		impulse_ws.set_multiple(&pTempWheels[iWheel].ground_normal_ws, flImpulse);
		pCarCore->push_core_ws(&pTempWheels[iWheel].ground_hit_ws, &impulse_ws);
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::DoSimulationBooster(IVP_Event_Sim *pEventSim, IVP_Core *pCarCore)
{
	// Update the timer for next boost.
	if (booster_seconds_until_ready > 0.0f)
	{
		booster_seconds_until_ready -= pEventSim->delta_time;
	}

	// Update the timer for in boost.
	if (booster_seconds_to_go > 0.0f)
	{
		booster_seconds_to_go -= pEventSim->delta_time;

		if (booster_seconds_to_go <= 0.0f)
		{
			booster_force = 0.0f;
		}
	}

	// If we are boosting then add the boosting force to the speed.
	if (booster_force)
	{
		IVP_U_Float_Point booster_dir_ws;
		pCarCore->get_m_world_f_core_PSI()->get_col(IVP_COORDINATE_INDEX(index_z), &booster_dir_ws);
		pCarCore->speed.add_multiple(&booster_dir_ws, booster_force * pEventSim->delta_time);
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::DoSimulationSteering(IVP_Raycast_Car_Wheel_Temp *pTempWheels,
														  IVP_Core *pCarCore, IVP_Event_Sim *pEventSim)
{
	IVP_FLOAT forcesNeededToDriveStraight[IVP_CAR_SYSTEM_MAX_AXIS];

	CalcSteeringForces(pTempWheels, pCarCore, pEventSim, forcesNeededToDriveStraight);
	ApplySteeringForces(pTempWheels, pCarCore, pEventSim, forcesNeededToDriveStraight);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::CalcSteeringForces(IVP_Raycast_Car_Wheel_Temp *pTempWheels,
														IVP_Core *pCarCore, IVP_Event_Sim *pEventSim,
														IVP_FLOAT *pForcesNeededToDriveStraight)
{
	for (int iAxis = 0; iAxis < n_axis; ++iAxis)
	{
		pForcesNeededToDriveStraight[iAxis] = 0.0f;
	}

	if (n_axis < 2 || n_wheels < 2)
	{
		return;
	}

	int firstWheelPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	int lastWheelPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	int iFrontAxis = -1;
	int iRearAxis = -1;
	if (GetFrontAndRearAxes(&iFrontAxis, &iRearAxis, firstWheelPerAxis, lastWheelPerAxis) == IVP_FALSE || iFrontAxis == iRearAxis)
	{
		return;
	}

	const int iFrontWheel0 = firstWheelPerAxis[iFrontAxis];
	const int iFrontWheel1 = lastWheelPerAxis[iFrontAxis];
	const int iRearWheel0 = firstWheelPerAxis[iRearAxis];
	const int iRearWheel1 = lastWheelPerAxis[iRearAxis];
	if (iFrontWheel0 < 0 || iRearWheel0 < 0)
	{
		return;
	}

	IVP_Solver_Core_Reaction coreReactionSolver[2];
	IVP_U_Point frontPosWS;
	IVP_U_Point backPosWS;
	IVP_U_Float_Point frontAxisDirWS;
	IVP_U_Float_Point rearAxisDirWS;

	frontPosWS.set_interpolate(&pTempWheels[iFrontWheel0].ground_hit_ws, &pTempWheels[iFrontWheel1].ground_hit_ws, 0.5f);
	backPosWS.set_interpolate(&pTempWheels[iRearWheel0].ground_hit_ws, &pTempWheels[iRearWheel1].ground_hit_ws, 0.5f);

	frontAxisDirWS.add(&pTempWheels[iFrontWheel0].axis_direction_ws, &pTempWheels[iFrontWheel1].axis_direction_ws);
	if (frontAxisDirWS.normize() == IVP_FAULT)
	{
		frontAxisDirWS = pTempWheels[iFrontWheel0].axis_direction_ws;
	}

	rearAxisDirWS.add(&pTempWheels[iRearWheel0].axis_direction_ws, &pTempWheels[iRearWheel1].axis_direction_ws);
	if (rearAxisDirWS.normize() == IVP_FAULT)
	{
		rearAxisDirWS = pTempWheels[iRearWheel0].axis_direction_ws;
	}

	// Initialize the reaction solvers (for the core) front and back wheels.
	coreReactionSolver[0].init_reaction_solver_translation_ws(pCarCore, NULL, frontPosWS, &frontAxisDirWS, NULL, NULL);
	coreReactionSolver[1].init_reaction_solver_translation_ws(pCarCore, NULL, backPosWS, &rearAxisDirWS, NULL, NULL);

	// How does a push at the front/back wheel influence the back/front wheel?
	IVP_FLOAT front_back;
	front_back = coreReactionSolver[0].cr_mult_inv0[0].dot_product(&coreReactionSolver[1].cross_direction_position_cs0[0]); // rotation part
	front_back += pCarCore->get_inv_mass() * frontAxisDirWS.dot_product(&rearAxisDirWS);

	IVP_DOUBLE relaxation_rate = 1.2f;
	IVP_DOUBLE a = -coreReactionSolver[0].delta_velocity_ds.k[0] * pEventSim->i_delta_time * relaxation_rate;
	IVP_DOUBLE b = -coreReactionSolver[1].delta_velocity_ds.k[0] * pEventSim->i_delta_time * relaxation_rate;

	IVP_DOUBLE inv_mat2x2[4];
	const IVP_DOUBLE mtx2x2_00 = coreReactionSolver[0].m_velocity_ds_f_impulse_ds.get_elem(0, 0);
	const IVP_DOUBLE mtx2x2_01 = front_back;
	const IVP_DOUBLE mtx2x2_10 = front_back;
	const IVP_DOUBLE mtx2x2_11 = coreReactionSolver[1].m_velocity_ds_f_impulse_ds.get_elem(0, 0);

	// dimhotepus: mtx2x2_01 -> mtx2x2_10
	IVP_RETURN_TYPE retValue = IVP_Inline_Math::invert_2x2_matrix(mtx2x2_00, mtx2x2_01, mtx2x2_10, mtx2x2_11, &inv_mat2x2[0], &inv_mat2x2[1], &inv_mat2x2[2], &inv_mat2x2[3]);
	if (retValue == IVP_OK)
	{
		const IVP_FLOAT flFrontForce = inv_mat2x2[0] * a + inv_mat2x2[1] * b;
		const IVP_FLOAT flRearForce = inv_mat2x2[2] * a + inv_mat2x2[3] * b;

		if (iRearAxis == iFrontAxis)
		{
			pForcesNeededToDriveStraight[iFrontAxis] = flFrontForce;
			return;
		}

		const IVP_FLOAT flInvAxisSpan = 1.0f / (IVP_FLOAT)(iRearAxis - iFrontAxis);
		for (int iAxis = iFrontAxis; iAxis <= iRearAxis; ++iAxis)
		{
			if (firstWheelPerAxis[iAxis] < 0)
			{
				continue;
			}
			const IVP_FLOAT t = (iAxis - iFrontAxis) * flInvAxisSpan;
			pForcesNeededToDriveStraight[iAxis] = flFrontForce + (flRearForce - flFrontForce) * t;
		}
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::ApplySteeringForces(IVP_Raycast_Car_Wheel_Temp *pTempWheels,
														 IVP_Core *pCarCore, IVP_Event_Sim *pEventSim,
														 IVP_FLOAT *pForcesNeededToDriveStraight)
{
	if (n_wheels <= 0)
	{
		return;
	}

	IVP_FLOAT forcesNeededPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	for (int iAxis = 0; iAxis < n_axis; ++iAxis)
	{
		forcesNeededPerAxis[iAxis] = pForcesNeededToDriveStraight[iAxis];
	}

	bool bForceFixedWheel = false;
	bool bHasTorque = false;
	int iWheel;
	for (iWheel = 0; iWheel < n_wheels; ++iWheel)
	{
		IVP_Raycast_Car_Wheel *pWheel = get_wheel(IVP_POS_WHEEL(iWheel));
		if (pWheel->torque != 0.0f)
		{
			bHasTorque = true;
		}
	}

	IVP_FLOAT flSpeed = pCarCore->speed.fast_real_length();
	if (fabs(flSpeed) < 0.5f && !bHasTorque)
	{
		bForceFixedWheel = true;
	}

	for (iWheel = 0; iWheel < n_wheels; ++iWheel)
	{
		// left right
		IVP_Raycast_Car_Wheel *pWheel = get_wheel(IVP_POS_WHEEL(iWheel));
		IVP_Raycast_Car_Wheel_Temp *pTempWheel = &pTempWheels[iWheel];

		IVP_FLOAT flMaxForce = pWheel->pressure * pTempWheel->friction_value;

		IVP_U_Float_Point projected_front_dir_ws;
		projected_front_dir_ws.inline_calc_cross_product_and_normize(&pTempWheel->projected_axis_direction_ws, &pTempWheel->spring_direction_ws);

		IVP_FLOAT flFrForce = 0.0f;
		IVP_FLOAT flBodySpeed = pCarCore->speed.dot_product(&projected_front_dir_ws);

		if (pWheel->wheel_is_fixed || bForceFixedWheel)
		{
			flFrForce = -.5f * flBodySpeed * pCarCore->get_mass() * pEventSim->i_delta_time;
			pWheel->wheel_angular_velocity = 0.0f;
		}
		else
		{
			pWheel->wheel_angular_velocity = flBodySpeed * pWheel->inv_wheel_radius;

			IVP_FLOAT flSign = flBodySpeed >= 0.0f ? 1.0f : -1.0f;
			IVP_FLOAT flFrictionForce = pTempWheel->friction_value * (0.25f * pCarCore->get_mass()) * flSign;
			flFrForce = -2.5f * flFrictionForce;
			flFrForce += pWheel->torque * pWheel->inv_wheel_radius;
		}

		int nAxis = GetAxisForWheelIndex(iWheel);

		IVP_DOUBLE flForceStraight = forcesNeededPerAxis[nAxis];
		IVP_FLOAT flQuadSumForce = flFrForce * flFrForce + flForceStraight * flForceStraight;
		if (flQuadSumForce > flMaxForce * flMaxForce)
		{
			// printf("clipping of fr_force %f %f %f\n", force_straight, fr_force, max_force);

			IVP_FLOAT flFactor = IVP_Inline_Math::ivp_sqrtf(flMaxForce * flMaxForce / flQuadSumForce);
			flFrForce *= flFactor;
			flForceStraight *= flFactor;
			if (!pWheel->wheel_is_fixed)
			{
				// wheel_angular_velocity += fr_force / (max_force + P_FLOAT_EPS);
			}
		}

		forcesNeededPerAxis[nAxis] -= flForceStraight;

		pWheel->angle_wheel -= pWheel->wheel_angular_velocity * pEventSim->delta_time;

		// left right force
		IVP_U_Float_Point impulse;
		impulse.set_multiple(&pTempWheel->projected_axis_direction_ws, flForceStraight * pEventSim->delta_time);
		pCarCore->push_core_ws(&pTempWheel->ground_hit_ws, &impulse);
		// front back force
		impulse.set_multiple(&projected_front_dir_ws, flFrForce * pEventSim->delta_time);
		pCarCore->push_core_ws(&pTempWheel->ground_hit_ws, &impulse);
	}
}

void IVP_Controller_Raycast_Car::do_steering_wheel(IVP_POS_WHEEL wheel_nr, IVP_FLOAT s_angle)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(wheel_nr);

	wheel->axis_direction_cs.set_to_zero();
	wheel->axis_direction_cs.k[index_x] = 1.0f;
	wheel->axis_direction_cs.rotate(IVP_COORDINATE_INDEX(index_y), s_angle);
}

void IVP_Controller_Raycast_Car::change_spring_constant(IVP_POS_WHEEL pos, IVP_FLOAT spring_constant)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(pos);
	wheel->spring_constant = spring_constant;
}

void IVP_Controller_Raycast_Car::change_spring_dampening(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(pos);
	wheel->spring_damp_relax = spring_dampening;
}

void IVP_Controller_Raycast_Car::change_spring_dampening_compression(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(pos);
	wheel->spring_damp_compress = spring_dampening;
}

void IVP_Controller_Raycast_Car::change_spring_pre_tension(IVP_POS_WHEEL pos, IVP_FLOAT pre_tension_length)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(pos);
	wheel->spring_len = -pre_tension_length;
}

void IVP_Controller_Raycast_Car::change_spring_length(IVP_POS_WHEEL pos, IVP_FLOAT spring_length)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(pos);
	wheel->spring_len = spring_length;
}

void IVP_Controller_Raycast_Car::change_stabilizer_constant(IVP_POS_AXIS pos, IVP_FLOAT stabi_constant)
{
	IVP_Raycast_Car_Axis *axis = get_axis(pos);
	axis->stabilizer_constant = stabi_constant;
}

void IVP_Controller_Raycast_Car::change_fast_turn_factor(IVP_FLOAT fast_turn_factor_)
{
	// fast_turn_factor = fast_turn_factor_;
}

void IVP_Controller_Raycast_Car::change_wheel_torque(IVP_POS_WHEEL pos, IVP_FLOAT torque)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(pos);
	wheel->torque = torque;

	// Wake the physics object if need be!
	car_body->get_environment()->get_controller_manager()->ensure_controller_in_simulation(this);
}

void IVP_Controller_Raycast_Car::change_body_downforce(IVP_FLOAT force)
{
	down_force = force;
}

void IVP_Controller_Raycast_Car::fix_wheel(IVP_POS_WHEEL pos, IVP_BOOL stop_wheel)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(pos);
	wheel->wheel_is_fixed = stop_wheel;
}

IVP_DOUBLE IVP_Controller_Raycast_Car::get_body_speed(IVP_COORDINATE_INDEX index)
{
	// return (IVP_FLOAT)car_body->get_geom_center_speed();
	IVP_U_Float_Point *vec_ws = &car_body->get_core()->speed;
	// works well as we do not use merged cores
	const IVP_U_Matrix *mat_ws = car_body->get_core()->get_m_world_f_core_PSI();
	IVP_U_Point orientation;
	mat_ws->get_col(index, &orientation);

	return orientation.dot_product(vec_ws);
};

IVP_DOUBLE IVP_Controller_Raycast_Car::get_wheel_angular_velocity(IVP_POS_WHEEL pos)
{
	IVP_Raycast_Car_Wheel *wheel = get_wheel(pos);
	return wheel->wheel_angular_velocity;
}

IVP_DOUBLE IVP_Controller_Raycast_Car::get_orig_front_wheel_distance()
{
	if (n_wheels < 2 || n_axis < 1)
	{
		return 0.0f;
	}

	int firstWheelPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	int lastWheelPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	int iFrontAxis = -1;
	int iRearAxis = -1;
	if (GetFrontAndRearAxes(&iFrontAxis, &iRearAxis, firstWheelPerAxis, lastWheelPerAxis) == IVP_FALSE)
	{
		return 0.0f;
	}
	if (iRearAxis < iFrontAxis)
	{
		return 0.0f;
	}

	int iLeftWheel = firstWheelPerAxis[iFrontAxis];
	int iRightWheel = lastWheelPerAxis[iFrontAxis];
	if (iLeftWheel < 0 || iRightWheel < 0 || iLeftWheel == iRightWheel)
	{
		iLeftWheel = 0;
		iRightWheel = (n_wheels > 1) ? 1 : 0;
	}

	IVP_U_Float_Point *left_wheel_cs = &this->get_wheel(IVP_POS_WHEEL(iLeftWheel))->hp_cs;
	IVP_U_Float_Point *right_wheel_cs = &this->get_wheel(IVP_POS_WHEEL(iRightWheel))->hp_cs;

	IVP_DOUBLE dist = left_wheel_cs->k[this->index_x] - right_wheel_cs->k[this->index_x];

	return IVP_Inline_Math::fabsd(dist); // was fabs, which was a sml call
}

IVP_DOUBLE IVP_Controller_Raycast_Car::get_orig_axles_distance()
{
	if (n_wheels < 2 || n_axis < 2)
	{
		return 0.0f;
	}

	int firstWheelPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	int lastWheelPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
	int iFrontAxis = -1;
	int iRearAxis = -1;
	if (GetFrontAndRearAxes(&iFrontAxis, &iRearAxis, firstWheelPerAxis, lastWheelPerAxis) == IVP_FALSE || iFrontAxis == iRearAxis)
	{
		return 0.0f;
	}

	IVP_U_Float_Point *front_wheel0_cs = &this->get_wheel(IVP_POS_WHEEL(firstWheelPerAxis[iFrontAxis]))->hp_cs;
	IVP_U_Float_Point *front_wheel1_cs = &this->get_wheel(IVP_POS_WHEEL(lastWheelPerAxis[iFrontAxis]))->hp_cs;
	IVP_U_Float_Point *rear_wheel0_cs = &this->get_wheel(IVP_POS_WHEEL(firstWheelPerAxis[iRearAxis]))->hp_cs;
	IVP_U_Float_Point *rear_wheel1_cs = &this->get_wheel(IVP_POS_WHEEL(lastWheelPerAxis[iRearAxis]))->hp_cs;

	const IVP_DOUBLE front_z = 0.5f * (front_wheel0_cs->k[this->index_z] + front_wheel1_cs->k[this->index_z]);
	const IVP_DOUBLE rear_z = 0.5f * (rear_wheel0_cs->k[this->index_z] + rear_wheel1_cs->k[this->index_z]);
	IVP_DOUBLE dist = front_z - rear_z;

	return IVP_Inline_Math::fabsd(dist); // was fabs, which was a sml call
}

void IVP_Controller_Raycast_Car::get_skid_info(IVP_Wheel_Skid_Info *array_of_skid_info_out)
{
	for (int w = 0; w < n_wheels; w++)
	{
		IVP_Wheel_Skid_Info &info = array_of_skid_info_out[w];
		// IVP_Constraint_Car_Object *wheel = car_constraint_solver->wheel_objects.element_at(w);
		info.last_contact_position_ws.set_to_zero(); // = wheel->last_contact_position_ws;
		info.last_skid_value = 0.0f;				 // wheel->last_skid_value;
		info.last_skid_time = 0.0f;					 // wheel->last_skid_time;
	}
}

void IVP_Controller_Raycast_Car::set_powerslide(IVP_FLOAT front_accel, IVP_FLOAT rear_accel)
{
}

void IVP_Controller_Raycast_Car::do_steering(IVP_FLOAT steering_angle_in, bool bAnalog)
{

	// tell constraint system new steering positions of wheels
	if (steering_angle == steering_angle_in)
		return;

	this->steering_angle = steering_angle_in;

	car_body->get_environment()->get_controller_manager()->ensure_controller_in_simulation(this);

	for (int i = 0; i < n_wheels; i++)
	{
		if (GetAxisForWheelIndex(i) == 0)
		{
			this->do_steering_wheel(IVP_POS_WHEEL(i), steering_angle_in);
		}
	}
}

void IVP_Controller_Raycast_Car::set_booster_acceleration(IVP_FLOAT acceleration)
{
	booster_force = acceleration;
}

void IVP_Controller_Raycast_Car::activate_booster(IVP_FLOAT thrust, IVP_FLOAT duration, IVP_FLOAT delay)
{

	if (this->booster_force)
		return;

	// booster not ready yet
	if (this->booster_seconds_until_ready > 0.0f)
		return;

	booster_force = thrust;
	this->booster_seconds_to_go = duration;				  // seconds
	this->booster_seconds_until_ready = duration + delay; // time when next ignition is possible
}

IVP_FLOAT IVP_Controller_Raycast_Car::get_booster_delay()
{
	return booster_seconds_until_ready;
}

IVP_FLOAT IVP_Controller_Raycast_Car::get_booster_time_to_go()
{
	return booster_seconds_to_go;
}

//-----------------------------------------------------------------------------
// Purpose: (Ipion) Create a raycast car controller.
//   Input: pEnvironment - the physics environment the car is to reside in
//          pCarSystemTemplate - ipion's car system template (filled out with car parameters)
//-----------------------------------------------------------------------------
IVP_Controller_Raycast_Car::IVP_Controller_Raycast_Car(IVP_Environment *pEnvironment /*environment*/,
													   const IVP_Template_Car_System *pCarSystemTemplate /*tcs*/)
{
	InitRaycastCarBody(pCarSystemTemplate);
	InitRaycastCarEnvironment(pEnvironment, pCarSystemTemplate);
	InitRaycastCarWheels(pCarSystemTemplate);
	InitRaycastCarAxes(pCarSystemTemplate);
}

IVP_Controller_Raycast_Car::~IVP_Controller_Raycast_Car()
{
	car_body->get_environment()->get_controller_manager()->remove_controller_from_environment(this, IVP_TRUE);
}

//-----------------------------------------------------------------------------
// Purpose: Debug data for use in vphysics and the engine to visualize car data.
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::SetCarSystemDebugData(const IVP_CarSystemDebugData_t &carSystemDebugData)
{
	// Wheels (raycast data only!)
	for (int iWheel = 0; iWheel < IVP_CAR_SYSTEM_MAX_WHEELS; ++iWheel)
	{
		m_CarSystemDebugData.wheelRaycasts[iWheel][0] = carSystemDebugData.wheelRaycasts[iWheel][0];
		m_CarSystemDebugData.wheelRaycasts[iWheel][1] = carSystemDebugData.wheelRaycasts[iWheel][1];
		m_CarSystemDebugData.wheelRaycastImpacts[iWheel] = carSystemDebugData.wheelRaycastImpacts[iWheel];
	}
}

//-----------------------------------------------------------------------------
// Purpose: Debug data for use in vphysics and the engine to visualize car data.
//-----------------------------------------------------------------------------
void IVP_Controller_Raycast_Car::GetCarSystemDebugData(IVP_CarSystemDebugData_t &carSystemDebugData)
{
	// Wheels (raycast data only!)
	for (int iWheel = 0; iWheel < IVP_CAR_SYSTEM_MAX_WHEELS; ++iWheel)
	{
		carSystemDebugData.wheelRaycasts[iWheel][0] = m_CarSystemDebugData.wheelRaycasts[iWheel][0];
		carSystemDebugData.wheelRaycasts[iWheel][1] = m_CarSystemDebugData.wheelRaycasts[iWheel][1];
		carSystemDebugData.wheelRaycastImpacts[iWheel] = m_CarSystemDebugData.wheelRaycastImpacts[iWheel];
	}
}
