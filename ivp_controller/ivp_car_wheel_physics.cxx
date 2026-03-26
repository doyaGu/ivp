// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.
// Car-wheel-specific physics code extracted from ivp_friction.cxx and
// ivp_calc_next_psi_solver.cxx to break ivp_physics → ivp_controller cycle.

#include <ivp_physics.hxx>
#include <ivp_constraint_car.hxx>
#include <ivp_friction.hxx>
#include <ivp_solver_core_reaction.hxx>
#include <ivp_calc_next_psi_solver.hxx>

bool IVP_Contact_Point::friction_force_local_constraint_2d_wheel(IVP_Core *core_a, IVP_Impact_Solver_Long_Term *info,
																 const IVP_Event_Sim *es, IVP_FLOAT &flEnergy)
{
	IVP_Constraint_Car_Object *wheel = core_a->car_wheel;
	IVP_Constraint_Solver_Car *solver = wheel->solver_car;

	IVP_DOUBLE maximum_impulse_force = this->now_friction_pressure * this->real_friction_factor * es->delta_time;

	IVP_U_Float_Point axis_bs;
	wheel->target_position_bs.get_col(IVP_COORDINATE_INDEX(solver->x_idx), &axis_bs);
	IVP_U_Float_Point axis_ws;
	const IVP_U_Matrix *m_world_f_core = solver->body_object->get_core()->get_m_world_f_core_PSI();
	m_world_f_core->vmult3(&axis_bs, &axis_ws);

	// search new span system for which one axis.dot(axis_ws) == 0
	IVP_U_Float_Point span_v_0; // span which is orthogonal to any car body influences
	IVP_U_Float_Point span_v_1; // span which is influences by car body weight

	span_v_0.calc_cross_product(&axis_ws, &info->surf_normal);
	if (span_v_0.quad_length() < 0.001f)
		return false;

	span_v_0.normize();
	span_v_1.calc_cross_product(&span_v_0, &info->surf_normal);

	IVP_DOUBLE dot_old0_new0 = info->span_friction_v[0].dot_product(&span_v_0);
	IVP_DOUBLE dot_old0_new1 = info->span_friction_v[0].dot_product(&span_v_1);
	IVP_DOUBLE dot_old1_new0 = info->span_friction_v[1].dot_product(&span_v_0);
	IVP_DOUBLE dot_old1_new1 = info->span_friction_v[1].dot_product(&span_v_1);

	IVP_DOUBLE span_s_0 = span_friction_s[0] * dot_old0_new0 + span_friction_s[1] * dot_old1_new0;
	IVP_DOUBLE span_s_1 = span_friction_s[0] * dot_old0_new1 + span_friction_s[1] * dot_old1_new1;

	// calculate pushing behaviour between wheel and car in wheel axis direction
	IVP_DOUBLE p_wheel;
	IVP_DOUBLE p_body;

	IVP_DOUBLE wheel_vel;
	IVP_DOUBLE body_vel;

	IVP_Solver_Core_Reaction tcb;
	tcb.init_reaction_solver_translation_ws(core_a, NULL, info->contact_point_ws, &span_v_0, &span_v_1, NULL);
	{
		p_wheel = tcb.m_velocity_ds_f_impulse_ds.get_elem(1, 1);
		wheel_vel = tcb.delta_velocity_ds.k[1];
	}

	{
		IVP_Solver_Core_Reaction sc_body;
		sc_body.init_reaction_solver_translation_ws(solver->body_object->get_core(), NULL, info->contact_point_ws, &span_v_1, NULL, NULL);
		p_body = sc_body.m_velocity_ds_f_impulse_ds.get_elem(0, 0);
		body_vel = sc_body.delta_velocity_ds.k[0];
	}

	IVP_DOUBLE a = span_s_0 * es->i_delta_time - tcb.delta_velocity_ds.k[0];
	// IVP_DOUBLE b = span_s_1 * es->i_delta_time - tcb.delta_velocity_ds.k[1];
	IVP_DOUBLE b = span_s_1 * es->i_delta_time - 1.0f * (wheel_vel * 1.0f + body_vel * 0.0f);

	IVP_U_Matrix3 &tpm = tcb.m_velocity_ds_f_impulse_ds;

	IVP_DOUBLE inv_mat2x2[4];

	const IVP_DOUBLE tpm00 = tpm.get_elem(0, 0);
	const IVP_DOUBLE tpm01 = tpm.get_elem(0, 1);
	const IVP_DOUBLE tpm10 = tpm.get_elem(1, 0);
	const IVP_DOUBLE tpm11 = tpm.get_elem(1, 1);

	// dimhotepus: tpm01 -> tpm10
	IVP_RETURN_TYPE ret = IVP_Inline_Math::invert_2x2_matrix(tpm00, tpm01, tpm10, tpm11, &inv_mat2x2[0], &inv_mat2x2[1], &inv_mat2x2[2], &inv_mat2x2[3]);
	if (ret != IVP_OK)
	{
		flEnergy = 0.0f;
		return false; // fall back to generic 2D friction solver
	}

	IVP_U_Float_Point impulses;
	impulses.k[0] = inv_mat2x2[0] * a + inv_mat2x2[1] * b;
	impulses.k[1] = inv_mat2x2[2] * a + inv_mat2x2[3] * b;

	IVP_DOUBLE project_span_v1 = IVP_Inline_Math::fabsd(span_v_1.dot_product(&axis_ws));

	//
	IVP_DOUBLE relaxation_coefficient = 0.3f; // ~1.0f tends to oscillate
	if (IVP_Inline_Math::fabsd(p_body) < P_FLOAT_RES)
	{
		flEnergy = 0.0f;
		return false; // avoid unstable wheel-body scaling, use generic solver instead
	}
	IVP_DOUBLE body_impulse_factor = (project_span_v1 * p_wheel / p_body + 1.0f) * relaxation_coefficient;

	IVP_DOUBLE imp0_sqrd = impulses.k[0] * impulses.k[0];
	IVP_DOUBLE imp1_sqrd = impulses.k[1] * impulses.k[1];

	// check for sliding with body push
	core_a->car_wheel->last_contact_position_ws.set(&info->contact_point_ws);
	core_a->car_wheel->last_skid_time = es->environment->get_current_time();

	// handbrake !!!
	IVP_DOUBLE square_impulse;
	if (core_a->car_wheel->fix_wheel_constraint)
	{
		square_impulse = (imp0_sqrd + imp1_sqrd) * body_impulse_factor * body_impulse_factor;

		if (square_impulse > maximum_impulse_force * maximum_impulse_force) // test for sliding
		{
			// sliding handbrake
			IVP_DOUBLE isum_impulse = IVP_Inline_Math::isqrt_float(square_impulse); // body push
			IVP_DOUBLE f = maximum_impulse_force * isum_impulse * body_impulse_factor;
			impulses.k[0] *= f;
			impulses.k[1] *= f;

			f /= body_impulse_factor;

			span_friction_s[0] *= f; // update reference point
			span_friction_s[1] *= f;

			core_a->car_wheel->last_skid_value = (1.0f - f) * this->now_friction_pressure * solver->body_object->get_core()->get_inv_mass();
			// ivp_message( "handbrake factor %f\n", f);
		}
		else
		{
			// still handbrake
			IVP_DOUBLE impulse_factor = .5f; // reduce real values to avoid jitter effects
			impulses.k[0] *= body_impulse_factor * impulse_factor;
			impulses.k[1] *= body_impulse_factor * impulse_factor;
			core_a->car_wheel->last_skid_value = 0.0f;
		}
	}
	else
	{
		square_impulse = imp0_sqrd + imp1_sqrd * body_impulse_factor * body_impulse_factor;
		if (1 && (square_impulse > maximum_impulse_force * maximum_impulse_force))
		{
			// check for sliding with no body push
			IVP_DOUBLE square2 = imp0_sqrd + imp1_sqrd;
			if (1 && (square2 > maximum_impulse_force * maximum_impulse_force))
			{
				// clip impulse on a circle
				IVP_DOUBLE isum_impulse = IVP_Inline_Math::isqrt_float(square2); // no body push
				IVP_DOUBLE f = maximum_impulse_force * isum_impulse;
				impulses.k[0] *= f;
				impulses.k[1] *= f;

				span_friction_s[0] *= f; // update reference point
				span_friction_s[1] *= f;
				core_a->car_wheel->last_skid_value = (1.0f - f) * this->now_friction_pressure * solver->body_object->get_core()->get_inv_mass();
				// ivp_message(" sliding %f factor %f\n", impulses.k[1], f );
			}
			else
			{
				// reduce body push until no sliding (clipping impulse on a elipse)
				IVP_DOUBLE y = IVP_Inline_Math::sqrtd(maximum_impulse_force * maximum_impulse_force - imp0_sqrd);
				IVP_DOUBLE old1 = impulses.k[1];
				if (impulses.k[1] > 0)
				{
					impulses.k[1] = y;
				}
				else
				{
					impulses.k[1] = -y;
				}

				if (1) // update reference point
				{
					// span_s_1 *= 0.95f;
					static IVP_DOUBLE extra_factor = .9f;
					IVP_DOUBLE s_factor = extra_factor * impulses.k[1] / (old1 * body_impulse_factor);
					core_a->car_wheel->last_skid_value = 0.3f * (1.0f - s_factor) * this->now_friction_pressure * solver->body_object->get_core()->get_inv_mass();
					if (s_factor < 1.0f)
					{
						span_s_1 *= s_factor;
						IVP_DOUBLE s0 = span_s_0 * dot_old0_new0 + span_s_1 * dot_old0_new1;
						IVP_DOUBLE s1 = span_s_0 * dot_old1_new0 + span_s_1 * dot_old1_new1;
						span_friction_s[0] = s0;
						span_friction_s[1] = s1;
					}
					// ivp_message(" reduced body push %f  s_factor %f\n", impulses.k[1], s_factor );
				}
			}

			square_impulse = maximum_impulse_force * maximum_impulse_force;
		}
		else // no sliding
		{
			// impulses.k[1] *= body_impulse_factor;
			// ivp_message("normal %f\n", impulses.k[1] );
		}
	}

	tcb.exert_impulse_dim2(core_a, /*core_b*/ NULL, impulses);

	flEnergy = 0.0f; // current_energy
	return true;
}

#ifdef IVP_FAST_WHEELS_ENABLED
void IVP_Calc_Next_PSI_Solver::calc_fast_wheel_rotation(IVP_FLOAT delta_sim_time, IVP_U_Quat *q_core_f_core)
{
	IVP_Core *pc = core;
	int axis = pc->car_wheel->solver_car->x_idx;
	IVP_DOUBLE rot_speed_wheel_axis = pc->rot_speed.k[axis];
	IVP_U_Float_Point p(pc->rot_speed);
	p.k[axis] = 0.0f;

	IVP_U_Quat gyro_rotation;
	gyro_rotation.set_fast_multiple_with_clip(&p, delta_sim_time);

	IVP_U_Quat wheel_rotation;

	p.set_to_zero();
	IVP_DOUBLE sinus = IVP_Inline_Math::sind(rot_speed_wheel_axis * 0.5f * delta_sim_time);
	p.k[axis] = sinus;

	wheel_rotation.x = p.k[0];
	wheel_rotation.y = p.k[1];
	wheel_rotation.z = p.k[2];

	wheel_rotation.w = IVP_Inline_Math::sqrtd(1.0f - sinus * sinus);

	q_core_f_core->inline_set_mult_quat(&gyro_rotation, &wheel_rotation);
}
#endif
