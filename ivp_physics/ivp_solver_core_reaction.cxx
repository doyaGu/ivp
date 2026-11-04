#include <ivp_physics.hxx>
#include <ivp_solver_core_reaction.hxx>
#if IVP_ENABLE_X87_EMULATION
#include <ivu_x87.hxx>
#endif

// IVP_EXPORT_PRIVATE

#if IVP_ENABLE_X87_EMULATION
static IVP_U_Float80 ivp_translation_cross(IVP_X87_Arithmetic &fp, const IVP_U_Matrix *matrix,
    const IVP_U_Float_Point &point, const IVP_U_Float_Point *direction, IVP_U_Point_4 *output)
{
    IVP_U_Float80 cross[3], projected[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        const int y = (axis + 1) % 3, z = (axis + 2) % 3;
        cross[axis] = fp.subtract(fp.multiply(fp.load(point.k[y]), fp.load(direction->k[z])),
            fp.multiply(fp.load(point.k[z]), fp.load(direction->k[y])));
    }
    cross[2] = fp.load(fp.store_float(cross[2]));
    for (int axis = 0; axis < 3; ++axis)
        projected[axis] = fp.add(fp.add(fp.multiply(cross[2], fp.load(matrix->get_elem(2, axis))),
            fp.multiply(cross[1], fp.load(matrix->get_elem(1, axis)))), fp.multiply(cross[0], fp.load(matrix->get_elem(0, axis))));
    for (int axis = 0; axis < 3; ++axis)
        output->k[axis] = fp.store_float(projected[axis]);
    return projected[2];
}
#else
// RVA 0x33560 keeps cross_ws.x/y wide until the inverse rotation; z spills
// to float. Return the last transformed component before its float store.
static IVP_DOUBLE ivp_translation_cross(const IVP_U_Matrix *m,
    const IVP_U_Float_Point &p, const IVP_U_Float_Point *d, IVP_U_Point_4 *out)
{
    const IVP_DOUBLE x=IVP_DOUBLE(p.k[1])*d->k[2]-IVP_DOUBLE(p.k[2])*d->k[1];
    const IVP_DOUBLE y=IVP_DOUBLE(p.k[2])*d->k[0]-IVP_DOUBLE(p.k[0])*d->k[2];
    const IVP_FLOAT z=IVP_DOUBLE(p.k[0])*d->k[1]-IVP_DOUBLE(p.k[1])*d->k[0];
    const IVP_DOUBLE zz=z*m->get_elem(2,2)+y*m->get_elem(1,2)+x*m->get_elem(0,2);
    out->set(z*m->get_elem(2,0)+y*m->get_elem(1,0)+x*m->get_elem(0,0),
        z*m->get_elem(2,1)+y*m->get_elem(1,1)+x*m->get_elem(0,1),zz);
    return zz;
}

#endif

void IVP_Solver_Core_Reaction::init_trans_ws(const IVP_U_Point *pos_ws, IVP_Core *core,
                                             IVP_U_Point_4 cross_direction_position_cs[3],
                                             IVP_U_Point_4 cr_mult_inv[3], IVP_FLOAT sign)
{
    const IVP_U_Matrix *m_world_f_core = core->get_m_world_f_core_PSI();
    m_world_f_core_last_psi[0] = m_world_f_core;

    IVP_U_Float_Point pos_rel_core;
    pos_rel_core.subtract(pos_ws, core->get_position_PSI());

    const IVP_U_Point_4 *im = core->get_inv_masses();

    // direction_ws[0]
    {
#if IVP_ENABLE_X87_EMULATION
        IVP_X87_Arithmetic fp(IVP_X87_Control::current());
        const IVP_U_Float80 raw_z = ivp_translation_cross(fp, m_world_f_core, pos_rel_core, direction_ws[0], &cross_direction_position_cs[0]);
        cross_direction_position_cs[0].hesse_val = 1.0f;
        cr_mult_inv[0].set_line_wise_mult4(&cross_direction_position_cs[0], im);
        const IVP_U_Float80 inertia_z = fp.multiply(raw_z, fp.load(im->k[2]));
        cr_mult_inv[0].k[2] = fp.store_float(inertia_z);
        const IVP_U_Float80 inertia_y = fp.multiply(fp.multiply(fp.load(cross_direction_position_cs[0].k[1]), fp.load(im->k[1])), fp.load(cross_direction_position_cs[0].k[1]));
        IVP_U_Float80 diagonal = fp.add(inertia_y, fp.load(im->hesse_val));
        diagonal = fp.add(diagonal, fp.multiply(inertia_z, fp.load(cross_direction_position_cs[0].k[2])));
        diagonal = fp.add(diagonal, fp.multiply(fp.load(cross_direction_position_cs[0].k[0]), fp.load(cr_mult_inv[0].k[0])));
        const IVP_DOUBLE val = fp.store_double(fp.add(diagonal, fp.load(m_velocity_ds_f_impulse_ds.get_elem(0, 0))));
#else
        const IVP_DOUBLE raw_z=ivp_translation_cross(m_world_f_core,pos_rel_core,direction_ws[0],&cross_direction_position_cs[0]);
        cross_direction_position_cs[0].hesse_val = 1.0f;

        cr_mult_inv[0].set_line_wise_mult4(&cross_direction_position_cs[0], im);
        // RVA 0x33560 multiplies the retained transformed Z by inverse
        // inertia before narrowing; reusing the stored float rounds twice.
        cr_mult_inv[0].k[2] = raw_z * im->k[2];

        IVP_DOUBLE val = IVP_DOUBLE(cross_direction_position_cs[0].k[1])*im->k[1]*cross_direction_position_cs[0].k[1]
            + im->hesse_val + raw_z*im->k[2]*cross_direction_position_cs[0].k[2]
            + IVP_DOUBLE(cross_direction_position_cs[0].k[0])*cr_mult_inv[0].k[0]
            + m_velocity_ds_f_impulse_ds.get_elem(0,0);
#endif
        m_velocity_ds_f_impulse_ds.set_elem(0, 0, val);

        IVP_DOUBLE a = core->rot_speed.dot_product(&cross_direction_position_cs[0]) + core->speed.dot_product(direction_ws[0]);
        delta_velocity_ds.k[0] += a * sign;
    }

    if (direction_ws[1])
    {
#if IVP_ENABLE_X87_EMULATION
        IVP_X87_Arithmetic fp(IVP_X87_Control::current());
        ivp_translation_cross(fp, m_world_f_core, pos_rel_core, direction_ws[1], &cross_direction_position_cs[1]);
#else
        ivp_translation_cross(m_world_f_core, pos_rel_core, direction_ws[1], &cross_direction_position_cs[1]);
#endif
        cross_direction_position_cs[1].hesse_val = 1.0f;

        cr_mult_inv[1].set_line_wise_mult4(&cross_direction_position_cs[1], im);

        IVP_DOUBLE val = m_velocity_ds_f_impulse_ds.get_elem(1, 1) + cr_mult_inv[1].dot_product4(cross_direction_position_cs[1]);
        m_velocity_ds_f_impulse_ds.set_elem(1, 1, val);

        val = m_velocity_ds_f_impulse_ds.get_elem(0, 1) + cr_mult_inv[1].dot_product(&cross_direction_position_cs[0]);
        m_velocity_ds_f_impulse_ds.set_elem(0, 1, val);

        IVP_DOUBLE a = core->rot_speed.dot_product(&cross_direction_position_cs[1]) + core->speed.dot_product(direction_ws[1]);
        delta_velocity_ds.k[1] += a * sign;
    }
    else
    {
        return;
    }

    if (direction_ws[2])
    {
        IVP_U_Float_Point cross_ws;
        cross_ws.inline_calc_cross_product(&pos_rel_core, direction_ws[2]);

        m_world_f_core->inline_vimult3(&cross_ws, &cross_direction_position_cs[2]);
        cross_direction_position_cs[2].hesse_val = 1.0f;

        cr_mult_inv[2].set_line_wise_mult4(&cross_direction_position_cs[2], im);

        IVP_DOUBLE val = m_velocity_ds_f_impulse_ds.get_elem(2, 2) + cr_mult_inv[2].dot_product4(cross_direction_position_cs[2]);
        m_velocity_ds_f_impulse_ds.set_elem(2, 2, val);

        val = m_velocity_ds_f_impulse_ds.get_elem(0, 2) + cr_mult_inv[2].dot_product(&cross_direction_position_cs[0]);
        m_velocity_ds_f_impulse_ds.set_elem(0, 2, val);

        val = m_velocity_ds_f_impulse_ds.get_elem(1, 2) + cr_mult_inv[2].dot_product(&cross_direction_position_cs[1]);
        m_velocity_ds_f_impulse_ds.set_elem(1, 2, val);

        IVP_DOUBLE a = core->rot_speed.dot_product(&cross_direction_position_cs[2]) + core->speed.dot_product(direction_ws[2]);
        delta_velocity_ds.k[2] += a * sign;
    }
}

void IVP_Solver_Core_Reaction::init_rot_ws(IVP_Core *core, IVP_U_Point_4 cross_direction_position_cs[3],
                                           IVP_U_Point_4 cr_mult_inv[3], IVP_FLOAT sign)
{
    const IVP_U_Matrix *m_world_f_core = core->get_m_world_f_core_PSI();
    m_world_f_core_last_psi[0] = m_world_f_core;

    const IVP_U_Point_4 *im = core->get_inv_masses();

    // direciton_ws[0]
    {
        m_world_f_core->inline_vimult3(direction_ws[0], &cross_direction_position_cs[0]);
        cross_direction_position_cs[0].hesse_val = 0.0f;

        cr_mult_inv[0].set_line_wise_mult4(&cross_direction_position_cs[0], im);

        IVP_DOUBLE val = m_velocity_ds_f_impulse_ds.get_elem(0, 0) + cr_mult_inv[0].dot_product(&cross_direction_position_cs[0]);
        m_velocity_ds_f_impulse_ds.set_elem(0, 0, val);

        IVP_DOUBLE a = core->rot_speed.dot_product(&cross_direction_position_cs[0]);
        delta_velocity_ds.k[0] += a * sign;
    }

    if (direction_ws[1])
    {
        m_world_f_core->inline_vimult3(direction_ws[1], &cross_direction_position_cs[1]);
        cross_direction_position_cs[1].hesse_val = 0.0f;

        cr_mult_inv[1].set_line_wise_mult4(&cross_direction_position_cs[1], im);

        IVP_DOUBLE val = m_velocity_ds_f_impulse_ds.get_elem(1, 1) + cr_mult_inv[1].dot_product(&cross_direction_position_cs[1]);
        m_velocity_ds_f_impulse_ds.set_elem(1, 1, val);

        val = m_velocity_ds_f_impulse_ds.get_elem(0, 1) + cr_mult_inv[1].dot_product(&cross_direction_position_cs[0]);
        m_velocity_ds_f_impulse_ds.set_elem(0, 1, val);

        IVP_DOUBLE a = core->rot_speed.dot_product(&cross_direction_position_cs[1]);
        delta_velocity_ds.k[1] += a * sign;
    }

    if (direction_ws[2])
    {
        m_world_f_core->vimult3(direction_ws[2], &cross_direction_position_cs[2]);
        cross_direction_position_cs[2].hesse_val = 0.0f;

        cr_mult_inv[2].set_line_wise_mult4(&cross_direction_position_cs[2], im);

        IVP_DOUBLE val = m_velocity_ds_f_impulse_ds.get_elem(2, 2) + cr_mult_inv[2].dot_product(&cross_direction_position_cs[2]);
        m_velocity_ds_f_impulse_ds.set_elem(2, 2, val);

        val = m_velocity_ds_f_impulse_ds.get_elem(0, 2) + cr_mult_inv[2].dot_product(&cross_direction_position_cs[0]);
        m_velocity_ds_f_impulse_ds.set_elem(0, 2, val);

        val = m_velocity_ds_f_impulse_ds.get_elem(1, 2) + cr_mult_inv[2].dot_product(&cross_direction_position_cs[1]);
        m_velocity_ds_f_impulse_ds.set_elem(1, 2, val);

        IVP_DOUBLE a = core->rot_speed.dot_product(&cross_direction_position_cs[2]);
        delta_velocity_ds.k[2] += a * sign;
    }
}

void IVP_Solver_Core_Reaction::init_reaction_solver_translation_ws(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Point &pos_ws,
                                                                   IVP_U_Float_Point *direction_0_ws, // the directions define the d_space (_ds)
                                                                   IVP_U_Float_Point *direction_1_ws,
                                                                   IVP_U_Float_Point *direction_2_ws)
{
    // if <3, m_velocity_ds_f_impulse_ds is not filled completely
    direction_ws[0] = direction_0_ws;
    direction_ws[1] = direction_1_ws;
    direction_ws[2] = direction_2_ws;

    if (direction_1_ws)
    {
        // clear all matrix ???
        P_MEM_CLEAR(&m_velocity_ds_f_impulse_ds);
        delta_velocity_ds.set_to_zero();
    }
    else
    {
        m_velocity_ds_f_impulse_ds.set_elem(0, 0, 0.0f); // just one element
        delta_velocity_ds.k[0] = 0;
    }

    if (core_0)
    {
        init_trans_ws(&pos_ws, core_0, cross_direction_position_cs0, cr_mult_inv0, 1.0f);
    }

    if (core_1)
    {
        init_trans_ws(&pos_ws, core_1, cross_direction_position_cs1, cr_mult_inv1, -1.0f);
    }
}

void IVP_Solver_Core_Reaction::init_reaction_solver_rotation_ws(IVP_Core *core_0, IVP_Core *core_1,
                                                                IVP_U_Float_Point *direction_0_ws, // the directions define the d_space (_ds)
                                                                IVP_U_Float_Point *direction_1_ws,
                                                                IVP_U_Float_Point *direction_2_ws)
{
    // if <3, m_velocity_ds_f_impulse_ds is not filled completely
    direction_ws[0] = direction_0_ws;
    direction_ws[1] = direction_1_ws;
    direction_ws[2] = direction_2_ws;

    P_MEM_CLEAR(&m_velocity_ds_f_impulse_ds);
    P_MEM_CLEAR(&delta_velocity_ds);

    if (core_0)
    {
        init_rot_ws(core_0, cross_direction_position_cs0, cr_mult_inv0, 1.0f);
    }

    if (core_1)
    {
        init_rot_ws(core_1, cross_direction_position_cs1, cr_mult_inv1, -1.0f);
    }
}

void IVP_Solver_Core_Reaction::exert_impulse_dim3(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds)
{
    if (core_0)
    {
        IVP_DOUBLE inv_m = core_0->get_inv_mass();
        core_0->speed.add_multiple(direction_ws[0], impulse_ds.k[0] * inv_m);
        core_0->speed.add_multiple(direction_ws[1], impulse_ds.k[1] * inv_m);
        core_0->speed.add_multiple(direction_ws[2], impulse_ds.k[2] * inv_m);
        core_0->rot_speed.add_multiple(&cr_mult_inv0[0], impulse_ds.k[0]);
        core_0->rot_speed.add_multiple(&cr_mult_inv0[1], impulse_ds.k[1]);
        core_0->rot_speed.add_multiple(&cr_mult_inv0[2], impulse_ds.k[2]);
    }

    if (core_1)
    {
        IVP_DOUBLE inv_m = core_1->get_inv_mass();
        core_1->speed.add_multiple(direction_ws[0], -impulse_ds.k[0] * inv_m);
        core_1->speed.add_multiple(direction_ws[1], -impulse_ds.k[1] * inv_m);
        core_1->speed.add_multiple(direction_ws[2], -impulse_ds.k[2] * inv_m);
        core_1->rot_speed.add_multiple(&cr_mult_inv1[0], -impulse_ds.k[0]);
        core_1->rot_speed.add_multiple(&cr_mult_inv1[1], -impulse_ds.k[1]);
        core_1->rot_speed.add_multiple(&cr_mult_inv1[2], -impulse_ds.k[2]);
    }
}

void IVP_Solver_Core_Reaction::exert_impulse_dim2(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds)
{
    if (core_0)
    {
        IVP_DOUBLE inv_m = core_0->get_inv_mass();
        core_0->speed.add_multiple(direction_ws[0], impulse_ds.k[0] * inv_m);
        core_0->speed.add_multiple(direction_ws[1], impulse_ds.k[1] * inv_m);
        core_0->rot_speed.add_multiple(&cr_mult_inv0[0], impulse_ds.k[0]);
        core_0->rot_speed.add_multiple(&cr_mult_inv0[1], impulse_ds.k[1]);
    }

    if (core_1)
    {
        IVP_DOUBLE inv_m = core_1->get_inv_mass();
        core_1->speed.add_multiple(direction_ws[0], -impulse_ds.k[0] * inv_m);
        core_1->speed.add_multiple(direction_ws[1], -impulse_ds.k[1] * inv_m);
        core_1->rot_speed.add_multiple(&cr_mult_inv1[0], -impulse_ds.k[0]);
        core_1->rot_speed.add_multiple(&cr_mult_inv1[1], -impulse_ds.k[1]);
    }
}

void IVP_Solver_Core_Reaction::exert_impulse_dim1(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds)
{
    if (core_0)
    {
        core_0->speed.add_multiple(direction_ws[0], impulse_ds.k[0] * core_0->get_inv_mass());
        core_0->rot_speed.add_multiple(&cr_mult_inv0[0], impulse_ds.k[0]);
    }

    if (core_1)
    {
        core_1->speed.add_multiple(direction_ws[0], -impulse_ds.k[0] * core_1->get_inv_mass());
        core_1->rot_speed.add_multiple(&cr_mult_inv1[0], -impulse_ds.k[0]);
    }
}

void IVP_Solver_Core_Reaction::exert_angular_impulse_dim3(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds)
{
    if (core_0)
    {
        core_0->rot_speed.add_multiple(&cr_mult_inv0[0], impulse_ds.k[0]);
        core_0->rot_speed.add_multiple(&cr_mult_inv0[1], impulse_ds.k[1]);
        core_0->rot_speed.add_multiple(&cr_mult_inv0[2], impulse_ds.k[2]);
    }

    if (core_1)
    {
        core_1->rot_speed.add_multiple(&cr_mult_inv1[0], -impulse_ds.k[0]);
        core_1->rot_speed.add_multiple(&cr_mult_inv1[1], -impulse_ds.k[1]);
        core_1->rot_speed.add_multiple(&cr_mult_inv1[2], -impulse_ds.k[2]);
    }
}

void IVP_Solver_Core_Reaction::exert_angular_impulse_dim2(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds)
{
    if (core_0)
    {
        core_0->rot_speed.add_multiple(&cr_mult_inv0[0], impulse_ds.k[0]);
        core_0->rot_speed.add_multiple(&cr_mult_inv0[1], impulse_ds.k[1]);
    }

    if (core_1)
    {
        core_1->rot_speed.add_multiple(&cr_mult_inv1[0], -impulse_ds.k[0]);
        core_1->rot_speed.add_multiple(&cr_mult_inv1[1], -impulse_ds.k[1]);
    }
}

void IVP_Solver_Core_Reaction::exert_angular_impulse_dim1(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds)
{
    if (core_0)
    {
        core_0->rot_speed.add_multiple(&cr_mult_inv0[0], impulse_ds.k[0]);
    }

    if (core_1)
    {
        core_1->rot_speed.add_multiple(&cr_mult_inv1[0], -impulse_ds.k[0]);
    }
}

IVP_RETURN_TYPE IVP_Solver_Core_Reaction::invert_3x3_matrix()
{
    m_velocity_ds_f_impulse_ds.set_elem(1, 0, m_velocity_ds_f_impulse_ds.get_elem(0, 1));
    m_velocity_ds_f_impulse_ds.set_elem(2, 0, m_velocity_ds_f_impulse_ds.get_elem(0, 2));
    m_velocity_ds_f_impulse_ds.set_elem(2, 1, m_velocity_ds_f_impulse_ds.get_elem(1, 2));
    return m_velocity_ds_f_impulse_ds.real_invert();
}
