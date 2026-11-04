// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	File:	       	ivp_rot_inertia_solver.cxx
 *	Description:	function collection
 ********************************************************************************/

#include <ivp_physics.hxx>
#include <ivu_float.hxx>
#if IVP_ENABLE_X87_EMULATION
#include <ivu_x87.hxx>
#endif
#include <ivu_hash.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_rot_inertia_solver.hxx>
#include <ivp_compact_ledge_solver.hxx>

/********************************************************************************
 *	Names:	       	p_find_center_given_xyz
 *	Description:	integrate the volume of a compact ledge: dz*dy/dx
 *	In:
 *		x,y,z:	permutation of the original axis: x is index of x-axis ...
 *		transform:	transformation matrix of the original points
 *	Out:
 *		center:		the center of mass x-axis
 *		mass:		the volume of the object
 *		inertia:	the inertia given that the object is centered
 ********************************************************************************/

IVP_Compact_Ledge_Mass_Center_Solver::IVP_Compact_Ledge_Mass_Center_Solver(const IVP_U_Matrix *transform_in)
{
    transform = transform_in;
    msum = 0.0f;
    mzsum = 0.0f;
    mzzsum = 0.0f;
}

IVP_Compact_Ledge_Find_Mass_Center::IVP_Compact_Ledge_Find_Mass_Center()
{
    sum_det = 0.0f;
    qsum_surface = 0.0f;
    sum_mass.set_to_zero();
}

#if IVP_ENABLE_X87_EMULATION
// Integrate the quadratic edge coefficients while preserving the separate
// binary64 endpoint powers and the register-width cubic integral.
static void inertia_edge_integral(IVP_FLOAT p0_x, IVP_FLOAT p0_y, IVP_FLOAT p0_z,
    IVP_FLOAT p1_x, IVP_DOUBLE vector_x, IVP_DOUBLE vector_y, IVP_DOUBLE vector_z,
    IVP_BOOL triangle_area, IVP_DOUBLE fzdy, IVP_DOUBLE fydz, IVP_DOUBLE *area)
{
    IVP_X87_Arithmetic fp(IVP_X87_Control::current());
    const IVP_U_Float80 x0 = fp.load(p0_x), x1 = fp.load(p1_x);
    const IVP_U_Float80 slope_y = fp.load(fp.store_double(fp.divide(fp.load(vector_y), fp.load(vector_x))));
    const IVP_U_Float80 offset_y = fp.subtract(fp.load(p0_y), fp.multiply(x0, slope_y));
    const IVP_U_Float80 stored_y = fp.load(fp.store_double(offset_y));
    IVP_U_Float80 a, b, c;
    if (triangle_area)
    {
        a = fp.multiply(fp.multiply(offset_y, offset_y), fp.load(fzdy));
        b = fp.multiply(fp.multiply(stored_y, slope_y), fp.load(fzdy));
        b = fp.add(b, b);
        c = fp.multiply(fp.multiply(slope_y, slope_y), fp.load(fzdy));
    }
    else
    {
        const IVP_U_Float80 slope_z = fp.load(fp.store_double(fp.divide(fp.load(vector_z), fp.load(vector_x))));
        const IVP_U_Float80 offset_z = fp.subtract(fp.load(p0_z), fp.multiply(x0, slope_z));
        const IVP_U_Float80 stored_z = fp.load(fp.store_double(offset_z));
        a = fp.multiply(fp.add(offset_y, fp.multiply(offset_z, fp.load(fydz))), offset_z);
        b = fp.multiply(fp.multiply(stored_z, slope_z), fp.load(fydz));
        b = fp.add(fp.add(fp.add(b, b), fp.multiply(slope_z, stored_y)), fp.multiply(stored_z, slope_y));
        c = fp.multiply(fp.add(fp.multiply(slope_z, fp.load(fydz)), slope_y), slope_z);
    }
    const IVP_U_Float80 square1 = fp.multiply(x1, x1);
    const IVP_U_Float80 square0 = fp.load(fp.store_double(fp.multiply(x0, x0)));
    const IVP_U_Float80 integral2 = fp.load(fp.store_double(fp.multiply(fp.subtract(square1, square0), fp.load(0.5))));
    const IVP_U_Float80 cube1 = fp.load(fp.store_double(fp.multiply(square1, x1)));
    const IVP_U_Float80 cube0 = fp.load(fp.store_double(fp.multiply(square0, x0)));
    const IVP_U_Float80 integral3 = fp.multiply(fp.subtract(cube1, cube0), fp.load(1.0 / 3.0));
    const IVP_U_Float80 fourth1 = fp.load(fp.store_double(fp.multiply(cube1, x1)));
    const IVP_U_Float80 fourth0 = fp.load(fp.store_double(fp.multiply(cube0, x0)));
    const IVP_U_Float80 integral4 = fp.load(fp.store_double(fp.multiply(fp.subtract(fourth1, fourth0), fp.load(0.25))));
    const IVP_U_Float80 integral5 = fp.multiply(fp.subtract(fp.multiply(fourth1, x1), fp.multiply(fourth0, x0)), fp.load(0.2f));
    area[0] = fp.store_double(fp.add(fp.add(fp.add(fp.multiply(fp.subtract(x1, x0), a),
        fp.multiply(integral3, c)), fp.multiply(integral2, b)), fp.load(area[0])));
    area[1] = fp.store_double(fp.add(fp.add(fp.add(fp.multiply(integral4, c),
        fp.multiply(integral3, b)), fp.multiply(integral2, a)), fp.load(area[1])));
    area[2] = fp.store_double(fp.add(fp.add(fp.add(fp.multiply(integral5, c),
        fp.multiply(integral4, b)), fp.multiply(integral3, a)), fp.load(area[2])));
}
#endif

void IVP_Compact_Ledge_Mass_Center_Solver::integrate_triangle(const IVP_Compact_Ledge *ledge, const IVP_Compact_Triangle *triangle, int x, int y, int z)
{

    IVP_U_Point hesse2;
    const IVP_Compact_Edge *edge = triangle->get_first_edge();
    IVP_CLS.calc_hesse_vec_object_not_normized(edge, ledge, &hesse2);
    hesse2.normize();
    IVP_U_Point *hesse = &hesse2;

    IVP_DOUBLE fzdy; // Steigung der Flaeche in z nach dy
    IVP_DOUBLE fydz;
    IVP_BOOL three_edge_area; // flag: integrate triangle or square

    if (IVP_DOUBLE(IVP_Inline_Math::fabsd(hesse->k[z])) < IVP_DOUBLE(IVP_Inline_Math::fabsd(hesse->k[y])))
    {
        three_edge_area = IVP_FALSE; // intagrate triangle plus square
        fydz = 0.5f * hesse->k[z] / hesse->k[y];
        fzdy = 0.0f;
    }
    else
    {
        three_edge_area = IVP_TRUE; // Integrate triangle only
        if (IVP_Inline_Math::fabsd(hesse->k[z]) > P_DOUBLE_RES)
        {
            fzdy = -0.5f * hesse->k[y] / hesse->k[z]; // hesse form is vertical to area !!
        }
        else
        {
            fzdy = 0.0f;
        }
        fydz = 0.0f;
    }
    int l;
    IVP_DOUBLE area[3]={0.0,0.0,0.0};
    for (l = 0; l < 3; l++)
    {
        IVP_U_Float_Point p0, p1;
        { // get the point coords
            const IVP_Compact_Edge *e = triangle->get_edge(l);
            const IVP_Compact_Edge *en = e->get_next();
            const IVP_U_Float_Point *p0_object = IVP_CLS.give_object_coords(e, ledge);
            const IVP_U_Float_Point *p1_object = IVP_CLS.give_object_coords(en, ledge);
            transform->vimult4(p0_object, &p0); // start point
            transform->vimult4(p1_object, &p1); // start point
        }

        IVP_U_Point vec;
        vec.subtract(&p1, &p0);
        if (IVP_Inline_Math::fabsd(vec.k[x]) < P_DOUBLE_RES * vec.real_length())
        {
            continue;
        }
#if IVP_ENABLE_X87_EMULATION
        inertia_edge_integral(p0.k[x],p0.k[y],p0.k[z],p1.k[x],
                              vec.k[x],vec.k[y],vec.k[z],three_edge_area,fzdy,fydz,area);
#else
        IVP_DOUBLE lydx = vec.k[y] / vec.k[x];	   // Steigung der Linie in y nach dx
        IVP_DOUBLE ly0 = p0.k[y] - lydx * p0.k[x]; // intersection point of line with x-axis

        IVP_DOUBLE ka, kb, kc;
        if (three_edge_area)
        {
            // loese Integrale im Bereich [x0 x1]: (y0 + x lydx)(y0 + x lydx) fzdy [x][x] dx
            ka = ly0 * ly0 * fzdy;
            kb = ly0 * lydx * fzdy + ly0 * lydx * fzdy;
            kc = lydx * lydx * fzdy;
        }
        else
        {
            // loese Integral (x*y)dx = (y0 + x lydx)(lz0 + x lzdx)dx
            //					+ fydz(z0 + x lzdx)(lz0 + x lzdx)
            IVP_DOUBLE lzdx = vec.k[z] / vec.k[x];
            IVP_DOUBLE lz0 = p0.k[z] - lzdx * p0.k[x];
            ka = (ly0 + fydz * lz0) * lz0;
            kb = lz0 * lzdx * fydz + lz0 * lzdx * fydz + lzdx * ly0 + lz0 * lydx;
            kc = (lzdx * fydz + lydx) * lzdx;
        }
        IVP_DOUBLE x0 = p0.k[x];
        IVP_DOUBLE x1 = p1.k[x];
        IVP_DOUBLE dx = x1 - x0;
        IVP_DOUBLE dx2 = (x1 * x1 - x0 * x0) * .5f;
        // RVA 0x3B550 uses a double 1/3, not the rounded float constant.
        IVP_DOUBLE dx3 = (x1 * x1 * x1 - x0 * x0 * x0) * (1.0 / 3.0);
        IVP_DOUBLE dx4 = (x1 * x1 * x1 * x1 - x0 * x0 * x0 * x0) * .25f;
        IVP_DOUBLE dx5 = (x1 * x1 * x1 * x1 * x1 - x0 * x0 * x0 * x0 * x0) * .2f;
        IVP_DOUBLE f = dx * ka + dx3 * kc + dx2 * kb;  // Flaecheninhalt
        IVP_DOUBLE k = dx4 * kc + dx3 * kb + dx2 * ka; // Kippmoment
        IVP_DOUBLE m = dx5 * kc + dx4 * kb + dx3 * ka; // Drehmomentum
        area[0] += f;
        area[1] += k;
        area[2] += m;
#endif
    }
    msum += area[0];
    mzsum += area[1];
    mzzsum += area[2];
}

void IVP_Compact_Ledge_Find_Mass_Center::integrate_triangle(const IVP_Compact_Ledge *ledge, const IVP_Compact_Triangle *triangle)
{
    const IVP_Compact_Edge *edge = triangle->get_first_edge();
    const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(edge, ledge);
    const IVP_U_Float_Point *p1 = IVP_CLS.give_object_coords(edge->get_next(), ledge);
    const IVP_U_Float_Point *p2 = IVP_CLS.give_object_coords(edge->get_prev(), ledge);

#if IVP_ENABLE_X87_EMULATION
    IVP_X87_Arithmetic fp(IVP_X87_Control::current());
    IVP_U_Float80 a[3], b[3];
    for (int i = 0; i < 3; ++i)
    {
        a[i] = fp.load(fp.store_double(fp.subtract(fp.load(p2->k[i]), fp.load(p0->k[i]))));
        b[i] = fp.load(fp.store_double(fp.subtract(fp.load(p1->k[i]), fp.load(p0->k[i]))));
    }
    const IVP_U_Float80 cross_x = fp.load(fp.store_float(fp.subtract(fp.multiply(b[1], a[2]), fp.multiply(b[2], a[1]))));
    const IVP_U_Float80 cross_y = fp.load(fp.store_float(fp.subtract(fp.multiply(b[2], a[0]), fp.multiply(b[0], a[2]))));
    const IVP_U_Float80 cross_z = fp.subtract(fp.multiply(b[0], a[1]), fp.multiply(b[1], a[0]));
    const IVP_U_Float80 determinant = fp.add(fp.add(fp.multiply(cross_x, fp.load(p0->k[0])),
        fp.multiply(cross_z, fp.load(p0->k[2]))), fp.multiply(cross_y, fp.load(p0->k[1])));
    const IVP_U_Float80 square = fp.add(fp.add(fp.multiply(cross_z, cross_z),
        fp.multiply(cross_y, cross_y)), fp.multiply(cross_x, cross_x));
    qsum_surface = fp.store_double(fp.add(square, fp.load(qsum_surface)));
    sum_det = fp.store_double(fp.add(determinant, fp.load(sum_det)));
    const IVP_U_Float80 mass_factor = fp.multiply(determinant, fp.load(0.25));
    const IVP_U_Float_Point *vertices[3] = { p0, p1, p2 };
    IVP_DOUBLE previous_y = 0.0;
    for (int vertex = 0; vertex < 3; ++vertex)
    {
        IVP_U_Float80 values[3];
        for (int axis = 0; axis < 3; ++axis)
        {
            const IVP_U_Float80 previous = axis == 1 && vertex == 1 ? fp.load(previous_y) : fp.load(sum_mass.k[axis]);
            values[axis] = fp.add(fp.multiply(fp.load(vertices[vertex]->k[axis]), mass_factor), previous);
            if (axis != 0) values[axis] = fp.load(fp.store_double(values[axis]));
        }
        previous_y = fp.store_double(values[1]);
        for (int axis = 0; axis < 3; ++axis)
            sum_mass.k[axis] = fp.store_float(values[axis]);
    }
#else
    IVP_U_Float_Point surface_normal;
    surface_normal.inline_set_vert_to_area_defined_by_three_points(p0, p2, p1);
    IVP_DOUBLE det = surface_normal.dot_product(p0);

    qsum_surface += surface_normal.quad_length();
    sum_det += det;
    sum_mass.add_multiple(p0, det * 0.25f);
    sum_mass.add_multiple(p1, det * 0.25f);
    sum_mass.add_multiple(p2, det * 0.25f);
#endif
}

void IVP_Compact_Ledge_Find_Mass_Center::integrate_ledge(const IVP_Compact_Ledge *ledge)
{
    const IVP_Compact_Triangle *tri;
    int t;
    for (t = 0, tri = ledge->get_first_triangle();
         t < ledge->get_n_triangles();
         t++, tri = tri->get_next_tri())
    {
        integrate_triangle(ledge, tri);
    }
}

void IVP_Compact_Ledge_Find_Mass_Center::integrate_ledges(IVP_U_BigVector<IVP_Compact_Ledge> *v_ledges)
{
    for (int l = v_ledges->len() - 1; l >= 0; l--)
    {
        const IVP_Compact_Ledge *ledge = v_ledges->element_at(l);
        integrate_ledge(ledge);
    }
}

void IVP_Rot_Inertia_Solver::find_center_given_xyz(IVP_U_BigVector<IVP_Compact_Ledge> *v_ledges,
                                                   int x, int y, int z, const IVP_U_Matrix *transform,
                                                   IVP_DOUBLE *center, IVP_DOUBLE *mass, IVP_DOUBLE *inertia)
{
    /* Search Center value, relativ x-Axis */
    IVP_Compact_Ledge_Mass_Center_Solver solver(transform);
    for (int l = v_ledges->len() - 1; l >= 0; l--)
    {
        const IVP_Compact_Ledge *ledge = v_ledges->element_at(l);
        const IVP_Compact_Triangle *tri;
        int t;
        for (t = 0, tri = ledge->get_first_triangle();
             t < ledge->get_n_triangles();
             t++, tri = tri->get_next_tri())
        {

            solver.integrate_triangle(ledge, tri, x, y, z);
        }
    }
    if (solver.msum < P_DOUBLE_EPS)
    {
        *mass = 0.0f;
        *center = 0.0f;
        *inertia = 1.0f;
    }
    else
    {
        *center = solver.mzsum / solver.msum;
        *mass = solver.msum;
        *inertia = solver.mzzsum / solver.msum;
    }
}

static IVP_RETURN_TYPE localCalcMassCenterAndRotInertia(
    IVP_U_BigVector<IVP_Compact_Ledge> *all_ledges,
    IVP_U_Point *mass_center_out,
    IVP_U_Point *rotation_inertia_out)
{
    IVP_U_Matrix transform;
    transform.init();

    IVP_Compact_Ledge_Find_Mass_Center fmc;
    fmc.integrate_ledges(all_ledges);

    // do not check the surface if height of object is very smally
    IVP_DOUBLE surface = IVP_Inline_Math::sqrtd(fmc.qsum_surface);
    if (fmc.sum_det > surface * surface * surface * 1e-9f)
    {
        transform.init();
        transform.vv = fmc.get_mass_center();
        mass_center_out->set(transform.get_position());

        IVP_DOUBLE volume;
        IVP_U_Point rot_inertia_2d;
        IVP_U_Point check_center;
        IVP_Rot_Inertia_Solver::find_center_given_xyz(all_ledges, 0, 1, 2, &transform, &check_center.k[0], &volume, &rot_inertia_2d.k[0]);
        IVP_Rot_Inertia_Solver::find_center_given_xyz(all_ledges, 1, 2, 0, &transform, &check_center.k[1], &volume, &rot_inertia_2d.k[1]);
        IVP_Rot_Inertia_Solver::find_center_given_xyz(all_ledges, 2, 0, 1, &transform, &check_center.k[2], &volume, &rot_inertia_2d.k[2]);

        { // calc 3d rotation inertia based on 2d values
            IVP_DOUBLE a = rot_inertia_2d.k[0];
            IVP_DOUBLE b = rot_inertia_2d.k[1];
            IVP_DOUBLE c = rot_inertia_2d.k[2];
            a *= a;
            b *= b;
            c *= c;
            IVP_DOUBLE sa = IVP_Fast_Math::sqrt(b + c);
            IVP_DOUBLE sb = IVP_Fast_Math::sqrt(a + c);
            IVP_DOUBLE sc = IVP_Fast_Math::sqrt(a + b);
            rotation_inertia_out->set(sa, sb, sc);
        }
        return IVP_OK;
    }
    else
    { // no valid masses found, take geom center
        IVP_DOUBLE radius;
        IVP_U_Point min_extents, max_extents;

        IVP_CLS.calc_bounding_box(all_ledges->element_at(0), &min_extents, &max_extents);
        for (int l = all_ledges->len() - 1; l > 0; l--)
        {
            IVP_U_Point mi, ma;
            const IVP_Compact_Ledge *ledge = all_ledges->element_at(l);
            IVP_CLS.calc_bounding_box(ledge, &mi, &ma);
            min_extents.line_min(&mi);
            max_extents.line_max(&ma);
        }
        mass_center_out->set_interpolate(&min_extents, &max_extents, 0.5f);
        radius = 0.5f * IVP_Inline_Math::sqrtd(min_extents.quad_distance_to(&max_extents));
        IVP_DOUBLE estimate_inertia = radius * radius * 0.5f;
        rotation_inertia_out->set(estimate_inertia, estimate_inertia, estimate_inertia);
        return IVP_FAULT;
    }
}

IVP_RETURN_TYPE IVP_Rot_Inertia_Solver::calc_mass_center_and_rotation_inertia(
    const IVP_Compact_Surface *c_surface_in,
    IVP_U_Point *mass_center_out,
    IVP_U_Point *rotation_inertia_out)
{
    IVP_U_BigVector<IVP_Compact_Ledge> all_ledges(128);
    IVP_Compact_Ledge_Solver::get_all_ledges(c_surface_in, &all_ledges);

    return localCalcMassCenterAndRotInertia(&all_ledges, mass_center_out, rotation_inertia_out);
}

#ifdef HAVOK_MOPP
IVP_RETURN_TYPE IVP_Rot_Inertia_Solver::calc_mass_center_and_rotation_inertia(
    const IVP_Compact_Mopp *c_mopp_in,
    IVP_U_Point *mass_center_out,
    IVP_U_Point *rotation_inertia_out)
{
    IVP_U_BigVector<IVP_Compact_Ledge> all_ledges(128);
    IVP_Compact_Ledge_Solver::get_all_ledges(c_mopp_in, &all_ledges);

    return localCalcMassCenterAndRotInertia(&all_ledges, mass_center_out, rotation_inertia_out);
}
#endif // HAVOK_MOPP