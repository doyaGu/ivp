// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_templates_intern.hxx>

IVP_Template_Polygon::IVP_Template_Polygon()
{
    this->n_points = this->n_surfaces = this->n_lines = 0;
    this->points = NULL;
    this->surfaces = NULL;
    this->lines = NULL;
}

IVP_Template_Polygon::IVP_Template_Polygon(int point_count,
                                           int line_count,
                                           int surface_count)
{
    this->n_points = point_count;
    this->n_lines = line_count;
    this->n_surfaces = surface_count;
    this->lines = new IVP_Template_Line[n_lines];
    this->points = new IVP_Template_Point[n_points];
    this->surfaces = new IVP_Template_Surface[n_surfaces];
    int i;
    for (i = 0; i < this->n_surfaces; i++)
    {
        this->surfaces[i].templ_poly = this;
    }
}

IVP_Template_Polygon::~IVP_Template_Polygon()
{
    P_DELETE_ARRAY(this->lines);
    P_DELETE_ARRAY(this->points);
    P_DELETE_ARRAY(this->surfaces);
}

void IVP_Template_Polygon::scale(int scaling_factor)
{
    if (this->n_points <= 0 || !this->points)
    {
        return;
    }
    IVP_DOUBLE factor = (IVP_DOUBLE)scaling_factor;
    for (int i = 0; i < this->n_points; i++)
    {
        this->points[i].mult(factor);
    }
}

IVP_Template_Surface::IVP_Template_Surface()
{
    templ_poly = NULL;
    n_lines = 0;
    lines = NULL;
    revert_line = NULL;
}

void IVP_Template_Surface::close_surface()
{
    P_FREE(this->lines);
    P_DELETE_ARRAY(this->revert_line);
    this->lines = NULL;
    this->revert_line = NULL;
    this->n_lines = 0;
}

void IVP_Template_Surface::calc_surface_normal_template(int a, int b, int c)
{
    IVP_U_Hesse my_hesse;
    my_hesse.calc_hesse(&templ_poly->points[a], &templ_poly->points[b], &templ_poly->points[c]);
    my_hesse.normize();
    this->normal.set(my_hesse.k[0], my_hesse.k[1], my_hesse.k[2]);
}

void IVP_Template_Surface::init_surface(int line_count)
{
    this->close_surface();
    if (line_count <= 0)
    {
        return;
    }

    this->lines = (ushort *)p_calloc(line_count, sizeof(ushort));
    this->revert_line = new char[line_count];
    if (!this->lines || !this->revert_line)
    {
        P_FREE(this->lines);
        P_DELETE_ARRAY(this->revert_line);
        this->lines = NULL;
        this->revert_line = NULL;
        return;
    }
    this->n_lines = line_count;
}

int IVP_Template_Surface::get_surface_index()
{
    return this - templ_poly->surfaces;
}

IVP_Template_Surface::~IVP_Template_Surface()
{
    this->close_surface();
}
