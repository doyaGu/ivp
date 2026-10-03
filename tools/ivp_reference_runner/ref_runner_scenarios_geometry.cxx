#include "ref_runner.hxx"

#include <ivp_surbuild_ledge_soup.hxx>
#include <ivp_template_surbuild.hxx>
#include <ivp_compact_grid.hxx>
#include <ivp_gridbuild_array.hxx>
#include <ivp_surman_grid.hxx>

/* Geometry / collision coverage scenarios: concave ledge soups (static and
 * dynamic compounds, with and without root convex hull), irregular pointsoup
 * hulls and a compact grid heightfield.  Every point list, quaternion and
 * material is a literal shared with libivp's tests/test_scenarios.c. */

namespace ref_runner {

static IVP_Compact_Ledge *geo_hull_ledge(const double *c, int n) {
    IVP_U_Vector<IVP_U_Point> points(n);
    for (int i = 0; i < n; ++i) {
        IVP_U_Point *p = new IVP_U_Point();
        p->set(c[3 * i + 0], c[3 * i + 1], c[3 * i + 2]);
        points.add(p);
    }
    IVP_Compact_Ledge *l = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&points);
    for (int i = 0; i < points.len(); ++i) delete points.element_at(i);
    return l;
}

static IVP_Compact_Surface *geo_hull_surface(const double *c, int n) {
    IVP_U_Vector<IVP_U_Point> points(n);
    for (int i = 0; i < n; ++i) {
        IVP_U_Point *p = new IVP_U_Point();
        p->set(c[3 * i + 0], c[3 * i + 1], c[3 * i + 2]);
        points.add(p);
    }
    IVP_Compact_Surface *s = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface(&points);
    for (int i = 0; i < points.len(); ++i) delete points.element_at(i);
    return s;
}

/* corners in build_box_compact_surface order (sx, sy, sz = -1, +1), shifted */
static void geo_box_points(double *c, double hx, double hy, double hz, double ox, double oy, double oz) {
    int k = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                c[k++] = (double)sx * hx + ox;
                c[k++] = (double)sy * hy + oy;
                c[k++] = (double)sz * hz + oz;
            }
}

static IVP_Compact_Ledge *geo_box_ledge(double hx, double hy, double hz, double ox, double oy, double oz) {
    double c[24];
    geo_box_points(c, hx, hy, hz, ox, oy, oz);
    return geo_hull_ledge(c, 8);
}

static IVP_Compact_Surface *geo_compile_soup(IVP_Compact_Ledge **ledges, int n, IVP_BOOL root_hull) {
    IVP_SurfaceBuilder_Ledge_Soup soup;
    for (int i = 0; i < n; ++i) soup.insert_ledge(ledges[i]);
    IVP_Template_Surbuild_LedgeSoup templ;
    templ.build_root_convex_hull = root_hull;
    return soup.compile(&templ); /* frees the input ledges (template default) */
}

static void geo_quat(IVP_U_Quat *q, double x, double y, double z, double w) {
    q->x = x; q->y = y; q->z = z; q->w = w;
}

static IVP_Polygon *geo_static_polygon(IVP_Environment *env, IVP_SurfaceManager *surman, IVP_Material *mat,
                                       const IVP_U_Quat *q, const IVP_U_Point *pos) {
    IVP_Template_Real_Object templ_obj;
    configure_static_template(&templ_obj, mat);
    IVP_Polygon *obj = env->create_polygon(surman, &templ_obj, q, pos);
    obj->enable_collision_detection(IVP_TRUE);
    return obj;
}

/* inertia == NULL: reference default rot_inertia_is_factor with (1,1,1),
 * i.e. the surface's unit mass inertia times the mass */
static IVP_Polygon *geo_dynamic_polygon(IVP_Environment *env, IVP_Compact_Surface *cs, IVP_Material *mat,
                                        double mass, const double *inertia, double speed_damp,
                                        const IVP_U_Quat *q, const IVP_U_Point *pos) {
    IVP_SurfaceManager_Polygon *surman = new IVP_SurfaceManager_Polygon(cs);
    IVP_Template_Real_Object templ_obj;
    configure_dynamic_template(&templ_obj, mat, mass);
    if (inertia) configure_explicit_inertia(&templ_obj, inertia[0], inertia[1], inertia[2]);
    templ_obj.speed_damp_factor = speed_damp;
    IVP_Polygon *obj = env->create_polygon(surman, &templ_obj, q, pos);
    wake_and_enable(obj);
    return obj;
}

static IVP_Polygon *geo_dynamic_box(IVP_Environment *env, IVP_Material *mat, double hx, double hy, double hz,
                                    double mass, double ix, double iy, double iz, const IVP_U_Quat *q,
                                    const IVP_U_Point *pos) {
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);
    return create_dynamic_box(env, mat, hx, hy, hz, mass, ix, iy, iz, 0.0, &rot_damp_zero, q, pos);
}

static IVP_Ball *geo_dynamic_ball(IVP_Environment *env, IVP_Material *mat, double radius, double mass,
                                  double inertia, const IVP_U_Point *pos) {
    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    IVP_U_Point rot_damp_zero; rot_damp_zero.set(0.0, 0.0, 0.0);
    return create_dynamic_ball(env, mat, radius, mass, inertia, inertia, inertia, 0.0, &rot_damp_zero,
                               &q_ident, pos);
}

/* ── shared shapes ─────────────────────────────────────────────────────── */

static const double GEO_TETRA[4 * 3] = {
    0.0, -0.35, 0.0,
    0.3, 0.2, 0.3,
    -0.35, 0.2, 0.15,
    0.05, 0.2, -0.35,
};

static const double GEO_PRISM[6 * 3] = {
    -0.4, 0.25, -0.5,  0.4, 0.25, -0.5,  0.0, -0.45, -0.5,
    -0.4, 0.25, 0.5,   0.4, 0.25, 0.5,   0.0, -0.45, 0.5,
};

static const double GEO_PYRAMID[5 * 3] = {
    -0.35, 0.2, -0.35,  0.35, 0.2, -0.35,  0.35, 0.2, 0.35,  -0.35, 0.2, 0.35,
    0.05, -0.45, 0.0,
};

static const double GEO_ROCK[10 * 3] = {
    0.42, 0.05, 0.11,   -0.38, 0.12, 0.2,   0.1, -0.41, 0.05,  0.05, 0.36, -0.12,
    -0.15, -0.2, 0.37,  0.22, 0.18, -0.33,  -0.27, -0.25, -0.24, 0.31, -0.22, 0.29,
    -0.05, 0.3, 0.33,   0.12, 0.02, -0.44,
};

/* octagonal cylinder, axis along z: radius 0.35, half length 0.4 */
static void geo_cylinder_points(double *c) {
    static const double ring[8][2] = {
        {0.35, 0.0}, {0.24748737341529164, 0.24748737341529164}, {0.0, 0.35},
        {-0.24748737341529164, 0.24748737341529164}, {-0.35, 0.0},
        {-0.24748737341529164, -0.24748737341529164}, {0.0, -0.35},
        {0.24748737341529164, -0.24748737341529164}};
    int k = 0;
    for (int s = -1; s <= 1; s += 2)
        for (int i = 0; i < 8; ++i) {
            c[k++] = ring[i][0];
            c[k++] = ring[i][1];
            c[k++] = (double)s * 0.4;
        }
}

/* ── concave_static: concave trough (ledge soup, no root hull) ─────────── */

static void setup_concave_static(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_static(0.7, 0.1);
    static IVP_Material_Simple mat_dyn(0.5, 0.2);

    IVP_Compact_Ledge *ledges[5];
    ledges[0] = geo_box_ledge(3.0, 0.25, 1.5, 0.0, 0.25, 0.0);      /* floor */
    static const double ramp[8 * 3] = {
        -3.0, 0.0, -1.5,  -3.0, 0.0, 1.5,  -6.0, -2.5, -1.5,  -6.0, -2.5, 1.5,
        -6.0, 0.5, -1.5,  -6.0, 0.5, 1.5,  -3.0, 0.5, -1.5,   -3.0, 0.5, 1.5,
    };
    ledges[1] = geo_hull_ledge(ramp, 8);                              /* ramp */
    ledges[2] = geo_box_ledge(0.25, 1.25, 1.5, 3.25, -0.75, 0.0);   /* right wall */
    ledges[3] = geo_box_ledge(0.4, 0.15, 1.5, 1.0, -0.15, 0.0);     /* bump */
    ledges[4] = geo_box_ledge(4.75, 0.75, 0.25, -1.25, -0.25, -1.75); /* back wall */
    IVP_Compact_Surface *trough = geo_compile_soup(ledges, 5, IVP_FALSE);

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    IVP_U_Point pos0; pos0.set(0.0, 0.0, 0.0);
    int idx = 0;
    scene->objects[idx] = geo_static_polygon(env, new IVP_SurfaceManager_Polygon(trough), &mat_static, &q_ident, &pos0);
    scene->types[idx++] = "trough";

    IVP_U_Point p1; p1.set(-5.2, -4.0, 0.3);
    scene->objects[idx] = geo_dynamic_ball(env, &mat_dyn, 0.4, 1.0, 0.064, &p1);
    scene->types[idx++] = "ball";

    IVP_U_Quat q2; geo_quat(&q2, 0.0, 0.0, 0.17364817766693033, 0.984807753012208);
    IVP_U_Point p2; p2.set(-4.0, -3.0, -0.6);
    scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.3, 0.3, 0.3, 0.8, 0.048, 0.048, 0.048, &q2, &p2);
    scene->types[idx++] = "box";

    IVP_U_Point p3; p3.set(1.5, -2.0, 0.4);
    IVP_Ball *b3 = geo_dynamic_ball(env, &mat_dyn, 0.35, 0.7, 0.0343, &p3);
    b3->get_core()->speed.set(3.0f, 0.0f, 0.0f);
    scene->objects[idx] = b3;
    scene->types[idx++] = "ball";

    IVP_U_Quat q4; geo_quat(&q4, 0.3007057995042731, 0.0, 0.0, 0.9537169507482269);
    IVP_U_Point p4; p4.set(0.8, -1.2, -0.3);
    scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.4, 0.2, 0.3, 1.2, 0.052, 0.1, 0.08, &q4, &p4);
    scene->types[idx++] = "box";

    IVP_U_Point p5; p5.set(-2.0, -2.5, 0.5);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_TETRA, 4), &mat_dyn, 0.5, 0, 0.0,
                                              &q_ident, &p5);
    scene->types[idx++] = "hull";

    scene->count = idx;
}

/* ── compound_dynamic: moving compounds (with / without root hull) ─────── */

static void setup_compound_dynamic(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.6, 0.15);

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_ground, 20.0, 0.5, 20.0, &q_ident, &pos_ground);
    scene->types[idx++] = "ground";

    /* L shape, no root hull, mass center off the origin */
    IVP_Compact_Ledge *l_ledges[2];
    l_ledges[0] = geo_box_ledge(0.6, 0.15, 0.3, 0.0, 0.0, 0.0);
    l_ledges[1] = geo_box_ledge(0.15, 0.45, 0.3, -0.45, -0.6, 0.0);
    IVP_Compact_Surface *l_shape = geo_compile_soup(l_ledges, 2, IVP_FALSE);
    IVP_U_Quat q1; geo_quat(&q1, 0.15529142706151244, 0.0, 0.2070552360820166, 0.9659258262890683);
    IVP_U_Point p1; p1.set(-1.2, -1.0, 0.0);
    IVP_Polygon *lobj = geo_dynamic_polygon(env, l_shape, &mat_dyn, 2.0, 0, 0.0, &q1, &p1);
    lobj->get_core()->speed.set(1.0f, 0.0f, 0.0f);
    scene->objects[idx] = lobj;
    scene->types[idx++] = "compound";

    /* dumbbell with root convex hull */
    IVP_Compact_Ledge *d_ledges[3];
    d_ledges[0] = geo_box_ledge(0.3, 0.3, 0.3, -0.8, 0.0, 0.0);
    d_ledges[1] = geo_box_ledge(0.3, 0.3, 0.3, 0.8, 0.0, 0.0);
    d_ledges[2] = geo_box_ledge(0.5, 0.08, 0.08, 0.0, 0.0, 0.0);
    IVP_Compact_Surface *dumbbell = geo_compile_soup(d_ledges, 3, IVP_TRUE);
    IVP_U_Quat q2; geo_quat(&q2, 0.0, 0.0, 0.21643961393810288, 0.9762960071199334);
    IVP_U_Point p2; p2.set(1.4, -2.2, 0.25);
    IVP_Polygon *dobj = geo_dynamic_polygon(env, dumbbell, &mat_dyn, 3.0, 0, 0.0, &q2, &p2);
    dobj->get_core()->rot_speed.set(0.5f, 0.0f, 1.5f);
    scene->objects[idx] = dobj;
    scene->types[idx++] = "compound";

    IVP_U_Point p3; p3.set(1.3, -4.0, 0.0);
    scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.25, 0.25, 0.25, 0.5, 0.02, 0.02, 0.02, &q_ident, &p3);
    scene->types[idx++] = "box";

    IVP_U_Point p4; p4.set(-1.5, -3.5, 0.0);
    scene->objects[idx] = geo_dynamic_ball(env, &mat_dyn, 0.3, 0.4, 0.0144, &p4);
    scene->types[idx++] = "ball";

    scene->count = idx;
}

/* ── convex_hulls: irregular pointsoup hulls piling up ─────────────────── */

static void setup_convex_hulls(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.55, 0.1);

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_ground, 10.0, 0.5, 10.0, &q_ident, &pos_ground);
    scene->types[idx++] = "ground";

    IVP_U_Quat q1; geo_quat(&q1, 0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084);
    IVP_U_Point p1; p1.set(-1.5, -1.0, 0.0);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_TETRA, 4), &mat_dyn, 0.6, 0, 0.0, &q1, &p1);
    scene->types[idx++] = "tetra";

    IVP_U_Quat q2; geo_quat(&q2, 0.0, 0.0, 0.573576436351046, 0.8191520442889918);
    IVP_U_Point p2; p2.set(0.0, -1.0, 0.0);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_PRISM, 6), &mat_dyn, 0.9, 0, 0.0, &q2, &p2);
    scene->types[idx++] = "prism";

    double cyl[16 * 3];
    geo_cylinder_points(cyl);
    IVP_U_Point p3; p3.set(1.5, -0.5, 0.0);
    IVP_Polygon *cobj = geo_dynamic_polygon(env, geo_hull_surface(cyl, 16), &mat_dyn, 1.0, 0, 0.0, &q_ident, &p3);
    cobj->get_core()->speed.set(-1.5f, 0.0f, 0.0f);
    scene->objects[idx] = cobj;
    scene->types[idx++] = "cylinder";

    IVP_U_Quat q4; geo_quat(&q4, 0.24399876718044458, 0.24399876718044458, 0.24399876718044458, 0.9063077870366499);
    IVP_U_Point p4; p4.set(-0.3, -2.6, 0.1);
    static const double rock_inertia[3] = {0.09, 0.1, 0.08};
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_ROCK, 10), &mat_dyn, 1.3, rock_inertia, 0.0,
                                              &q4, &p4);
    scene->types[idx++] = "rock";

    IVP_U_Point p5; p5.set(-1.4, -2.8, 0.05);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_PYRAMID, 5), &mat_dyn, 0.7, 0, 0.0,
                                              &q_ident, &p5);
    scene->types[idx++] = "pyramid";

    scene->count = idx;
}

/* ── grid_terrain: compact grid bowl (IVP_GridBuilder_Array) ───────────── */

#define GEO_GRID_N 10

static void geo_grid_heights(float *h) {
    for (int r = 0; r < GEO_GRID_N; ++r)
        for (int c = 0; c < GEO_GRID_N; ++c) {
            int d = (2 * r - 9) * (2 * r - 9) + (2 * c - 9) * (2 * c - 9);
            h[r * GEO_GRID_N + c] = 0.02f * (float)d + 0.1f * (float)((r * 3 + c * 5) % 4);
        }
}

static void setup_grid_terrain(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_static(0.7, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.1);

    static float heights[GEO_GRID_N * GEO_GRID_N];
    geo_grid_heights(heights);
    IVP_Template_Compact_Grid gt;
    gt.row_info.n_points = GEO_GRID_N;
    gt.row_info.maps_to = IVP_INDEX_X;
    gt.row_info.invert_axis = IVP_FALSE;
    gt.column_info.n_points = GEO_GRID_N;
    gt.column_info.maps_to = IVP_INDEX_Z;
    gt.column_info.invert_axis = IVP_FALSE;
    gt.height_maps_to = IVP_INDEX_Y;
    gt.height_invert_axis = IVP_TRUE;
    gt.grid_field_size = 1.0f;
    gt.position_origin_os.set(0.0f, 0.0f, 0.0f);
    IVP_U_Memory *mm = new IVP_U_Memory();
    mm->init_mem();
    IVP_Compact_Grid *grid = IVP_GridBuilder_Array::convert_array_to_compact_grid(mm, &gt, heights);

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    int idx = 0;
    IVP_U_Point pos_grid; pos_grid.set(-4.5, 0.0, -4.5);
    scene->objects[idx] = geo_static_polygon(env, new IVP_SurfaceManager_Grid(grid), &mat_static, &q_ident, &pos_grid);
    scene->types[idx++] = "grid";

    IVP_U_Point p1; p1.set(-3.0, -2.5, -0.4);
    scene->objects[idx] = geo_dynamic_ball(env, &mat_dyn, 0.4, 1.0, 0.064, &p1);
    scene->types[idx++] = "ball";

    IVP_U_Point p2; p2.set(2.6, -2.2, 1.5);
    scene->objects[idx] = geo_dynamic_ball(env, &mat_dyn, 0.3, 0.6, 0.0216, &p2);
    scene->types[idx++] = "ball";

    IVP_U_Quat q3; geo_quat(&q3, 0.0, 0.13052619222005157, 0.0, 0.9914448613738104);
    IVP_U_Point p3; p3.set(0.4, -2.6, -3.1);
    scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.3, 0.3, 0.3, 0.8, 0.048, 0.048, 0.048, &q3, &p3);
    scene->types[idx++] = "box";

    IVP_U_Quat q4; geo_quat(&q4, 0.25881904510252074, 0.0, 0.0, 0.9659258262890683);
    IVP_U_Point p4; p4.set(-1.8, -2.4, 2.6);
    scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.5, 0.2, 0.35, 1.5, 0.08125, 0.18625, 0.145, &q4, &p4);
    scene->types[idx++] = "box";

    IVP_U_Quat q5; geo_quat(&q5, 0.0, 0.49999999999999994, 0.0, 0.8660254037844387);
    IVP_U_Point p5; p5.set(3.0, -2.4, -1.2);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_PYRAMID, 5), &mat_dyn, 0.7, 0, 0.0,
                                              &q5, &p5);
    scene->types[idx++] = "pyramid";

    scene->count = idx;
}

/* ── more shapes ───────────────────────────────────────────────────────── */

static const double GEO_GEODE[32 * 3] = {
    0.092, 0.358, 0.0,     -0.112, 0.325, 0.102,   0.018, 0.328, -0.208,   0.134, 0.277, 0.175,
    -0.262, 0.275, -0.046, 0.237, 0.244, -0.151,   -0.074, 0.21, 0.274,    -0.149, 0.202, -0.286,
    0.292, 0.165, 0.107,   -0.318, 0.153, 0.131,   0.141, 0.122, -0.301,   0.102, 0.1, 0.325,
    -0.317, 0.082, -0.184, 0.386, 0.062, -0.085,   -0.204, 0.033, 0.291,   -0.047, 0.011, -0.36,
    0.296, -0.012, 0.25,   -0.405, -0.038, 0.017,  0.269, -0.06, -0.268,   -0.017, -0.082, 0.364,
    -0.251, -0.115, -0.301, 0.328, -0.121, 0.044,  -0.301, -0.163, 0.21,   0.071, -0.172, -0.316,
    0.151, -0.19, 0.263,   -0.273, -0.212, -0.087, 0.252, -0.242, -0.117,  -0.107, -0.287, 0.256,
    -0.076, -0.282, -0.212, 0.183, -0.325, 0.096,  -0.159, -0.352, 0.042,  0.05, -0.361, -0.078,
};

static const double GEO_SMALL_TETRA[4 * 3] = {
    0.0, -0.175, 0.0,  0.15, 0.1, 0.15,  -0.175, 0.1, 0.075,  0.025, 0.1, -0.175,
};

/* 16-gon disc, axis y: radius 0.45, half thickness 0.05 */
static void geo_disc_points(double *c) {
    static const double ring[16][2] = {
        {0.45, 0.0}, {0.4157, 0.1722}, {0.3182, 0.3182}, {0.1722, 0.4157},
        {0.0, 0.45}, {-0.1722, 0.4157}, {-0.3182, 0.3182}, {-0.4157, 0.1722},
        {-0.45, 0.0}, {-0.4157, -0.1722}, {-0.3182, -0.3182}, {-0.1722, -0.4157},
        {0.0, -0.45}, {0.1722, -0.4157}, {0.3182, -0.3182}, {0.4157, -0.1722}};
    int k = 0;
    for (int s = -1; s <= 1; s += 2)
        for (int i = 0; i < 16; ++i) {
            c[k++] = ring[i][0];
            c[k++] = (double)s * 0.05;
            c[k++] = ring[i][1];
        }
}

/* hexagonal needle along x: half length 0.6, radius 0.06 */
static void geo_needle_points(double *c) {
    static const double ring[6][2] = {
        {0.06, 0.0}, {0.03, 0.052}, {-0.03, 0.052}, {-0.06, 0.0}, {-0.03, -0.052}, {0.03, -0.052}};
    int k = 0;
    for (int s = -1; s <= 1; s += 2)
        for (int i = 0; i < 6; ++i) {
            c[k++] = (double)s * 0.6;
            c[k++] = ring[i][0];
            c[k++] = ring[i][1];
        }
}

static IVP_Compact_Surface *geo_dumbbell_surface() {
    IVP_Compact_Ledge *d_ledges[3];
    d_ledges[0] = geo_box_ledge(0.3, 0.3, 0.3, -0.8, 0.0, 0.0);
    d_ledges[1] = geo_box_ledge(0.3, 0.3, 0.3, 0.8, 0.0, 0.0);
    d_ledges[2] = geo_box_ledge(0.5, 0.08, 0.08, 0.0, 0.0, 0.0);
    return geo_compile_soup(d_ledges, 3, IVP_TRUE);
}

static IVP_Compact_Surface *geo_l_surface() {
    IVP_Compact_Ledge *l_ledges[2];
    l_ledges[0] = geo_box_ledge(0.6, 0.15, 0.3, 0.0, 0.0, 0.0);
    l_ledges[1] = geo_box_ledge(0.15, 0.45, 0.3, -0.45, -0.6, 0.0);
    return geo_compile_soup(l_ledges, 2, IVP_FALSE);
}

/* ── hull_pile: mixed hulls and compounds piling up in a rotated hopper ── */

static void setup_hull_pile(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_static(0.6, 0.1);
    static IVP_Material_Simple mat_dyn(0.45, 0.2);

    IVP_Compact_Ledge *ledges[5];
    ledges[0] = geo_box_ledge(1.0, 0.1, 1.0, 0.0, 0.1, 0.0);
    static const double wall_xm[8 * 3] = {
        -1.0, 0.2, -2.8,  -1.0, 0.2, 2.8,  -2.8, -3.0, -2.8,  -2.8, -3.0, 2.8,
        -3.1, -3.0, -2.8, -3.1, -3.0, 2.8, -1.3, 0.2, -2.8,   -1.3, 0.2, 2.8,
    };
    static const double wall_xp[8 * 3] = {
        1.0, 0.2, -2.8,  1.0, 0.2, 2.8,  2.8, -3.0, -2.8,  2.8, -3.0, 2.8,
        3.1, -3.0, -2.8, 3.1, -3.0, 2.8, 1.3, 0.2, -2.8,   1.3, 0.2, 2.8,
    };
    static const double wall_zm[8 * 3] = {
        -2.8, 0.2, -1.0,  2.8, 0.2, -1.0,  -2.8, -3.0, -2.8,  2.8, -3.0, -2.8,
        -2.8, -3.0, -3.1, 2.8, -3.0, -3.1, -2.8, 0.2, -1.3,   2.8, 0.2, -1.3,
    };
    static const double wall_zp[8 * 3] = {
        -2.8, 0.2, 1.0,  2.8, 0.2, 1.0,  -2.8, -3.0, 2.8,  2.8, -3.0, 2.8,
        -2.8, -3.0, 3.1, 2.8, -3.0, 3.1, -2.8, 0.2, 1.3,   2.8, 0.2, 1.3,
    };
    ledges[1] = geo_hull_ledge(wall_xm, 8);
    ledges[2] = geo_hull_ledge(wall_xp, 8);
    ledges[3] = geo_hull_ledge(wall_zm, 8);
    ledges[4] = geo_hull_ledge(wall_zp, 8);
    IVP_Compact_Surface *hopper = geo_compile_soup(ledges, 5, IVP_FALSE);

    int idx = 0;
    IVP_U_Quat q_hopper; geo_quat(&q_hopper, 0.0, 0.25881904510252074, 0.0, 0.9659258262890683);
    IVP_U_Point pos_hopper; pos_hopper.set(0.5, 1.0, -0.3);
    scene->objects[idx] = geo_static_polygon(env, new IVP_SurfaceManager_Polygon(hopper), &mat_static, &q_hopper,
                                             &pos_hopper);
    scene->types[idx++] = "hopper";

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    IVP_U_Quat q_a; geo_quat(&q_a, 0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084);
    IVP_U_Quat q_b; geo_quat(&q_b, 0.24399876718044458, 0.24399876718044458, 0.24399876718044458, 0.9063077870366499);
    IVP_U_Quat q_c; geo_quat(&q_c, 0.15529142706151244, 0.0, 0.2070552360820166, 0.9659258262890683);

    IVP_U_Point p;
    p.set(0.6, -2.0, -0.2);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_GEODE, 32), &mat_dyn, 1.2, 0, 0.0, &q_ident, &p);
    scene->types[idx++] = "geode";

    double disc[32 * 3];
    geo_disc_points(disc);
    p.set(0.3, -2.9, -0.5);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(disc, 32), &mat_dyn, 0.8, 0, 0.0, &q_a, &p);
    scene->types[idx++] = "disc";

    double needle[12 * 3];
    geo_needle_points(needle);
    p.set(0.9, -3.6, 0.1);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(needle, 12), &mat_dyn, 0.3, 0, 0.0, &q_b, &p);
    scene->types[idx++] = "needle";

    p.set(0.2, -4.4, -0.1);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_l_surface(), &mat_dyn, 2.0, 0, 0.0, &q_c, &p);
    scene->types[idx++] = "compound";

    p.set(0.7, -5.6, -0.4);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_dumbbell_surface(), &mat_dyn, 3.0, 0, 0.0, &q_a, &p);
    scene->types[idx++] = "compound";

    p.set(0.1, -6.6, -0.3);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_ROCK, 10), &mat_dyn, 1.3, 0, 0.0, &q_b, &p);
    scene->types[idx++] = "rock";

    p.set(0.8, -7.4, 0.0);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_PRISM, 6), &mat_dyn, 0.9, 0, 0.0, &q_c, &p);
    scene->types[idx++] = "prism";

    p.set(0.4, -8.2, -0.6);
    scene->objects[idx] = geo_dynamic_ball(env, &mat_dyn, 0.3, 0.6, 0.0216, &p);
    scene->types[idx++] = "ball";

    p.set(0.5, -9.0, -0.1);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_PYRAMID, 5), &mat_dyn, 0.7, 0, 0.0, &q_a, &p);
    scene->types[idx++] = "pyramid";

    p.set(0.6, -9.8, -0.4);
    scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.25, 0.2, 0.3, 0.9, 0.039, 0.0459, 0.0309, &q_b, &p);
    scene->types[idx++] = "box";

    p.set(0.3, -10.6, -0.2);
    scene->objects[idx] = geo_dynamic_ball(env, &mat_dyn, 0.22, 0.4, 0.007744, &p);
    scene->types[idx++] = "ball";

    scene->count = idx;
}

/* ── galton: deep static ledge soup (49 ledges) with balls and hulls ───── */

static void setup_galton(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_static(0.4, 0.3);
    static IVP_Material_Simple mat_dyn(0.3, 0.4);

    IVP_Compact_Ledge *ledges[64];
    int n = 0;
    ledges[n++] = geo_box_ledge(3.4, 3.0, 0.1, 0.0, -2.0, -0.6);  /* back plate */
    ledges[n++] = geo_box_ledge(3.4, 3.0, 0.1, 0.0, -2.0, 0.6);   /* front plate */
    ledges[n++] = geo_box_ledge(3.4, 0.1, 0.7, 0.0, 1.0, 0.0);    /* floor */
    ledges[n++] = geo_box_ledge(0.1, 3.0, 0.7, -3.3, -2.0, 0.0);  /* side walls */
    ledges[n++] = geo_box_ledge(0.1, 3.0, 0.7, 3.3, -2.0, 0.0);
    for (int i = 0; i < 5; ++i)                                     /* bin dividers */
        ledges[n++] = geo_box_ledge(0.03, 0.4, 0.5, -2.0 + 1.0 * (double)i, 0.5, 0.0);
    for (int row = 0; row < 6; ++row) {                            /* diamond pegs */
        int n_pegs = (row % 2 == 0) ? 7 : 6;
        double x0 = (row % 2 == 0) ? -2.4 : -2.0;
        double py = -3.5 + 0.7 * (double)row;
        for (int i = 0; i < n_pegs; ++i) {
            double px = x0 + 0.8 * (double)i;
            double c[8 * 3];
            int k = 0;
            for (int s = -1; s <= 1; s += 2) {
                double z = (double)s * 0.5;
                c[k++] = px + 0.15; c[k++] = py;        c[k++] = z;
                c[k++] = px;        c[k++] = py + 0.15; c[k++] = z;
                c[k++] = px - 0.15; c[k++] = py;        c[k++] = z;
                c[k++] = px;        c[k++] = py - 0.15; c[k++] = z;
            }
            ledges[n++] = geo_hull_ledge(c, 8);
        }
    }
    IVP_Compact_Surface *board = geo_compile_soup(ledges, n, IVP_FALSE);

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    int idx = 0;
    IVP_U_Point pos0; pos0.set(0.0, 0.0, 0.0);
    scene->objects[idx] = geo_static_polygon(env, new IVP_SurfaceManager_Polygon(board), &mat_static, &q_ident, &pos0);
    scene->types[idx++] = "board";

    IVP_U_Quat q_a; geo_quat(&q_a, 0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084);
    for (int i = 0; i < 12; ++i) {
        IVP_U_Point p;
        p.set(-0.3 + 0.17 * (double)(i % 5), -4.6 - 0.45 * (double)i, -0.1 + 0.05 * (double)(i % 3));
        if (i % 6 == 2) {
            scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.12, 0.12, 0.12, 0.2, 0.00192, 0.00192, 0.00192,
                                                  &q_a, &p);
            scene->types[idx++] = "box";
        } else if (i % 6 == 5) {
            scene->objects[idx] = geo_dynamic_polygon(env, geo_hull_surface(GEO_SMALL_TETRA, 4), &mat_dyn, 0.15, 0,
                                                      0.0, &q_a, &p);
            scene->types[idx++] = "hull";
        } else {
            scene->objects[idx] = geo_dynamic_ball(env, &mat_dyn, 0.18, 0.3, 0.003888, &p);
            scene->types[idx++] = "ball";
        }
    }
    scene->count = idx;
}

/* ── fast_impacts: fast projectiles against thin plates ────────────────── */

static void setup_fast_impacts(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_static(0.5, 0.5);
    static IVP_Material_Simple mat_dyn(0.4, 0.6);

    /* closed room of thin plates with a tilted plate and a low inner wall */
    IVP_Compact_Ledge *ledges[8];
    ledges[0] = geo_box_ledge(4.0, 0.025, 2.5, 0.0, 0.5, 0.0);     /* floor */
    ledges[1] = geo_box_ledge(4.0, 0.025, 2.5, 0.0, -3.0, 0.0);    /* ceiling */
    ledges[2] = geo_box_ledge(0.025, 1.75, 2.5, 4.0, -1.25, 0.0);  /* walls */
    ledges[3] = geo_box_ledge(0.025, 1.75, 2.5, -4.0, -1.25, 0.0);
    ledges[4] = geo_box_ledge(4.0, 1.75, 0.025, 0.0, -1.25, 2.5);
    ledges[5] = geo_box_ledge(4.0, 1.75, 0.025, 0.0, -1.25, -2.5);
    ledges[6] = geo_box_ledge(0.025, 0.8, 1.2, 1.5, -0.3, 0.0);    /* inner wall */
    static const double tilted[8 * 3] = {
        -1.5, 0.5, -2.4,   -1.5, 0.5, 2.4,   -3.0, -1.5, -2.4,   -3.0, -1.5, 2.4,
        -1.54, 0.53, -2.4, -1.54, 0.53, 2.4, -3.04, -1.47, -2.4, -3.04, -1.47, 2.4,
    };
    ledges[7] = geo_hull_ledge(tilted, 8);
    IVP_Compact_Surface *plates = geo_compile_soup(ledges, 8, IVP_FALSE);

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    int idx = 0;
    IVP_U_Point pos0; pos0.set(0.0, 0.0, 0.0);
    scene->objects[idx] = geo_static_polygon(env, new IVP_SurfaceManager_Polygon(plates), &mat_static, &q_ident, &pos0);
    scene->types[idx++] = "plates";

    IVP_U_Point p;
    p.set(-1.0, -0.5, 0.0);
    IVP_Ball *b1 = geo_dynamic_ball(env, &mat_dyn, 0.1, 0.2, 0.0008, &p);
    b1->get_core()->speed.set(40.0f, -2.0f, 0.0f);
    scene->objects[idx] = b1; scene->types[idx++] = "ball";

    IVP_U_Quat q_a; geo_quat(&q_a, 0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084);
    p.set(0.0, -1.0, 0.5);
    IVP_Polygon *b2 = geo_dynamic_box(env, &mat_dyn, 0.1, 0.1, 0.1, 0.3, 0.002, 0.002, 0.002, &q_a, &p);
    b2->get_core()->speed.set(-35.0f, -5.0f, 1.0f);
    b2->get_core()->rot_speed.set(10.0f, 0.0f, 5.0f);
    scene->objects[idx] = b2; scene->types[idx++] = "box";

    p.set(1.0, -2.0, -1.0);
    IVP_Polygon *b3 = geo_dynamic_polygon(env, geo_hull_surface(GEO_TETRA, 4), &mat_dyn, 0.5, 0, 0.0, &q_ident, &p);
    b3->get_core()->speed.set(20.0f, 25.0f, 0.0f);
    scene->objects[idx] = b3; scene->types[idx++] = "hull";

    IVP_U_Quat q_d; geo_quat(&q_d, 0.0, 0.0, 0.21643961393810288, 0.9762960071199334);
    p.set(-1.0, -1.5, -1.5);
    IVP_Polygon *b4 = geo_dynamic_polygon(env, geo_dumbbell_surface(), &mat_dyn, 3.0, 0, 0.0, &q_d, &p);
    b4->get_core()->speed.set(0.0f, 15.0f, 0.0f);
    b4->get_core()->rot_speed.set(0.0f, 0.0f, 40.0f);
    scene->objects[idx] = b4; scene->types[idx++] = "compound";

    double needle[12 * 3];
    geo_needle_points(needle);
    IVP_U_Quat q_b; geo_quat(&q_b, 0.24399876718044458, 0.24399876718044458, 0.24399876718044458, 0.9063077870366499);
    p.set(1.5, -2.5, 1.5);
    IVP_Polygon *b5 = geo_dynamic_polygon(env, geo_hull_surface(needle, 12), &mat_dyn, 0.2, 0, 0.0, &q_b, &p);
    b5->get_core()->speed.set(-10.0f, -20.0f, 0.0f);
    b5->get_core()->rot_speed.set(0.0f, 20.0f, 30.0f);
    scene->objects[idx] = b5; scene->types[idx++] = "needle";

    p.set(-2.0, -2.0, 1.0);
    IVP_Ball *b6 = geo_dynamic_ball(env, &mat_dyn, 0.25, 1.0, 0.025, &p);
    b6->get_core()->speed.set(-30.0f, 10.0f, -2.0f);
    scene->objects[idx] = b6; scene->types[idx++] = "ball";

    scene->count = idx;
}

/* ── spawn_remove: objects created and deleted during the simulation ───── */

namespace {

struct SpawnRemoveScenario {
    SceneObjects *scene;
    IVP_Real_Object *middle_box; /* unlisted, deleted at step 300 */
    IVP_Real_Object *hull;       /* unlisted, deleted at step 450 */
};

void spawn_remove_step_hook(int step, IVP_Environment *env, ScenarioResources *resources) {
    SpawnRemoveScenario *sr = (SpawnRemoveScenario *)resources->scenario_data;
    if (!sr) return;
    static IVP_Material_Simple mat_dyn(0.5, 0.2);
    SceneObjects *scene = sr->scene;
    if (step == 200) {
        IVP_U_Quat q; geo_quat(&q, 0.24399876718044458, 0.24399876718044458, 0.24399876718044458, 0.9063077870366499);
        IVP_U_Point p; p.set(-2.0, -2.5, 0.1);
        scene->objects[scene->count] = geo_dynamic_polygon(env, geo_hull_surface(GEO_ROCK, 10), &mat_dyn, 1.3, 0, 0.0,
                                                           &q, &p);
        scene->types[scene->count++] = "rock";
    }
    if (step == 300 && sr->middle_box) {
        sr->middle_box->delete_and_check_vicinity();
        sr->middle_box = 0;
    }
    if (step == 400) {
        IVP_U_Quat q; geo_quat(&q, 0.15529142706151244, 0.0, 0.2070552360820166, 0.9659258262890683);
        IVP_U_Point p; p.set(4.3, -1.6, 0.3);
        IVP_Polygon *l = geo_dynamic_polygon(env, geo_l_surface(), &mat_dyn, 2.0, 0, 0.0, &q, &p);
        l->get_core()->speed.set(-1.0f, 0.0f, 0.0f);
        scene->objects[scene->count] = l;
        scene->types[scene->count++] = "compound";
    }
    if (step == 450 && sr->hull) {
        sr->hull->delete_and_check_vicinity();
        sr->hull = 0;
    }
    if (step == 550) {
        IVP_U_Point p; p.set(-4.0, 0.9, 0.0);
        IVP_Ball *b = geo_dynamic_ball(env, &mat_dyn, 0.2, 0.3, 0.0048, &p);
        b->get_core()->speed.set(15.0f, 0.0f, 0.0f);
        scene->objects[scene->count] = b;
        scene->types[scene->count++] = "ball";
    }
}

void spawn_remove_cleanup_hook(ScenarioResources *resources) {
    delete (SpawnRemoveScenario *)resources->scenario_data;
    resources->scenario_data = 0;
}

} // namespace

static void setup_spawn_remove(IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    static IVP_Material_Simple mat_ground(0.8, 0.0);
    static IVP_Material_Simple mat_dyn(0.5, 0.2);

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_ground, 20.0, 0.5, 20.0, &q_ident, &pos_ground);
    scene->types[idx++] = "ground";

    IVP_Compact_Ledge *ledges[3];
    ledges[0] = geo_box_ledge(1.5, 0.25, 1.0, 0.0, 1.25, 0.0);
    ledges[1] = geo_box_ledge(1.0, 0.25, 1.0, 0.5, 0.75, 0.0);
    ledges[2] = geo_box_ledge(0.5, 0.25, 1.0, 1.0, 0.25, 0.0);
    IVP_U_Point pos_stairs; pos_stairs.set(4.0, 0.0, 0.0);
    scene->objects[idx] = geo_static_polygon(env, new IVP_SurfaceManager_Polygon(geo_compile_soup(ledges, 3, IVP_FALSE)),
                                             &mat_ground, &q_ident, &pos_stairs);
    scene->types[idx++] = "stairs";

    IVP_U_Point p;
    p.set(-2.0, 1.1, 0.0);
    scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.0, 0.1067, 0.1067, 0.1067, &q_ident, &p);
    scene->types[idx++] = "box";
    p.set(-2.0, 0.3, 0.0);
    IVP_Polygon *middle = geo_dynamic_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.0, 0.1067, 0.1067, 0.1067, &q_ident, &p);
    p.set(-2.0, -0.5, 0.0);
    scene->objects[idx] = geo_dynamic_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.0, 0.1067, 0.1067, 0.1067, &q_ident, &p);
    scene->types[idx++] = "box";

    p.set(0.5, 1.15, 2.5);
    scene->objects[idx] = geo_dynamic_polygon(env, geo_dumbbell_surface(), &mat_dyn, 3.0, 0, 0.0, &q_ident, &p);
    scene->types[idx++] = "compound";
    p.set(0.5, -0.2, 2.5);
    IVP_Polygon *hull = geo_dynamic_polygon(env, geo_hull_surface(GEO_TETRA, 4), &mat_dyn, 0.5, 0, 0.0, &q_ident, &p);

    p.set(4.0, -1.0, 0.2);
    scene->objects[idx] = geo_dynamic_ball(env, &mat_dyn, 0.3, 0.6, 0.0216, &p);
    scene->types[idx++] = "ball";
    scene->count = idx;

    SpawnRemoveScenario *sr = new SpawnRemoveScenario;
    sr->scene = scene;
    sr->middle_box = middle;
    sr->hull = hull;
    resources->scenario_data = sr;
    resources->step_hook = spawn_remove_step_hook;
    resources->cleanup_hook = spawn_remove_cleanup_hook;
}

/* ── long_range: long throws over a huge floor into a far concave catcher
 *    and onto a far, rotated compact grid (rows -> -z, columns -> x) ───── */

static void geo_bump_heights(float *h) {
    for (int r = 0; r < GEO_GRID_N; ++r)
        for (int c = 0; c < GEO_GRID_N; ++c)
            h[r * GEO_GRID_N + c] = 0.15f * (float)((r * 7 + c * 3) % 5);
}

static void setup_long_range(IVP_Environment *env, SceneObjects *scene) {
    static IVP_Material_Simple mat_static(0.6, 0.1);
    static IVP_Material_Simple mat_dyn(0.4, 0.3);

    IVP_U_Quat q_ident; geo_quat(&q_ident, 0.0, 0.0, 0.0, 1.0);
    int idx = 0;
    IVP_U_Point pos_ground; pos_ground.set(0.0, 2.0, 0.0);
    scene->objects[idx] = create_static_box(env, &mat_static, 300.0, 0.5, 300.0, &q_ident, &pos_ground);
    scene->types[idx++] = "ground";

    IVP_Compact_Ledge *ledges[3];
    ledges[0] = geo_box_ledge(0.5, 3.0, 6.0, 6.5, -1.5, 0.0);
    ledges[1] = geo_box_ledge(6.5, 3.0, 0.5, 0.0, -1.5, 6.5);
    ledges[2] = geo_box_ledge(6.5, 3.0, 0.5, 0.0, -1.5, -6.5);
    IVP_U_Point pos_catcher; pos_catcher.set(118.5, 0.0, 0.0);
    scene->objects[idx] = geo_static_polygon(env, new IVP_SurfaceManager_Polygon(geo_compile_soup(ledges, 3, IVP_FALSE)),
                                             &mat_static, &q_ident, &pos_catcher);
    scene->types[idx++] = "catcher";

    static float heights[GEO_GRID_N * GEO_GRID_N];
    geo_bump_heights(heights);
    IVP_Template_Compact_Grid gt;
    gt.row_info.n_points = GEO_GRID_N;
    gt.row_info.maps_to = IVP_INDEX_Z;
    gt.row_info.invert_axis = IVP_TRUE;
    gt.column_info.n_points = GEO_GRID_N;
    gt.column_info.maps_to = IVP_INDEX_X;
    gt.column_info.invert_axis = IVP_FALSE;
    gt.height_maps_to = IVP_INDEX_Y;
    gt.height_invert_axis = IVP_TRUE;
    gt.grid_field_size = 2.0f;
    gt.position_origin_os.set(1.0f, 0.5f, 2.0f);
    IVP_U_Memory *mm = new IVP_U_Memory();
    mm->init_mem();
    IVP_Compact_Grid *grid = IVP_GridBuilder_Array::convert_array_to_compact_grid(mm, &gt, heights);
    IVP_U_Quat q_grid; geo_quat(&q_grid, 0.0, 0.7071067811865475, 0.0, 0.7071067811865476);
    IVP_U_Point pos_grid; pos_grid.set(-88.0, 0.9, 0.0);
    scene->objects[idx] = geo_static_polygon(env, new IVP_SurfaceManager_Grid(grid), &mat_static, &q_grid, &pos_grid);
    scene->types[idx++] = "grid";

    IVP_U_Point p;
    p.set(0.0, -1.0, 0.0);
    IVP_Ball *b1 = geo_dynamic_ball(env, &mat_dyn, 0.5, 1.0, 0.1, &p);
    b1->get_core()->speed.set(35.0f, -12.0f, 1.0f);
    scene->objects[idx] = b1; scene->types[idx++] = "ball";

    IVP_U_Quat q_a; geo_quat(&q_a, 0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084);
    p.set(0.0, -1.0, 2.0);
    IVP_Polygon *b2 = geo_dynamic_box(env, &mat_dyn, 0.4, 0.4, 0.4, 1.0, 0.1067, 0.1067, 0.1067, &q_a, &p);
    b2->get_core()->speed.set(40.0f, -10.0f, -2.0f);
    scene->objects[idx] = b2; scene->types[idx++] = "box";

    IVP_U_Quat q_b; geo_quat(&q_b, 0.24399876718044458, 0.24399876718044458, 0.24399876718044458, 0.9063077870366499);
    p.set(0.0, -1.0, -1.0);
    IVP_Polygon *b3 = geo_dynamic_polygon(env, geo_hull_surface(GEO_ROCK, 10), &mat_dyn, 1.3, 0, 0.0, &q_b, &p);
    b3->get_core()->speed.set(-38.0f, -11.0f, -4.0f);
    scene->objects[idx] = b3; scene->types[idx++] = "rock";

    p.set(0.0, -1.0, -3.0);
    IVP_Polygon *b4 = geo_dynamic_polygon(env, geo_dumbbell_surface(), &mat_dyn, 3.0, 0, 0.0, &q_ident, &p);
    b4->get_core()->speed.set(-34.0f, -14.0f, -3.0f);
    b4->get_core()->rot_speed.set(0.0f, 3.0f, 6.0f);
    scene->objects[idx] = b4; scene->types[idx++] = "compound";

    p.set(2.0, -1.0, 0.0);
    IVP_Ball *b5 = geo_dynamic_ball(env, &mat_dyn, 0.3, 0.6, 0.0216, &p);
    b5->get_core()->speed.set(2.0f, -9.0f, 30.0f);
    scene->objects[idx] = b5; scene->types[idx++] = "ball";

    IVP_U_Quat q_c; geo_quat(&q_c, 0.15529142706151244, 0.0, 0.2070552360820166, 0.9659258262890683);
    p.set(0.0, -1.0, 4.0);
    IVP_Polygon *b6 = geo_dynamic_polygon(env, geo_l_surface(), &mat_dyn, 2.0, 0, 0.0, &q_c, &p);
    b6->get_core()->speed.set(0.0f, -20.0f, 0.0f);
    scene->objects[idx] = b6; scene->types[idx++] = "compound";

    p.set(-2.0, -1.0, 0.0);
    IVP_Polygon *b7 = geo_dynamic_polygon(env, geo_hull_surface(GEO_TETRA, 4), &mat_dyn, 0.5, 0, 0.0, &q_ident, &p);
    b7->get_core()->speed.set(30.0f, -15.0f, 0.0f);
    scene->objects[idx] = b7; scene->types[idx++] = "hull";

    scene->count = idx;
}

bool setup_geometry_scenarios(Scenario scenario, IVP_Environment *env, SceneObjects *scene,
                              ScenarioResources *resources) {
    if (scenario == SCENARIO_CONCAVE_STATIC) {
        setup_concave_static(env, scene);
        return true;
    }
    if (scenario == SCENARIO_COMPOUND_DYNAMIC) {
        setup_compound_dynamic(env, scene);
        return true;
    }
    if (scenario == SCENARIO_CONVEX_HULLS) {
        setup_convex_hulls(env, scene);
        return true;
    }
    if (scenario == SCENARIO_GRID_TERRAIN) {
        setup_grid_terrain(env, scene);
        return true;
    }
    if (scenario == SCENARIO_HULL_PILE) {
        setup_hull_pile(env, scene);
        return true;
    }
    if (scenario == SCENARIO_GALTON) {
        setup_galton(env, scene);
        return true;
    }
    if (scenario == SCENARIO_FAST_IMPACTS) {
        setup_fast_impacts(env, scene);
        return true;
    }
    if (scenario == SCENARIO_SPAWN_REMOVE) {
        setup_spawn_remove(env, scene, resources);
        return true;
    }
    if (scenario == SCENARIO_LONG_RANGE) {
        setup_long_range(env, scene);
        return true;
    }
    return false;
}

} // namespace ref_runner
