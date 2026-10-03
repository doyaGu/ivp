/* ive_sample_common.cxx -- Implementation of shared sample utilities */

#include "ive_sample_common.hxx"
#include <ivp_cache_object.hxx>

#include <SDL3/SDL.h>

#include <cmath>
#include <cstdio>
#include <cstring>

namespace ive {

/* ── Type conversions ─────────────────────────────────────────────────── */

void quat_to_col_rot(const IVP_U_Quat *q, float rot[9])
{
    double x = q->x, y = q->y, z = q->z, w = q->w;
    double x2 = x + x, y2 = y + y, z2 = z + z;
    double xx = x * x2, xy = x * y2, xz = x * z2;
    double yy = y * y2, yz = y * z2, zz = z * z2;
    double wx = w * x2, wy = w * y2, wz = w * z2;

    /* Column-major 3x3 */
    rot[0] = (float)(1.0 - (yy + zz));
    rot[1] = (float)(xy + wz);
    rot[2] = (float)(xz - wy);

    rot[3] = (float)(xy - wz);
    rot[4] = (float)(1.0 - (xx + zz));
    rot[5] = (float)(yz + wx);

    rot[6] = (float)(xz + wy);
    rot[7] = (float)(yz - wx);
    rot[8] = (float)(1.0 - (xx + yy));
}

void point_to_float3(const IVP_U_Point *p, float out[3])
{
    out[0] = (float)p->k[0];
    out[1] = (float)p->k[1];
    out[2] = (float)p->k[2];
}

void float_point_to_float3(const IVP_U_Float_Point *p, float out[3])
{
    out[0] = (float)p->k[0];
    out[1] = (float)p->k[1];
    out[2] = (float)p->k[2];
}

float speed_magnitude(const IVP_U_Float_Point *v)
{
    double x = v->k[0], y = v->k[1], z = v->k[2];
    return (float)std::sqrt(x * x + y * y + z * z);
}

/* ── Screen ray ──────────────────────────────────────────────────────── */

bool screen_ray(const ivp_camera_t *cam, int w, int h,
                float mx, float my, float ro[3], float rd[3])
{
    if (!cam || w <= 0 || h <= 0) return false;
    float fwd[3] = {
        cam->target[0] - cam->eye[0],
        cam->target[1] - cam->eye[1],
        cam->target[2] - cam->eye[2]
    };
    float fl = std::sqrt(fwd[0]*fwd[0] + fwd[1]*fwd[1] + fwd[2]*fwd[2]);
    if (fl < 1e-6f) return false;
    fwd[0] /= fl; fwd[1] /= fl; fwd[2] /= fl;

    float right[3] = {
        fwd[1]*cam->up[2] - fwd[2]*cam->up[1],
        fwd[2]*cam->up[0] - fwd[0]*cam->up[2],
        fwd[0]*cam->up[1] - fwd[1]*cam->up[0]
    };
    float rl = std::sqrt(right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
    if (rl < 1e-6f) return false;
    right[0] /= rl; right[1] /= rl; right[2] /= rl;

    float up[3] = {
        right[1]*fwd[2] - right[2]*fwd[1],
        right[2]*fwd[0] - right[0]*fwd[2],
        right[0]*fwd[1] - right[1]*fwd[0]
    };

    float aspect = (float)w / (float)h;
    float thf = std::tan(cam->fov_deg * 3.14159265358979323846f / 180.0f * 0.5f);
    float nx = (2.0f * mx / (float)w) - 1.0f;
    float ny = 1.0f - (2.0f * my / (float)h);

    ro[0] = cam->eye[0]; ro[1] = cam->eye[1]; ro[2] = cam->eye[2];
    rd[0] = fwd[0] + right[0]*nx*aspect*thf + up[0]*ny*thf;
    rd[1] = fwd[1] + right[1]*nx*aspect*thf + up[1]*ny*thf;
    rd[2] = fwd[2] + right[2]*nx*aspect*thf + up[2]*ny*thf;
    float rdl = std::sqrt(rd[0]*rd[0] + rd[1]*rd[1] + rd[2]*rd[2]);
    if (rdl < 1e-6f) return false;
    rd[0] /= rdl; rd[1] /= rdl; rd[2] /= rdl;
    return true;
}

/* ── Inertia helpers ──────────────────────────────────────────────────── */

void set_box_inertia(IVP_Template_Real_Object *t, double mass,
                     double hx, double hy, double hz)
{
    double w = 2.0 * hx, h = 2.0 * hy, d = 2.0 * hz;
    t->rot_inertia_is_factor = IVP_FALSE;
    t->rot_inertia.set(
        mass * (h * h + d * d) / 12.0,
        mass * (w * w + d * d) / 12.0,
        mass * (w * w + h * h) / 12.0);
}

void set_sphere_inertia(IVP_Template_Real_Object *t, double mass,
                        double radius)
{
    double I = 0.4 * mass * radius * radius;
    t->rot_inertia_is_factor = IVP_FALSE;
    t->rot_inertia.set(I, I, I);
}

/* ── Environment ──────────────────────────────────────────────────────── */

IVP_Environment *create_environment(double dt, IVP_Universe_Manager *um)
{
    IVP_Environment_Manager *mgr =
        IVP_Environment_Manager::get_environment_manager();
    IVP_Application_Environment appl_env;
    appl_env.universe_manager = um;
    IVP_Environment *env = mgr->create_environment(&appl_env, "IVP_Sample", 0);
    IVP_U_Point gravity(0.0, 9.81, 0.0);
    env->set_gravity(&gravity);
    env->set_delta_PSI_time(dt);
    return env;
}

/* ── Geometry builders ────────────────────────────────────────────────── */

IVP_Compact_Surface *build_box_surface(double hx, double hy, double hz)
{
    IVP_U_Point pts[8];
    IVP_U_Vector<IVP_U_Point> points;
    int idx = 0;
    for (int sz = -1; sz <= 1; sz += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sx = -1; sx <= 1; sx += 2) {
                pts[idx].set((double)sx * hx, (double)sy * hy, (double)sz * hz);
                points.add(&pts[idx]);
                idx++;
            }
    return IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface(&points);
}

/* ── Object creation ──────────────────────────────────────────────────── */

void configure_dynamic_template(IVP_Template_Real_Object *t,
                                IVP_Material *mat, double mass)
{
    *t = IVP_Template_Real_Object();
    t->physical_unmoveable = IVP_FALSE;
    t->pinned = IVP_FALSE;
    t->enable_piling_optimization = IVP_FALSE;
    t->mass = mass;
    t->material = mat;
    t->speed_damp_factor = 0.01;
    t->rot_speed_damp_factor.set(0.01, 0.01, 0.01);
    t->extra_radius = 0.0f;
}

void configure_static_template(IVP_Template_Real_Object *t,
                               IVP_Material *mat)
{
    *t = IVP_Template_Real_Object();
    t->physical_unmoveable = IVP_TRUE;
    t->pinned = IVP_TRUE;
    t->enable_piling_optimization = IVP_FALSE;
    t->mass = 0.0;
    t->material = mat;
    t->speed_damp_factor = 0.0;
    t->rot_speed_damp_factor.set(0.0, 0.0, 0.0);
    t->extra_radius = 0.0f;
}

IVP_Polygon *create_box(IVP_Environment *env, IVP_Material *mat,
                         double hx, double hy, double hz, double mass,
                         const IVP_U_Quat *q, const IVP_U_Point *pos)
{
    IVP_Compact_Surface *compact = build_box_surface(hx, hy, hz);
    IVP_SurfaceManager_Polygon *surman = new IVP_SurfaceManager_Polygon(compact);

    IVP_Template_Real_Object templ;
    if (mass > 0.0) {
        configure_dynamic_template(&templ, mat, mass);
        set_box_inertia(&templ, mass, hx, hy, hz);
    } else {
        configure_static_template(&templ, mat);
    }

    IVP_Polygon *obj = env->create_polygon(surman, &templ, q, pos);
    wake_and_enable(obj);
    return obj;
}

IVP_Ball *create_ball(IVP_Environment *env, IVP_Material *mat,
                      double radius, double mass,
                      const IVP_U_Quat *q, const IVP_U_Point *pos)
{
    IVP_Template_Real_Object templ;
    if (mass > 0.0) {
        configure_dynamic_template(&templ, mat, mass);
        set_sphere_inertia(&templ, mass, radius);
    } else {
        configure_static_template(&templ, mat);
    }

    IVP_Template_Ball tball;
    tball.radius = (IVP_FLOAT)radius;
    IVP_Ball *ball = env->create_ball(&tball, &templ, q, pos);
    wake_and_enable(ball);
    return ball;
}

void wake_and_enable(IVP_Real_Object *obj)
{
    obj->enable_collision_detection(IVP_TRUE);
    obj->ensure_in_simulation_now();
}

void set_quat_axis_angle(IVP_U_Quat *q, double ax, double ay, double az,
                         double radians)
{
    double half = 0.5 * radians;
    double s = std::sin(half);
    double c = std::cos(half);
    q->x = ax * s;
    q->y = ay * s;
    q->z = az * s;
    q->w = c;
}

/* ── Drawing helpers ──────────────────────────────────────────────────── */

void draw_object_box(ivp_renderer_t *r, IVP_Real_Object *obj,
                     double hx, double hy, double hz, const float color[3])
{
    IVP_Core *core = obj->get_core();
    float pos[3], rot[9], hs[3];
    point_to_float3(core->get_position_PSI(), pos);
    quat_to_col_rot(&core->q_world_f_core_last_psi, rot);
    hs[0] = (float)hx;
    hs[1] = (float)hy;
    hs[2] = (float)hz;
    ivp_draw_wire_box(r, pos, hs, rot, color);
}

void draw_object_ball(ivp_renderer_t *r, IVP_Real_Object *obj,
                      double radius, const float color[3])
{
    IVP_Core *core = obj->get_core();
    float pos[3];
    point_to_float3(core->get_position_PSI(), pos);
    ivp_draw_wire_sphere_ex(r, pos, (float)radius, color, IVE_SPHERE_SEGS);
}

void draw_object_cylinder(ivp_renderer_t *r, IVP_Real_Object *obj,
                           double radius, double half_height,
                           const float color[3])
{
    IVP_Core *core = obj->get_core();
    float pos[3], rot[9];
    point_to_float3(core->get_position_PSI(), pos);
    quat_to_col_rot(&core->q_world_f_core_last_psi, rot);

    /* Local Y axis becomes the cylinder axis in world space */
    float ax = rot[3], ay = rot[4], az = rot[5];
    float base[3], axis[3];
    float hh = (float)half_height;
    base[0] = pos[0] - ax * hh;
    base[1] = pos[1] - ay * hh;
    base[2] = pos[2] - az * hh;
    axis[0] = ax; axis[1] = ay; axis[2] = az;
    ivp_draw_wire_cylinder(r, base, axis, (float)(2.0 * half_height),
                           (float)radius, IVE_CYLINDER_SEGS, color);
}

void draw_velocity_arrow(ivp_renderer_t *r, IVP_Real_Object *obj,
                         const float color[3])
{
    IVP_Core *core = obj->get_core();
    float from[3], to[3];
    point_to_float3(core->get_position_PSI(), from);
    to[0] = from[0] + (float)core->speed.k[0];
    to[1] = from[1] + (float)core->speed.k[1];
    to[2] = from[2] + (float)core->speed.k[2];
    ivp_draw_arrow(r, from, to, color, 0.15f);
}

void draw_spring_line(ivp_renderer_t *r, IVP_Real_Object *a,
                      IVP_Real_Object *b, const float color[3])
{
    float pa[3], pb[3];
    point_to_float3(a->get_core()->get_position_PSI(), pa);
    point_to_float3(b->get_core()->get_position_PSI(), pb);
    ivp_draw_thick_line(r, pa, pb, color, 2.0f);
}

void draw_anchor_line(ivp_renderer_t *r, IVP_Real_Object *a,
                      double ax, double ay, double az,
                      IVP_Real_Object *b,
                      double bx, double by, double bz,
                      const float color[3])
{
    IVP_U_Float_Point local_a, local_b;
    local_a.set(ax, ay, az);
    local_b.set(bx, by, bz);

    IVP_U_Point wa, wb;
    {
        IVP_Cache_Object *ca = a->get_cache_object();
        ca->transform_position_to_world_coords(&local_a, &wa);
        ca->remove_reference();
    }
    {
        IVP_Cache_Object *cb = b->get_cache_object();
        cb->transform_position_to_world_coords(&local_b, &wb);
        cb->remove_reference();
    }

    float fa[3], fb[3];
    point_to_float3(&wa, fa);
    point_to_float3(&wb, fb);
    ivp_draw_thick_line(r, fa, fb, color, 2.0f);
}

/* ── Simulation loop ──────────────────────────────────────────────────── */

void sim_loop_init(SimLoop *sl, const ivp_renderer_t *r)
{
    std::memset(sl, 0, sizeof(*sl));
    sl->t_prev = ivp_renderer_get_time(r);
    sl->sim_dt = 1.0f / 60.0f;
    sl->max_substeps = 4;
    sl->fps_display = 60.0f;
}

float sim_loop_begin(SimLoop *sl, const ivp_renderer_t *r)
{
    double now = ivp_renderer_get_time(r);
    float dt = (float)(now - sl->t_prev);
    sl->t_prev = now;
    if (dt > 0.1f) dt = 0.1f;
    if (dt < 0.0f) dt = 0.0f;

    sl->fps_timer += dt;
    sl->frame_count++;
    if (sl->fps_timer >= 0.5f) {
        sl->fps_display = (float)sl->frame_count / sl->fps_timer;
        sl->fps_timer = 0.0f;
        sl->frame_count = 0;
    }
    return dt;
}

int sim_loop_step(SimLoop *sl, IVP_Environment *env, float frame_dt)
{
    sl->sim_accum += frame_dt;
    int steps = 0;
    while (sl->sim_accum >= sl->sim_dt && steps < sl->max_substeps) {
        IVP_Time cur = env->get_current_time();
        env->simulate_dtime(sl->sim_dt);
        sl->sim_accum -= sl->sim_dt;
        steps++;
    }
    if (sl->sim_accum > sl->sim_dt)
        sl->sim_accum = sl->sim_dt;
    return steps;
}

/* ── HUD ──────────────────────────────────────────────────────────────── */

void draw_hud_fps(ivp_renderer_t *r, const SimLoop *sl)
{
    char buf[64];
    std::sprintf(buf, "FPS: %.0f", sl->fps_display);
    const float col[3] = {0.8f, 0.8f, 0.8f};
    int w = 0, h = 0;
    ivp_renderer_get_size(r, &w, &h);
    ivp_draw_text_2d(r, (float)(w - 120), 10.0f, buf, col);
}

void draw_hud_object_info(ivp_renderer_t *r, float x, float y,
                          const char *label, IVP_Real_Object *obj)
{
    IVP_Core *core = obj->get_core();
    const IVP_U_Point *pos = core->get_position_PSI();
    float spd = speed_magnitude(&core->speed);

    char buf[256];
    std::sprintf(buf, "%s  pos:(%.1f, %.1f, %.1f)  vel:%.2f",
                 label,
                 (double)pos->k[0], (double)pos->k[1], (double)pos->k[2],
                 (double)spd);
    const float col[3] = {0.7f, 0.9f, 0.7f};
    ivp_draw_text_2d(r, x, y, buf, col);
}

/* ── Keyboard helpers ─────────────────────────────────────────────────── */

bool key_pressed(int scancode)
{
    const bool *keys = SDL_GetKeyboardState(NULL);
    return keys[scancode];
}

bool key_just_pressed(int scancode, bool *prev_state)
{
    bool cur = key_pressed(scancode);
    bool result = cur && !*prev_state;
    *prev_state = cur;
    return result;
}

/* ── Color palette ────────────────────────────────────────────────────── */

namespace color {
    const float grid[3]       = {0.30f, 0.30f, 0.30f};
    const float title[3]      = {1.00f, 1.00f, 1.00f};
    const float help[3]       = {0.60f, 0.60f, 0.60f};
    const float info[3]       = {0.70f, 0.90f, 0.70f};
    const float object_a[3]   = {0.20f, 0.80f, 1.00f};
    const float object_b[3]   = {1.00f, 0.60f, 0.20f};
    const float static_obj[3] = {0.50f, 0.50f, 0.50f};
    const float velocity[3]   = {1.00f, 0.40f, 0.20f};
    const float spring[3]     = {0.40f, 1.00f, 0.40f};
    const float highlight[3]  = {1.00f, 1.00f, 0.30f};
    const float warning[3]    = {1.00f, 0.20f, 0.20f};
    const float water[3]      = {0.20f, 0.40f, 0.80f};
} /* namespace color */

} /* namespace ive */
