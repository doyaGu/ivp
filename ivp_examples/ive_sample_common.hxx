/* ive_sample_common.hxx -- Shared C++ utilities for IVP graphical samples
 *
 * Bridges legacy IVP C++ types to the renderer (render.h).
 */
#ifndef IVE_SAMPLE_COMMON_HXX
#define IVE_SAMPLE_COMMON_HXX

#include "render.h"

#include <ivp_physics.hxx>
#include <ivp_controller_factory.hxx>
#include <ivp_material.hxx>
#include <ivp_templates.hxx>
#include <ivp_ball.hxx>
#include <ivp_polygon.hxx>
#include <ivp_core.hxx>
#include <ivp_surman_polygon.hxx>
#include <ivp_surbuild_pointsoup.hxx>
#include <ivp_surbuild_ledge_soup.hxx>

namespace ive {

/* ── Type conversions ─────────────────────────────────────────────────── */

void quat_to_col_rot(const IVP_U_Quat *q, float rot[9]);
void point_to_float3(const IVP_U_Point *p, float out[3]);
void float_point_to_float3(const IVP_U_Float_Point *p, float out[3]);
float speed_magnitude(const IVP_U_Float_Point *v);

/* ── Inertia helpers ──────────────────────────────────────────────────── */

void set_box_inertia(IVP_Template_Real_Object *t, double mass,
                     double hx, double hy, double hz);
void set_sphere_inertia(IVP_Template_Real_Object *t, double mass,
                        double radius);

/* ── Environment ──────────────────────────────────────────────────────── */

IVP_Environment *create_environment(double dt = 1.0 / 60.0,
                                    class IVP_Universe_Manager *um = 0);

/* ── Geometry builders ────────────────────────────────────────────────── */

IVP_Compact_Surface *build_box_surface(double hx, double hy, double hz);

/* ── Object creation ──────────────────────────────────────────────────── */

void configure_dynamic_template(IVP_Template_Real_Object *t,
                                IVP_Material *mat, double mass);
void configure_static_template(IVP_Template_Real_Object *t,
                               IVP_Material *mat);

IVP_Polygon *create_box(IVP_Environment *env, IVP_Material *mat,
                         double hx, double hy, double hz, double mass,
                         const IVP_U_Quat *q, const IVP_U_Point *pos);

IVP_Ball *create_ball(IVP_Environment *env, IVP_Material *mat,
                      double radius, double mass,
                      const IVP_U_Quat *q, const IVP_U_Point *pos);

void wake_and_enable(IVP_Real_Object *obj);
void set_quat_axis_angle(IVP_U_Quat *q, double ax, double ay, double az,
                         double radians);

/* ── Screen ray ──────────────────────────────────────────────────────── */

/* Compute a world-space ray from a screen pixel position. */
bool screen_ray(const ivp_camera_t *cam, int w, int h,
                float mx, float my, float ro[3], float rd[3]);

/* ── Drawing helpers ──────────────────────────────────────────────────── */

/* Tessellation quality constants */
static const int IVE_SPHERE_SEGS   = 32;
static const int IVE_CYLINDER_SEGS = 24;

void draw_object_box(ivp_renderer_t *r, IVP_Real_Object *obj,
                     double hx, double hy, double hz, const float color[3]);
void draw_object_ball(ivp_renderer_t *r, IVP_Real_Object *obj,
                      double radius, const float color[3]);
void draw_object_cylinder(ivp_renderer_t *r, IVP_Real_Object *obj,
                           double radius, double half_height,
                           const float color[3]);
void draw_velocity_arrow(ivp_renderer_t *r, IVP_Real_Object *obj,
                         const float color[3]);
void draw_spring_line(ivp_renderer_t *r, IVP_Real_Object *a,
                      IVP_Real_Object *b, const float color[3]);
void draw_anchor_line(ivp_renderer_t *r, IVP_Real_Object *a,
                      double ax, double ay, double az,
                      IVP_Real_Object *b,
                      double bx, double by, double bz,
                      const float color[3]);

/* ── Fixed-timestep simulation loop ───────────────────────────────────── */

struct SimLoop {
    double t_prev;
    float sim_accum;
    float sim_dt;
    int max_substeps;
    int frame_count;
    float fps_timer;
    float fps_display;
};

void sim_loop_init(SimLoop *sl, const ivp_renderer_t *r);
float sim_loop_begin(SimLoop *sl, const ivp_renderer_t *r);
int sim_loop_step(SimLoop *sl, IVP_Environment *env, float frame_dt);

/* ── HUD formatting ───────────────────────────────────────────────────── */

void draw_hud_fps(ivp_renderer_t *r, const SimLoop *sl);
void draw_hud_object_info(ivp_renderer_t *r, float x, float y,
                          const char *label, IVP_Real_Object *obj);

/* ── Keyboard state query (via SDL) ───────────────────────────────────── */

bool key_pressed(int scancode);
bool key_just_pressed(int scancode, bool *prev_state);

/* ── Centralized color palette ────────────────────────────────────────── */

namespace color {
    extern const float grid[3];
    extern const float title[3];
    extern const float help[3];
    extern const float info[3];
    extern const float object_a[3];   /* primary dynamic (blue)   */
    extern const float object_b[3];   /* secondary dynamic (orange) */
    extern const float static_obj[3]; /* static/ground (gray)     */
    extern const float velocity[3];
    extern const float spring[3];
    extern const float highlight[3];
    extern const float warning[3];
    extern const float water[3];
} /* namespace color */

} /* namespace ive */

#endif /* IVE_SAMPLE_COMMON_HXX */
