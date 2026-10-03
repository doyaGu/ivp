/* ive_gizmo.hxx -- Translation/rotation gizmo for IVP sample objects */
#ifndef IVE_GIZMO_HXX
#define IVE_GIZMO_HXX

#include "ive_sample_common.hxx"
struct nk_context;

namespace ive {

enum GizmoMode  { GIZMO_NONE, GIZMO_TRANSLATE, GIZMO_ROTATE };
enum GizmoAxis  { AXIS_NONE, AXIS_X, AXIS_Y, AXIS_Z };

/* Axis color constants (X=red, Y=green, Z=blue) */
namespace gizmo_color {
    extern const float axis_x[3];
    extern const float axis_y[3];
    extern const float axis_z[3];
    extern const float axis_hover[3];
    extern const float axis_active[3];
} /* namespace gizmo_color */

struct Gizmo {
    GizmoMode mode;
    GizmoAxis hover_axis;
    GizmoAxis active_axis;
    bool      dragging;

    /* Drag state */
    float drag_origin[3];
    float drag_start_angle;
    float drag_axis_param;

    /* Mouse button edge detection */
    bool  prev_lmb;
    bool  lmb_seeded;

    /* Settings */
    float screen_size;
    float pick_threshold;
};

void gizmo_init(Gizmo *g);
void gizmo_set_mode(Gizmo *g, GizmoMode mode);

/* Per-frame update. target = focused object (NULL to hide gizmo).
 * Returns true if gizmo consumed the mouse input this frame. */
bool gizmo_update(Gizmo *g, IVP_Real_Object *target,
                  const ivp_camera_t *cam,
                  ivp_renderer_t *r, int win_w, int win_h);

/* Draw 3D gizmo handles (arrows/circles) via the renderer. */
void gizmo_draw_3d(const Gizmo *g, IVP_Real_Object *target,
                   const ivp_camera_t *cam, ivp_renderer_t *r);

/* Draw 2D labels/readouts via Nuklear overlay. */
void gizmo_draw_overlay(const Gizmo *g, IVP_Real_Object *target,
                        struct nk_context *nk,
                        ivp_renderer_t *r, int win_w, int win_h);

bool gizmo_is_active(const Gizmo *g);

} /* namespace ive */

#endif /* IVE_GIZMO_HXX */
