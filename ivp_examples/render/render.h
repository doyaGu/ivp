/* render.h -- OpenGL 3.3 wireframe rendering for IVP examples
 *
 * Rendering interface implemented in render.c using glad + SDL3.
 * Provides:
 *   - Window creation and GL context setup
 *   - Camera (orbit around target, WASD pan, scroll zoom)
 *   - Wireframe cube, sphere, cylinder, cone, arrow, line, grid
 *   - 2D HUD text overlay (bitmap font)
 *   - Simple per-frame loop helpers
 */
#ifndef IVP_EXAMPLE_RENDER_H
#define IVP_EXAMPLE_RENDER_H

#include <stdbool.h>
#include <SDL3/SDL.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Types ──────────────────────────────────────────────────────────────── */

typedef struct ivp_renderer ivp_renderer_t;

typedef struct ivp_camera {
    float eye[3];
    float target[3];
    float up[3];
    float fov_deg;
    float near_plane;
    float far_plane;
    /* orbit controls */
    float orbit_distance;
    float orbit_yaw;
    float orbit_pitch;
} ivp_camera_t;

typedef struct ivp_render_config {
    int   width;
    int   height;
    const char *title;
    bool  vsync;
} ivp_render_config_t;

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

ivp_renderer_t *ivp_renderer_create(const ivp_render_config_t *config);
void ivp_renderer_destroy(ivp_renderer_t *r);

/* Returns false when user closes window */
bool ivp_renderer_begin_frame(ivp_renderer_t *r, ivp_camera_t *cam);
void ivp_renderer_end_frame(ivp_renderer_t *r);

/* ── Camera helpers ─────────────────────────────────────────────────────── */

void ivp_camera_init(ivp_camera_t *cam);
void ivp_camera_update_orbit(ivp_camera_t *cam);

/* ── Drawing primitives ─────────────────────────────────────────────────── */

/* Draw a wireframe box centered at pos with half-extents half_size.
 * rot is a 3x3 column-major rotation matrix (9 floats), NULL = identity. */
void ivp_draw_wire_box(ivp_renderer_t *r,
                         const float pos[3],
                         const float half_size[3],
                         const float *rot,     /* 9 floats column-major, or NULL */
                         const float color[3]);

/* Draw a wireframe sphere at pos with given radius. */
void ivp_draw_wire_sphere(ivp_renderer_t *r,
                            const float pos[3],
                            float radius,
                            const float color[3]);

/* Draw a wireframe cylinder from base to base+axis*height with given radius. */
void ivp_draw_wire_sphere_ex(ivp_renderer_t *r, const float pos[3],
                             float radius, const float color[3], int segments);

void ivp_draw_wire_cylinder(ivp_renderer_t *r,
                              const float base[3],
                              const float axis[3],  /* unit direction */
                              float height,
                              float radius,
                              int segments,
                              const float color[3]);

/* Draw a colored line segment from a to b. */
void ivp_draw_line(ivp_renderer_t *r,
                     const float a[3],
                     const float b[3],
                     const float color[3]);

/* Draw a thick line (multiple parallel lines for visibility). */
void ivp_draw_thick_line(ivp_renderer_t *r,
                           const float a[3],
                           const float b[3],
                           const float color[3],
                           float thickness);

/* Draw an arrow from a to b with arrowhead. */
void ivp_draw_arrow(ivp_renderer_t *r,
                      const float from[3],
                      const float to[3],
                      const float color[3],
                      float head_size);

/* Draw a ground plane grid centered at origin on the XZ plane. */
void ivp_draw_ground_grid(ivp_renderer_t *r,
                            float extent,
                            float spacing,
                            const float color[3]);

/* Draw a filled quad (two triangles) as wireframe (for water etc.) */
void ivp_draw_wire_quad(ivp_renderer_t *r,
                          const float corners[4][3],
                          const float color[3]);

/* Draw coordinate axes at a position with given scale. */
void ivp_draw_axes(ivp_renderer_t *r,
                     const float pos[3],
                     const float *rot,  /* 9 floats column-major, or NULL */
                     float scale);

/* ── 2D HUD text ────────────────────────────────────────────────────────── */

/* Draw text at 2D screen position (pixel coords, top-left origin).
 * Uses a minimal built-in bitmap font. */
void ivp_draw_text_2d(ivp_renderer_t *r,
                        float x, float y,
                        const char *text,
                        const float color[3]);

/* ── Time query ────────────────────────────────────────────────────────── */

/* Elapsed time in seconds since renderer creation */
double ivp_renderer_get_time(const ivp_renderer_t *r);

/* Frame delta time from last begin_frame call */
float ivp_renderer_get_dt(const ivp_renderer_t *r);

/* Window dimensions */
void ivp_renderer_get_size(const ivp_renderer_t *r, int *w, int *h);

/* Set background clear color. Alpha is currently for completeness. */
void ivp_renderer_set_clear_color(ivp_renderer_t *r,
                                    float red,
                                    float green,
                                    float blue,
                                    float alpha);

/* ── Event callbacks ───────────────────────────────────────────────────── */

/* Per-event callback: return true to consume the event (skip camera). */
typedef bool (*ivp_event_callback_t)(const SDL_Event *ev, void *userdata);
/* Frame-phase callback: called at specific points in the frame. */
typedef void (*ivp_frame_callback_t)(void *userdata);

void ivp_renderer_set_event_callback(ivp_renderer_t *r, ivp_event_callback_t cb, void *userdata);
void ivp_renderer_set_pre_events_callback(ivp_renderer_t *r, ivp_frame_callback_t cb, void *userdata);
void ivp_renderer_set_post_events_callback(ivp_renderer_t *r, ivp_frame_callback_t cb, void *userdata);
void ivp_renderer_set_pre_swap_callback(ivp_renderer_t *r, ivp_frame_callback_t cb, void *userdata);

/* Project a 3D world point to 2D screen coordinates (pixel space, top-left origin).
 * Returns false if the point is behind the camera. */
bool ivp_renderer_project(const ivp_renderer_t *r,
                          const float world[3], float screen[2]);

/* Access the combined view-projection matrix (16 floats, column-major). */
const float *ivp_renderer_get_vp_matrix(const ivp_renderer_t *r);

/* Access the view matrix (16 floats, column-major). */
const float *ivp_renderer_get_view_matrix(const ivp_renderer_t *r);

/* Access the underlying SDL window (e.g. for Nuklear init). */
SDL_Window *ivp_renderer_get_window(const ivp_renderer_t *r);

#ifdef __cplusplus
}
#endif

#endif /* IVP_EXAMPLE_RENDER_H */
