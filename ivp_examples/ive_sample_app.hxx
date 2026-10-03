/* ive_sample_app.hxx -- DRY SampleApp framework for IVP graphical samples
 *
 * Wraps renderer, camera, environment, interaction, sim loop, and
 * Nuklear GUI into a single lifecycle so each sample only contains
 * its unique scene setup, drawing, and UI controls.
 */
#ifndef IVE_SAMPLE_APP_HXX
#define IVE_SAMPLE_APP_HXX

#include "ive_sample_common.hxx"
#include "ive_sample_interaction.hxx"
#include "ive_gizmo.hxx"

/* Forward-declare Nuklear context (avoid including nuklear.h here) */
struct nk_context;

namespace ive {

/* ── Configuration ───────────────────────────────────────────────────── */

struct SampleAppConfig {
    const char *title;
    float orbit_dist, orbit_pitch, orbit_yaw;
    float target_x, target_y, target_z;
    double friction, elasticity;
    /* Optional universe manager; NULL means none */
    class IVP_Universe_Manager *universe_manager;
};

/* ── Initial state for scene reset ───────────────────────────────────── */

#define APP_MAX_SAVED_OBJECTS 64

struct SavedObjectState {
    IVP_Real_Object *obj;
    IVP_U_Point      pos;
    IVP_U_Quat       rot;
};

/* ── Application state ───────────────────────────────────────────────── */

struct SampleApp {
    explicit SampleApp(const SampleAppConfig &cfg);
    ivp_renderer_t       *renderer;
    ivp_camera_t          cam;
    ivp_camera_t          initial_cam;
    IVP_Environment      *env;
    IVP_Material_Simple   mat;
    Dragger               dragger;
    FocusController       focus;
    Gizmo                 gizmo;
    SimLoop               loop;
    IVP_Sample_Pick_List  pick_list;
    struct nk_context    *nk;

    int   win_w, win_h;
    float dt;
    bool  ui_hovered;
    float sim_speed;
    bool  paused;

    /* Scene reset */
    SavedObjectState saved[APP_MAX_SAVED_OBJECTS];
    int              saved_count;
    bool             reset_requested;

    /* Ground object for visual overlay (optional, set via app_set_ground) */
    IVP_Real_Object *ground_obj;
    float            ground_hx, ground_hy, ground_hz;
};

/* ── Lifecycle ───────────────────────────────────────────────────────── */

void       app_config_defaults(SampleAppConfig *cfg);
SampleApp *app_create(const SampleAppConfig *cfg);
bool       app_begin_frame(SampleApp *app);
void       app_step(SampleApp *app);
void       app_add_pick(SampleApp *app, IVP_Real_Object *obj);
void       app_clear_picks(SampleApp *app);
/* Invalidate interaction/reset references before deleting a scene object. */
void       app_delete_object(SampleApp *app, IVP_Real_Object *obj);
void       app_set_ground(SampleApp *app, IVP_Real_Object *obj,
                          float hx, float hy, float hz);
void       app_draw_grid(SampleApp *app);
void       app_draw_overlays(SampleApp *app);
void       app_end_frame(SampleApp *app);
void       app_destroy(SampleApp *app);

/* ── Scene reset ─────────────────────────────────────────────────────── */

/* Call after scene setup to snapshot all pickable objects' positions.  */
void app_save_initial_state(SampleApp *app);

/* Call at the start of the frame loop. Returns true if a reset occurred. */
bool app_check_reset(SampleApp *app);

/* ── Nuklear panel helpers ───────────────────────────────────────────── */

bool app_begin_info_panel(SampleApp *app, const char *title, float w, float h);
void app_end_info_panel(SampleApp *app);
void app_nk_fps(SampleApp *app);
void app_nk_sim_speed(SampleApp *app);
void app_nk_object_info(SampleApp *app, const char *label, IVP_Real_Object *obj);
void app_nk_help_section(SampleApp *app, const char *help_text);
void app_nk_label(SampleApp *app, const char *text);
void app_nk_spacing(SampleApp *app);

/* Standard controls panel section: shows reset button, single-step,
 * and built-in keyboard/mouse help. */
void app_nk_controls(SampleApp *app, const char *extra_help = NULL);

} /* namespace ive */

#endif /* IVE_SAMPLE_APP_HXX */
