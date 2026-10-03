/* ive_sample_app.cxx -- SampleApp framework implementation */

#include "ive_sample_app.hxx"
#include <ivp_cache_object.hxx>

/* Nuklear defines needed before including the header */
#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_UINT_DRAW_INDEX
#include "nuklear.h"
#include "nuklear_sdl3_gl3.h"


#include <new>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace ive {

/* ── Renderer callbacks (C linkage for function pointers) ────────────── */

static bool nk_event_cb(const SDL_Event *ev, void *ud)
{
    SampleApp *app = (SampleApp *)ud;

    /* Gate scroll events: only feed to NK when mouse hovers a panel */
    if (ev->type == SDL_EVENT_MOUSE_WHEEL) {
        if (nk_window_is_any_hovered(app->nk)) {
            nk_sdl_handle_event(const_cast<SDL_Event *>(ev));
            return true;   /* consumed by UI */
        }
        return false;      /* let renderer handle camera zoom */
    }

    /* Feed all other events to Nuklear */
    nk_sdl_handle_event(const_cast<SDL_Event *>(ev));

    /* Consume mouse events when hovering over a Nuklear window */
    if (app->ui_hovered) {
        switch (ev->type) {
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_UP:
        case SDL_EVENT_MOUSE_MOTION:
            return true; /* consumed */
        default:
            break;
        }
    }
    /* Consume keyboard events when a Nuklear widget is active */
    if (nk_item_is_any_active(app->nk)) {
        switch (ev->type) {
        case SDL_EVENT_KEY_DOWN:
        case SDL_EVENT_KEY_UP:
        case SDL_EVENT_TEXT_INPUT:
            return true;
        default:
            break;
        }
    }
    return false;
}

static void nk_pre_events_cb(void *ud)
{
    SampleApp *app = (SampleApp *)ud;
    nk_input_begin(app->nk);
}

static void nk_post_events_cb(void *ud)
{
    SampleApp *app = (SampleApp *)ud;
    nk_input_end(app->nk);
    app->ui_hovered = nk_window_is_any_hovered(app->nk) ? true : false;
}

static void nk_pre_swap_cb(void *ud)
{
    (void)ud;
    nk_sdl_render(NK_ANTI_ALIASING_ON, 512 * 1024, 128 * 1024);
}

/* ── Configuration defaults ──────────────────────────────────────────── */

void app_config_defaults(SampleAppConfig *cfg)
{
    std::memset(cfg, 0, sizeof(*cfg));
    cfg->title       = "IVP Sample";
    cfg->orbit_dist  = 20.0f;
    cfg->orbit_pitch = -30.0f;
    cfg->orbit_yaw   = 45.0f;
    cfg->target_x    = 0.0f;
    cfg->target_y    = -3.0f;
    cfg->target_z    = 0.0f;
    cfg->friction    = 0.7;
    cfg->elasticity  = 0.6;
    cfg->universe_manager = 0;
}

/* ── Create ──────────────────────────────────────────────────────────── */

SampleApp::SampleApp(const SampleAppConfig &cfg)
    : renderer(NULL), cam(), initial_cam(), env(NULL),
      mat(cfg.friction, cfg.elasticity), dragger(), focus(), gizmo(),
      loop(), pick_list(), nk(NULL), win_w(0), win_h(0), dt(0.0f),
      ui_hovered(false), sim_speed(1.0f), paused(false), saved(),
      saved_count(0), reset_requested(false), ground_obj(NULL),
      ground_hx(0.0f), ground_hy(0.0f), ground_hz(0.0f)
{
    ivp_camera_init(&cam);
    initial_cam = cam;
    dragger_init(&dragger);
    focus_init(&focus);
    gizmo_init(&gizmo);
}

SampleApp *app_create(const SampleAppConfig *cfg)
{
    if (!cfg) return NULL;
    SampleApp *app = new (std::nothrow) SampleApp(*cfg);
    if (!app) return NULL;

    /* Renderer */
    ivp_render_config_t rcfg;
    rcfg.width  = 1280;
    rcfg.height = 720;
    rcfg.title  = cfg->title;
    rcfg.vsync  = true;
    app->renderer = ivp_renderer_create(&rcfg);
    if (!app->renderer) { delete app; return NULL; }

    /* Camera */
    ivp_camera_init(&app->cam);
    app->cam.orbit_distance = cfg->orbit_dist;
    app->cam.orbit_pitch    = cfg->orbit_pitch;
    app->cam.orbit_yaw      = cfg->orbit_yaw;
    app->cam.target[0]      = cfg->target_x;
    app->cam.target[1]      = cfg->target_y;
    app->cam.target[2]      = cfg->target_z;

    /* Save initial camera for reset */
    std::memcpy(&app->initial_cam, &app->cam, sizeof(ivp_camera_t));

    /* Physics */
    app->env = create_environment(1.0 / 60.0, cfg->universe_manager);
    app->pick_list.count = 0;
    app->env->client_data = &app->pick_list;

    /* Interaction */
    dragger_init(&app->dragger);
    focus_init(&app->focus);
    gizmo_init(&app->gizmo);

    /* Sim loop */
    sim_loop_init(&app->loop, app->renderer);

    /* Nuklear */
    SDL_Window *win = ivp_renderer_get_window(app->renderer);
    app->nk = nk_sdl_init(win);

    {
        struct nk_font_atlas *atlas;
        nk_sdl_font_stash_begin(&atlas);
        struct nk_font *font = nk_font_atlas_add_default(atlas, 18.0f, NULL);
        nk_sdl_font_stash_end();
        nk_style_set_font(app->nk, &font->handle);
    }

    /* Style: semi-transparent dark background for panels */
    {
        struct nk_color bg = nk_rgba(25, 30, 38, 210);
        app->nk->style.window.fixed_background = nk_style_item_color(bg);
        app->nk->style.window.background = bg;
    }

    /* Hook callbacks */
    ivp_renderer_set_event_callback(app->renderer, nk_event_cb, app);
    ivp_renderer_set_pre_events_callback(app->renderer, nk_pre_events_cb, app);
    ivp_renderer_set_post_events_callback(app->renderer, nk_post_events_cb, app);
    ivp_renderer_set_pre_swap_callback(app->renderer, nk_pre_swap_cb, app);

    return app;
}

/* ── Frame begin ─────────────────────────────────────────────────────── */

bool app_begin_frame(SampleApp *app)
{
    if (!ivp_renderer_begin_frame(app->renderer, &app->cam))
        return false;
    ivp_renderer_get_size(app->renderer, &app->win_w, &app->win_h);
    app->dt = sim_loop_begin(&app->loop, app->renderer);
    return true;
}

/* ── Step simulation + interaction ───────────────────────────────────── */

void app_step(SampleApp *app)
{
    float step_dt = app->paused ? 0.0f : app->dt * app->sim_speed;
    sim_loop_step(&app->loop, app->env, step_dt);

    /* Esc: cancel gizmo first, then clear focus */
    static bool prev_esc = false;
    if (key_just_pressed(SDL_SCANCODE_ESCAPE, &prev_esc)) {
        if (app->gizmo.mode != GIZMO_NONE) {
            gizmo_set_mode(&app->gizmo, GIZMO_NONE);
        } else if (app->focus.focused) {
            app->focus.focused = NULL;
        }
    }

    /* G key: cycle gizmo mode */
    static bool prev_g = false;
    if (key_just_pressed(SDL_SCANCODE_G, &prev_g)) {
        GizmoMode next = (GizmoMode)((app->gizmo.mode + 1) % 3);
        gizmo_set_mode(&app->gizmo, next);

        /* Auto-focus first pickable object if nothing is focused */
        if (next != GIZMO_NONE && !app->focus.focused) {
            for (int i = 0; i < app->pick_list.count; i++) {
                IVP_Real_Object *obj = app->pick_list.objs[i];
                if (obj && obj->get_core() && !obj->get_core()->physical_unmoveable) {
                    app->focus.focused = obj;
                    break;
                }
            }
        }
    }

    /* Update gizmo (uses focused object from FocusController) */
    bool gizmo_consumed = gizmo_update(&app->gizmo, app->focus.focused,
                                        &app->cam, app->renderer,
                                        app->win_w, app->win_h);

    /* Only update dragger if gizmo didn't consume input AND UI isn't hovered */
    if (!gizmo_consumed && !app->ui_hovered)
        dragger_update(&app->dragger, &app->cam, app->env,
                       app->win_w, app->win_h, app->dt);
    focus_update(&app->focus, &app->cam, app->env, app->dt);
}

/* ── Pick list management ────────────────────────────────────────────── */

void app_add_pick(SampleApp *app, IVP_Real_Object *obj)
{
    if (app->pick_list.count < 64)
        app->pick_list.objs[app->pick_list.count++] = obj;
}

void app_clear_picks(SampleApp *app)
{
    app->pick_list.count = 0;
}

void app_delete_object(SampleApp *app, IVP_Real_Object *obj)
{
    if (!app || !obj) return;
    for (int i = 0; i < app->pick_list.count;) {
        if (app->pick_list.objs[i] != obj) { ++i; continue; }
        for (int j = i; j < app->pick_list.count - 1; ++j)
            app->pick_list.objs[j] = app->pick_list.objs[j + 1];
        app->pick_list.objs[--app->pick_list.count] = NULL;
    }
    for (int i = 0; i < app->saved_count;) {
        if (app->saved[i].obj != obj) { ++i; continue; }
        for (int j = i; j < app->saved_count - 1; ++j)
            app->saved[j] = app->saved[j + 1];
        app->saved[--app->saved_count].obj = NULL;
    }
    if (app->focus.focused == obj) {
        app->focus.focused = NULL;
        gizmo_init(&app->gizmo);
    }
    if (app->dragger.target == obj) {
        app->dragger.active = false;
        app->dragger.target = NULL;
        app->dragger.has_target_point = false;
    }
    if (app->ground_obj == obj) app->ground_obj = NULL;
    obj->delete_and_check_vicinity();
}

/* ── Scene reset ─────────────────────────────────────────────────────── */

void app_save_initial_state(SampleApp *app)
{
    app->saved_count = 0;
    for (int i = 0; i < app->pick_list.count && i < APP_MAX_SAVED_OBJECTS; i++) {
        IVP_Real_Object *obj = app->pick_list.objs[i];
        if (!obj) continue;
        IVP_Core *core = obj->get_core();
        if (!core) continue;

        SavedObjectState *s = &app->saved[app->saved_count++];
        s->obj = obj;

        /* Read current world position and rotation via cache */
        IVP_Cache_Object *cache = obj->get_cache_object();
        s->pos.set(cache->m_world_f_object.get_position());
        s->rot = cache->q_world_f_object;
        cache->remove_reference();
    }
}

static void reset_objects(SampleApp *app)
{
    for (int i = 0; i < app->saved_count; i++) {
        SavedObjectState *s = &app->saved[i];
        if (!s->obj) continue;
        IVP_Core *core = s->obj->get_core();
        if (!core) continue;

        /* Teleport to initial position/rotation */
        s->obj->beam_object_to_new_position(
            &s->rot, &s->pos,
            (i < app->saved_count - 1) ? IVP_TRUE : IVP_FALSE);

        /* Zero all velocities */
        core->speed.set(0.0f, 0.0f, 0.0f);
        core->rot_speed.set(0.0f, 0.0f, 0.0f);

        /* Wake up in case frozen */
        wake_and_enable(s->obj);
    }

    /* Reset camera */
    std::memcpy(&app->cam, &app->initial_cam, sizeof(ivp_camera_t));
    focus_init(&app->focus);
    dragger_init(&app->dragger);
    gizmo_init(&app->gizmo);
}

bool app_check_reset(SampleApp *app)
{
    /* Backspace key triggers reset */
    static bool prev_backspace = false;
    const bool *keys = SDL_GetKeyboardState(NULL);
    bool backspace = keys[SDL_SCANCODE_BACKSPACE];
    bool just_pressed = backspace && !prev_backspace;
    prev_backspace = backspace;

    if (just_pressed)
        app->reset_requested = true;

    if (app->reset_requested) {
        app->reset_requested = false;
        reset_objects(app);
        return true;
    }
    return false;
}

/* ── Drawing helpers ─────────────────────────────────────────────────── */

void app_set_ground(SampleApp *app, IVP_Real_Object *obj,
                    float hx, float hy, float hz)
{
    app->ground_obj = obj;
    app->ground_hx  = hx;
    app->ground_hy  = hy;
    app->ground_hz  = hz;
}

void app_draw_grid(SampleApp *app)
{
    ivp_draw_ground_grid(app->renderer, 30.0f, 1.0f, color::grid);
    if (app->ground_obj) {
        draw_object_box(app->renderer, app->ground_obj,
                        app->ground_hx, app->ground_hy, app->ground_hz,
                        color::static_obj);
    }
}

void app_draw_overlays(SampleApp *app)
{
    /* Draw 3D gizmo handles */
    gizmo_draw_3d(&app->gizmo, app->focus.focused, &app->cam, app->renderer);

    /* Draw gizmo mode HUD in bottom-center when active */
    if (app->gizmo.mode != GIZMO_NONE) {
        const char *mode_str = (app->gizmo.mode == GIZMO_TRANSLATE) ?
            "GIZMO: TRANSLATE" : "GIZMO: ROTATE";
        const float mode_col[3] = {1.0f, 0.9f, 0.3f};
        float tx = (float)(app->win_w / 2) - 90.0f;
        float ty = (float)(app->win_h) - 50.0f;
        ivp_draw_text_2d(app->renderer, tx, ty, mode_str, mode_col);

        if (!app->focus.focused) {
            const float hint_col[3] = {0.6f, 0.6f, 0.6f};
            ivp_draw_text_2d(app->renderer, tx, ty + 18.0f,
                             "Tab to select object", hint_col);
        }
    }

    draw_interaction_overlays(app->renderer, &app->dragger, &app->focus);
}

/* ── Tools panel (framework-managed, right side) ─────────────────────── */

static void draw_tools_panel(SampleApp *app)
{
    float panel_w = 170.0f;
    float panel_h = 260.0f;
    float panel_x = (float)app->win_w - panel_w - 10.0f;
    float panel_y = 130.0f; /* below camera gizmo */

    nk_flags flags = NK_WINDOW_BORDER | NK_WINDOW_TITLE |
                     NK_WINDOW_MOVABLE | NK_WINDOW_NO_SCROLLBAR;
    if (nk_begin(app->nk, "Tools", nk_rect(panel_x, panel_y, panel_w, panel_h), flags)) {

        /* Gizmo mode controls */
        nk_layout_row_dynamic(app->nk, 22, 1);
        {
            const char *mode_names[] = {"Off", "Translate", "Rotate"};
            char label[64];
            std::sprintf(label, "Gizmo: %s", mode_names[app->gizmo.mode]);
            nk_label(app->nk, label, NK_TEXT_LEFT);
        }
        nk_layout_row_dynamic(app->nk, 26, 3);
        GizmoMode new_gm = app->gizmo.mode;
        if (nk_button_label(app->nk, "Off"))       new_gm = GIZMO_NONE;
        if (nk_button_label(app->nk, "Translate"))  new_gm = GIZMO_TRANSLATE;
        if (nk_button_label(app->nk, "Rotate"))     new_gm = GIZMO_ROTATE;
        if (new_gm != app->gizmo.mode) {
            gizmo_set_mode(&app->gizmo, new_gm);
            /* Auto-focus first pickable object if nothing is focused */
            if (new_gm != GIZMO_NONE && !app->focus.focused) {
                for (int i = 0; i < app->pick_list.count; i++) {
                    IVP_Real_Object *obj = app->pick_list.objs[i];
                    if (obj && obj->get_core() &&
                        !obj->get_core()->physical_unmoveable) {
                        app->focus.focused = obj;
                        break;
                    }
                }
            }
        }

        /* Focused object info */
        if (app->focus.focused) {
            nk_layout_row_dynamic(app->nk, 8, 1);
            nk_spacing(app->nk, 1);
            IVP_Core *core = app->focus.focused->get_core();
            if (core) {
                const IVP_U_Point *pos = core->get_position_PSI();
                char buf[128];
                const char *name = app->focus.focused->get_name();
                std::sprintf(buf, "Focus: %s", name ? name : "object");
                nk_layout_row_dynamic(app->nk, 20, 1);
                nk_label(app->nk, buf, NK_TEXT_LEFT);
                std::sprintf(buf, "(%.1f, %.1f, %.1f)",
                             (double)pos->k[0], (double)pos->k[1], (double)pos->k[2]);
                nk_layout_row_dynamic(app->nk, 18, 1);
                nk_label(app->nk, buf, NK_TEXT_LEFT);
            }
        }

        /* Sim controls */
        nk_layout_row_dynamic(app->nk, 8, 1);
        nk_spacing(app->nk, 1);
        nk_layout_row_dynamic(app->nk, 26, 2);
        if (nk_button_label(app->nk, app->paused ? "Resume" : "Pause"))
            app->paused = !app->paused;
        if (nk_button_label(app->nk, "Step")) {
            app->paused = true;
            float step_dt = app->dt * app->sim_speed;
            if (step_dt < 0.001f) step_dt = 1.0f / 60.0f;
            sim_loop_step(&app->loop, app->env, step_dt);
        }
        nk_layout_row_dynamic(app->nk, 26, 1);
        if (nk_button_label(app->nk, "Reset Scene"))
            app->reset_requested = true;
    }
    nk_end(app->nk);
}

/* ── Camera orientation gizmo (top-right corner) ─────────────────────── */

static void draw_camera_gizmo(SampleApp *app)
{
    float gizmo_size = 110.0f;
    float gizmo_x = (float)app->win_w - gizmo_size - 10.0f;
    float gizmo_y = 10.0f;

    nk_flags flags = NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_BORDER |
                     NK_WINDOW_NO_INPUT;
    if (nk_begin(app->nk, "##CamGizmo",
                 nk_rect(gizmo_x, gizmo_y, gizmo_size, gizmo_size), flags)) {

        struct nk_command_buffer *canvas = nk_window_get_canvas(app->nk);
        struct nk_rect bounds = nk_window_get_content_region(app->nk);
        float cx = bounds.x + bounds.w * 0.5f;
        float cy = bounds.y + bounds.h * 0.5f;
        float arm = 35.0f;

        /* Dark semi-transparent background circle */
        nk_fill_circle(canvas,
            nk_rect(cx - arm - 6, cy - arm - 6,
                    (arm + 6) * 2, (arm + 6) * 2),
            nk_rgba(20, 24, 30, 180));

        /* Get view matrix rotation (3x3 from column-major 4x4) */
        const float *v = ivp_renderer_get_view_matrix(app->renderer);

        /* World axes transformed to view space:
         * X-axis direction in view space: (v[0], v[4], v[8]) — row 0 of 4x4
         * Y-axis direction in view space: (v[1], v[5], v[9]) — row 1
         * Z-axis direction in view space: (v[2], v[6], v[10]) — row 2
         *
         * For lookat matrix (column-major):
         *   row 0 = right (s),  row 1 = up (u),  row 2 = -forward (-f)
         * We want to project: use (view_x, -view_y) as 2D offset (negate Y for screen-down)
         */
        struct { float vx, vy, vz; struct nk_color col; const char *label; } axes[3] = {
            { v[0], v[4], v[8],  nk_rgb(220, 60, 60),   "X" },  /* red */
            { v[1], v[5], v[9],  nk_rgb(60, 200, 60),   "Y" },  /* green */
            { v[2], v[6], v[10], nk_rgb(60, 100, 220),  "Z" },  /* blue */
        };

        /* Depth sort: draw back-most axes first (painter's algorithm) */
        int order[3] = {0, 1, 2};
        for (int i = 0; i < 2; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (axes[order[i]].vz > axes[order[j]].vz) {
                    int tmp = order[i]; order[i] = order[j]; order[j] = tmp;
                }
            }
        }

        for (int idx = 0; idx < 3; idx++) {
            int a = order[idx];
            float tx = cx + axes[a].vx * arm;
            float ty = cy - axes[a].vy * arm; /* negate Y for screen coords */

            /* Axis line */
            nk_stroke_line(canvas, cx, cy, tx, ty, 2.0f, axes[a].col);

            /* Tip circle */
            float tip_r = 8.0f;
            nk_fill_circle(canvas,
                nk_rect(tx - tip_r, ty - tip_r, tip_r * 2, tip_r * 2),
                axes[a].col);

            /* Letter label (offset slightly for centering) */
            struct nk_rect label_rect = nk_rect(tx - tip_r, ty - tip_r,
                                                 tip_r * 2, tip_r * 2);
            nk_draw_text(canvas, label_rect, axes[a].label, 1,
                         app->nk->style.font,
                         nk_rgba(0, 0, 0, 0), nk_rgb(255, 255, 255));
        }
    }
    nk_end(app->nk);
}

/* ── Frame end ───────────────────────────────────────────────────────── */

void app_end_frame(SampleApp *app)
{
    /* Framework-managed panels */
    draw_tools_panel(app);
    draw_camera_gizmo(app);

    /* Gizmo 2D overlay (fullscreen transparent, behind other windows) */
    gizmo_draw_overlay(&app->gizmo, app->focus.focused, app->nk,
                       app->renderer, app->win_w, app->win_h);

    ivp_renderer_end_frame(app->renderer);
}

/* ── Destroy ─────────────────────────────────────────────────────────── */

void app_destroy(SampleApp *app)
{
    if (!app) return;
    nk_sdl_shutdown();
    delete app->env;
    ivp_renderer_destroy(app->renderer);
    delete app;
}

/* ── Nuklear panel helpers ───────────────────────────────────────────── */

bool app_begin_info_panel(SampleApp *app, const char *title, float w, float h)
{
    nk_flags flags = NK_WINDOW_BORDER | NK_WINDOW_MOVABLE |
                     NK_WINDOW_SCALABLE | NK_WINDOW_TITLE |
                     NK_WINDOW_MINIMIZABLE;
    return nk_begin(app->nk, title, nk_rect(10, 10, w, h), flags) != 0;
}

void app_end_info_panel(SampleApp *app)
{
    nk_end(app->nk);
}

void app_nk_fps(SampleApp *app)
{
    char buf[64];
    std::sprintf(buf, "FPS: %.0f", app->loop.fps_display);
    nk_layout_row_dynamic(app->nk, 22, 1);
    nk_label(app->nk, buf, NK_TEXT_LEFT);
}

void app_nk_sim_speed(SampleApp *app)
{
    nk_layout_row_dynamic(app->nk, 22, 1);
    nk_label(app->nk, "Sim Speed:", NK_TEXT_LEFT);
    nk_layout_row_dynamic(app->nk, 22, 1);
    nk_slider_float(app->nk, 0.0f, &app->sim_speed, 5.0f, 0.1f);

    nk_layout_row_dynamic(app->nk, 26, 2);
    if (nk_button_label(app->nk, app->paused ? "Resume" : "Pause"))
        app->paused = !app->paused;
    if (nk_button_label(app->nk, "Reset Speed"))
        app->sim_speed = 1.0f;
}

void app_nk_object_info(SampleApp *app, const char *label, IVP_Real_Object *obj)
{
    IVP_Core *core = obj->get_core();
    const IVP_U_Point *pos = core->get_position_PSI();
    float spd = speed_magnitude(&core->speed);

    char buf[256];
    std::sprintf(buf, "%s", label);
    nk_layout_row_dynamic(app->nk, 22, 1);
    nk_label(app->nk, buf, NK_TEXT_LEFT);

    std::sprintf(buf, "Pos: (%.1f, %.1f, %.1f)",
                 (double)pos->k[0], (double)pos->k[1], (double)pos->k[2]);
    nk_layout_row_dynamic(app->nk, 20, 1);
    nk_label(app->nk, buf, NK_TEXT_LEFT);

    std::sprintf(buf, "Speed: %.2f", (double)spd);
    nk_layout_row_dynamic(app->nk, 20, 1);
    nk_label(app->nk, buf, NK_TEXT_LEFT);
}

void app_nk_help_section(SampleApp *app, const char *help_text)
{
    nk_layout_row_dynamic(app->nk, 8, 1);
    nk_spacing(app->nk, 1);
    nk_layout_row_dynamic(app->nk, 22, 1);
    nk_label(app->nk, "Controls:", NK_TEXT_LEFT);

    /* Split help_text by newlines and render each as a label */
    char line[256];
    const char *p = help_text;
    while (*p) {
        int i = 0;
        while (*p && *p != '\n' && i < 255)
            line[i++] = *p++;
        line[i] = '\0';
        if (*p == '\n') p++;
        nk_layout_row_dynamic(app->nk, 18, 1);
        nk_label(app->nk, line, NK_TEXT_LEFT);
    }
}

void app_nk_label(SampleApp *app, const char *text)
{
    nk_layout_row_dynamic(app->nk, 22, 1);
    nk_label(app->nk, text, NK_TEXT_LEFT);
}

void app_nk_spacing(SampleApp *app)
{
    nk_layout_row_dynamic(app->nk, 8, 1);
    nk_spacing(app->nk, 1);
}

void app_nk_controls(SampleApp *app, const char *extra_help)
{
    app_nk_spacing(app);

    /* Sample-specific help */
    if (extra_help && extra_help[0]) {
        char line[256];
        const char *p = extra_help;
        while (*p) {
            int i = 0;
            while (*p && *p != '\n' && i < 255)
                line[i++] = *p++;
            line[i] = '\0';
            if (*p == '\n') p++;
            nk_layout_row_dynamic(app->nk, 18, 1);
            nk_label(app->nk, line, NK_TEXT_LEFT);
        }
        app_nk_spacing(app);
    }

    /* Standard controls */
    nk_layout_row_dynamic(app->nk, 18, 1);
    nk_label(app->nk, "LMB: drag object", NK_TEXT_LEFT);
    nk_layout_row_dynamic(app->nk, 18, 1);
    nk_label(app->nk, "RMB: orbit camera", NK_TEXT_LEFT);
    nk_layout_row_dynamic(app->nk, 18, 1);
    nk_label(app->nk, "MMB: pan camera", NK_TEXT_LEFT);
    nk_layout_row_dynamic(app->nk, 18, 1);
    nk_label(app->nk, "Scroll: zoom", NK_TEXT_LEFT);
    nk_layout_row_dynamic(app->nk, 18, 1);
    nk_label(app->nk, "Tab: cycle focus", NK_TEXT_LEFT);
    nk_layout_row_dynamic(app->nk, 18, 1);
    nk_label(app->nk, "G: cycle gizmo mode", NK_TEXT_LEFT);
    nk_layout_row_dynamic(app->nk, 18, 1);
    nk_label(app->nk, "Esc: cancel gizmo/focus", NK_TEXT_LEFT);
    nk_layout_row_dynamic(app->nk, 18, 1);
    nk_label(app->nk, "Backspace: reset scene", NK_TEXT_LEFT);
}

} /* namespace ive */
