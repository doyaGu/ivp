/* render.c -- Minimal OpenGL 3.3 wireframe rendering implementation
 *
 * Uses glad for GL loading and SDL3 for windowing + input.
 * All drawing is immediate-mode style for simplicity.
 */

#include "render.h"

#include <glad/glad.h>
#include <SDL3/SDL.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ── Constants ──────────────────────────────────────────────────────────── */

#define MAX_VERTICES    (128 * 1024)
#define MAX_HUD_VERTICES (32 * 1024)
#define SPHERE_SEGS     24
#define PI_F            3.14159265358979323846f

/* ── Shader sources ─────────────────────────────────────────────────────── */

static const char *vs_src =
    "#version 330 core\n"
    "layout(location=0) in vec3 aPos;\n"
    "layout(location=1) in vec3 aCol;\n"
    "uniform mat4 uMVP;\n"
    "out vec3 vCol;\n"
    "void main(){\n"
    "  gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "  vCol = aCol;\n"
    "}\n";

static const char *fs_src =
    "#version 330 core\n"
    "in vec3 vCol;\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "  FragColor = vec4(vCol, 1.0);\n"
    "}\n";

/* 2D HUD shaders (orthographic) */
static const char *vs_2d_src =
    "#version 330 core\n"
    "layout(location=0) in vec2 aPos;\n"
    "layout(location=1) in vec3 aCol;\n"
    "uniform mat4 uProj;\n"
    "out vec3 vCol;\n"
    "void main(){\n"
    "  gl_Position = uProj * vec4(aPos, 0.0, 1.0);\n"
    "  vCol = aCol;\n"
    "}\n";

static const char *fs_2d_src =
    "#version 330 core\n"
    "in vec3 vCol;\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "  FragColor = vec4(vCol, 1.0);\n"
    "}\n";

/* ── Vertex layout ──────────────────────────────────────────────────────── */

typedef struct line_vertex {
    float pos[3];
    float col[3];
} line_vertex_t;

typedef struct hud_vertex {
    float pos[2];
    float col[3];
} hud_vertex_t;

/* ── Renderer state ─────────────────────────────────────────────────────── */

struct ivp_renderer {
    SDL_Window   *window;
    SDL_GLContext  gl_ctx;

    /* 3D line shader */
    GLuint         shader;
    GLint          u_mvp;
    GLuint         vao, vbo;
    line_vertex_t *vertices;
    int            vertex_count;

    /* 2D HUD shader */
    GLuint         shader_2d;
    GLint          u_proj_2d;
    GLuint         vao_2d, vbo_2d;
    hud_vertex_t  *hud_vertices;
    int            hud_vertex_count;

    float          view_mat[16];
    float          proj_mat[16];
    float          vp_mat[16];

    int            width, height;
    Uint64         start_ticks;
    float          frame_dt;

    /* mouse orbit state */
    bool           mouse_down;
    bool           rmouse_down;
    float          last_mx, last_my;

    /* keyboard state for camera pan */
    bool           key_w, key_a, key_s, key_d, key_q, key_e, key_r, key_f;

    float          clear_col[4];

    /* event/frame callbacks (for Nuklear integration etc.) */
    ivp_event_callback_t event_cb;
    void                *event_cb_ud;
    ivp_frame_callback_t pre_events_cb;
    void                *pre_events_cb_ud;
    ivp_frame_callback_t post_events_cb;
    void                *post_events_cb_ud;
    ivp_frame_callback_t pre_swap_cb;
    void                *pre_swap_cb_ud;
};

/* ── Matrix helpers (column-major) ──────────────────────────────────────── */

static void mat4_identity(float m[16])
{
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

static void mat4_mul(float out[16], const float a[16], const float b[16])
{
    float tmp[16];
    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            tmp[c*4 + r] = 0;
            for (int k = 0; k < 4; k++) {
                tmp[c*4 + r] += a[k*4 + r] * b[c*4 + k];
            }
        }
    }
    memcpy(out, tmp, sizeof(tmp));
}

static void mat4_perspective(float out[16], float fov_rad, float aspect,
                               float zn, float zf)
{
    float f = 1.0f / tanf(fov_rad * 0.5f);
    memset(out, 0, 16 * sizeof(float));
    out[0]  = f / aspect;
    out[5]  = f;
    out[10] = (zf + zn) / (zn - zf);
    out[11] = -1.0f;
    out[14] = (2.0f * zf * zn) / (zn - zf);
}

static void vec3_sub(float out[3], const float a[3], const float b[3]) {
    out[0] = a[0] - b[0]; out[1] = a[1] - b[1]; out[2] = a[2] - b[2];
}
static void vec3_add(float out[3], const float a[3], const float b[3]) {
    out[0] = a[0] + b[0]; out[1] = a[1] + b[1]; out[2] = a[2] + b[2];
}
static void vec3_scale(float out[3], const float v[3], float s) {
    out[0] = v[0]*s; out[1] = v[1]*s; out[2] = v[2]*s;
}
static float vec3_dot(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static void vec3_cross(float out[3], const float a[3], const float b[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}
static void vec3_normalize(float v[3]) {
    float l = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (l > 1e-8f) { v[0]/=l; v[1]/=l; v[2]/=l; }
}

static void mat4_ortho(float out[16], float l, float r, float b, float t,
                         float n, float f)
{
    memset(out, 0, 16 * sizeof(float));
    out[0]  = 2.0f / (r - l);
    out[5]  = 2.0f / (t - b);
    out[10] = -2.0f / (f - n);
    out[12] = -(r + l) / (r - l);
    out[13] = -(t + b) / (t - b);
    out[14] = -(f + n) / (f - n);
    out[15] = 1.0f;
}

static void mat4_lookat(float out[16], const float eye[3],
                          const float target[3], const float up[3])
{
    float f[3], s[3], u[3];
    vec3_sub(f, target, eye);
    vec3_normalize(f);
    vec3_cross(s, f, up);
    vec3_normalize(s);
    vec3_cross(u, s, f);

    mat4_identity(out);
    out[0] = s[0]; out[4] = s[1]; out[8]  = s[2];
    out[1] = u[0]; out[5] = u[1]; out[9]  = u[2];
    out[2] =-f[0]; out[6] =-f[1]; out[10] =-f[2];
    out[12] = -(s[0]*eye[0] + s[1]*eye[1] + s[2]*eye[2]);
    out[13] = -(u[0]*eye[0] + u[1]*eye[1] + u[2]*eye[2]);
    out[14] =  (f[0]*eye[0] + f[1]*eye[1] + f[2]*eye[2]);
}

/* ── Shader compile helper ──────────────────────────────────────────────── */

static GLuint compile_shader(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    if (!s) return 0;
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetShaderInfoLog(s, 512, NULL, log);
        fprintf(stderr, "Shader error: %s\n", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

static GLuint link_program(GLuint vs, GLuint fs)
{
    GLuint p = glCreateProgram();
    if (!vs || !fs || !p) {
        if (vs) glDeleteShader(vs);
        if (fs) glDeleteShader(fs);
        if (p) glDeleteProgram(p);
        return 0;
    }
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetProgramInfoLog(p, 512, NULL, log);
        fprintf(stderr, "Link error: %s\n", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    if (!ok) {
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

ivp_renderer_t *ivp_renderer_create(const ivp_render_config_t *config)
{
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return NULL;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    int w = config->width  > 0 ? config->width  : 1280;
    int h = config->height > 0 ? config->height : 720;

    SDL_Window *win = SDL_CreateWindow(
        config->title ? config->title : "IVP Example",
        w, h,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return NULL;
    }

    SDL_GLContext gl = SDL_GL_CreateContext(win);
    if (!gl) {
        fprintf(stderr, "GL context failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(win);
        SDL_Quit();
        return NULL;
    }

    int version = gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress);
    if (!version) {
        fprintf(stderr, "gladLoadGL failed\n");
        SDL_GL_DestroyContext(gl);
        SDL_DestroyWindow(win);
        SDL_Quit();
        return NULL;
    }

    SDL_GL_SetSwapInterval(config->vsync ? 1 : 0);

    ivp_renderer_t *r = (ivp_renderer_t *)calloc(1, sizeof(*r));
    if (!r) {
        SDL_GL_DestroyContext(gl);
        SDL_DestroyWindow(win);
        SDL_Quit();
        return NULL;
    }
    r->window = win;
    r->gl_ctx = gl;
    r->vertices = (line_vertex_t *)malloc(MAX_VERTICES * sizeof(line_vertex_t));
    r->hud_vertices = (hud_vertex_t *)malloc(MAX_HUD_VERTICES * sizeof(hud_vertex_t));
    if (!r->vertices || !r->hud_vertices) {
        ivp_renderer_destroy(r);
        return NULL;
    }

    /* Compile 3D shaders */
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_src);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fs_src);
    GLuint prog = link_program(vs, fs);
    r->shader = prog;
    if (!prog) {
        ivp_renderer_destroy(r);
        return NULL;
    }

    /* Compile 2D HUD shaders */
    GLuint vs2 = compile_shader(GL_VERTEX_SHADER, vs_2d_src);
    GLuint fs2 = compile_shader(GL_FRAGMENT_SHADER, fs_2d_src);
    GLuint prog2 = link_program(vs2, fs2);
    r->shader_2d = prog2;
    if (!prog2) {
        ivp_renderer_destroy(r);
        return NULL;
    }

    /* VAO/VBO for 3D line rendering */
    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                   MAX_VERTICES * (GLsizeiptr)sizeof(line_vertex_t),
                   NULL, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                            sizeof(line_vertex_t),
                            (void *)offsetof(line_vertex_t, pos));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                            sizeof(line_vertex_t),
                            (void *)offsetof(line_vertex_t, col));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    /* VAO/VBO for 2D HUD rendering */
    GLuint vao2, vbo2;
    glGenVertexArrays(1, &vao2);
    glGenBuffers(1, &vbo2);
    glBindVertexArray(vao2);
    glBindBuffer(GL_ARRAY_BUFFER, vbo2);
    glBufferData(GL_ARRAY_BUFFER,
                   MAX_HUD_VERTICES * (GLsizeiptr)sizeof(hud_vertex_t),
                   NULL, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE,
                            sizeof(hud_vertex_t),
                            (void *)offsetof(hud_vertex_t, pos));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                            sizeof(hud_vertex_t),
                            (void *)offsetof(hud_vertex_t, col));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glLineWidth(1.5f);

    r->window       = win;
    r->gl_ctx       = gl;
    r->shader       = prog;
    r->u_mvp        = glGetUniformLocation(prog, "uMVP");
    r->vao          = vao;
    r->vbo          = vbo;
    r->shader_2d    = prog2;
    r->u_proj_2d    = glGetUniformLocation(prog2, "uProj");
    r->vao_2d       = vao2;
    r->vbo_2d       = vbo2;
    r->width        = w;
    r->height       = h;
    r->start_ticks  = SDL_GetPerformanceCounter();
    r->vertex_count = 0;
    r->hud_vertex_count = 0;
    r->clear_col[0] = 0.20f;
    r->clear_col[1] = 0.24f;
    r->clear_col[2] = 0.30f;
    r->clear_col[3] = 1.0f;
    return r;
}

void ivp_renderer_destroy(ivp_renderer_t *r)
{
    if (!r) return;
    glDeleteBuffers(1, &r->vbo);
    glDeleteBuffers(1, &r->vbo_2d);
    glDeleteVertexArrays(1, &r->vao);
    glDeleteVertexArrays(1, &r->vao_2d);
    glDeleteProgram(r->shader);
    glDeleteProgram(r->shader_2d);
    SDL_GL_DestroyContext(r->gl_ctx);
    SDL_DestroyWindow(r->window);
    free(r->vertices);
    free(r->hud_vertices);
    free(r);
    SDL_Quit();
}

/* ── Frame begin/end ────────────────────────────────────────────────────── */

bool ivp_renderer_begin_frame(ivp_renderer_t *r, ivp_camera_t *cam)
{
    static Uint64 prev_ticks = 0;
    Uint64 now_ticks = SDL_GetPerformanceCounter();
    if (prev_ticks == 0) prev_ticks = now_ticks;
    r->frame_dt = (float)((double)(now_ticks - prev_ticks) / (double)SDL_GetPerformanceFrequency());
    prev_ticks = now_ticks;
    if (r->frame_dt > 0.1f) r->frame_dt = 0.1f;

    /* Pre-events callback (e.g. nk_input_begin) */
    if (r->pre_events_cb) r->pre_events_cb(r->pre_events_cb_ud);

    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_EVENT_QUIT) return false;

        /* Per-event callback (e.g. Nuklear); if it returns true, skip camera handling */
        if (r->event_cb && r->event_cb(&ev, r->event_cb_ud))
            continue;

        /* Keyboard pan state */
        if (ev.type == SDL_EVENT_KEY_DOWN || ev.type == SDL_EVENT_KEY_UP) {
            bool down = (ev.type == SDL_EVENT_KEY_DOWN);
            switch (ev.key.key) {
            case SDLK_W: r->key_w = down; break;
            case SDLK_A: r->key_a = down; break;
            case SDLK_S: r->key_s = down; break;
            case SDLK_D: r->key_d = down; break;
            case SDLK_Q: r->key_q = down; break;
            case SDLK_E: r->key_e = down; break;
            case SDLK_R: r->key_r = down; break;
            case SDLK_F: r->key_f = down; break;
            default: break;
            }
        }

        /* Mouse orbit (right button) */
        if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN && ev.button.button == SDL_BUTTON_RIGHT) {
            r->rmouse_down = true;
            r->last_mx = ev.button.x;
            r->last_my = ev.button.y;
        }
        if (ev.type == SDL_EVENT_MOUSE_BUTTON_UP && ev.button.button == SDL_BUTTON_RIGHT) {
            r->rmouse_down = false;
        }
        /* Mouse pan (middle button) */
        if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN && ev.button.button == SDL_BUTTON_MIDDLE) {
            r->mouse_down = true;
            r->last_mx = ev.button.x;
            r->last_my = ev.button.y;
        }
        if (ev.type == SDL_EVENT_MOUSE_BUTTON_UP && ev.button.button == SDL_BUTTON_MIDDLE) {
            r->mouse_down = false;
        }

        if (ev.type == SDL_EVENT_MOUSE_MOTION && cam) {
            float dx = ev.motion.x - r->last_mx;
            float dy = ev.motion.y - r->last_my;
            r->last_mx = ev.motion.x;
            r->last_my = ev.motion.y;
            if (r->rmouse_down) {
                cam->orbit_yaw   += dx * 0.5f;
                cam->orbit_pitch += dy * 0.5f;
                if (cam->orbit_pitch >  89.0f) cam->orbit_pitch =  89.0f;
                if (cam->orbit_pitch < -89.0f) cam->orbit_pitch = -89.0f;
            }
            if (r->mouse_down) {
                float yr = cam->orbit_yaw * PI_F / 180.0f;
                float right[3] = { cosf(yr), 0, -sinf(yr) };
                float pan_speed = cam->orbit_distance * 0.003f;
                cam->target[0] -= right[0] * dx * pan_speed;
                cam->target[2] -= right[2] * dx * pan_speed;
                cam->target[1] -= dy * pan_speed;
            }
        }

        if (ev.type == SDL_EVENT_MOUSE_WHEEL && cam) {
            cam->orbit_distance -= ev.wheel.y * cam->orbit_distance * 0.1f;
            if (cam->orbit_distance < 0.5f) cam->orbit_distance = 0.5f;
            if (cam->orbit_distance > 200.0f) cam->orbit_distance = 200.0f;
        }
        if (ev.type == SDL_EVENT_WINDOW_RESIZED) {
            r->width  = ev.window.data1;
            r->height = ev.window.data2;
        }
    }

    /* Post-events callback (e.g. nk_input_end) */
    if (r->post_events_cb) r->post_events_cb(r->post_events_cb_ud);

    /* WASD camera pan */
    if (cam) {
        float pan_speed = cam->orbit_distance * 0.02f;
        float yr = cam->orbit_yaw * PI_F / 180.0f;
        float fwd[3] = { -sinf(yr), 0, -cosf(yr) };
        float right[3] = { -cosf(yr), 0, sinf(yr) };
        if (r->key_w) { cam->target[0]+=fwd[0]*pan_speed; cam->target[2]+=fwd[2]*pan_speed; }
        if (r->key_s) { cam->target[0]-=fwd[0]*pan_speed; cam->target[2]-=fwd[2]*pan_speed; }
        if (r->key_a) { cam->target[0]-=right[0]*pan_speed; cam->target[2]-=right[2]*pan_speed; }
        if (r->key_d) { cam->target[0]+=right[0]*pan_speed; cam->target[2]+=right[2]*pan_speed; }
        if (r->key_q) { cam->target[1] -= pan_speed; }
        if (r->key_e) { cam->target[1] += pan_speed; }
        if (r->key_r) { cam->target[1] -= pan_speed; }
        if (r->key_f) { cam->target[1] += pan_speed; }
    }

    /* Update camera matrices */
    if (cam) {
        ivp_camera_update_orbit(cam);
        mat4_lookat(r->view_mat, cam->eye, cam->target, cam->up);
    } else {
        mat4_identity(r->view_mat);
    }

    float aspect = (r->height > 0) ? (float)r->width / (float)r->height : 1.0f;
    float fov = cam ? cam->fov_deg : 60.0f;
    float zn  = cam ? cam->near_plane : 0.1f;
    float zf  = cam ? cam->far_plane : 500.0f;
    mat4_perspective(r->proj_mat, fov * PI_F / 180.0f, aspect, zn, zf);
    mat4_mul(r->vp_mat, r->proj_mat, r->view_mat);

    glViewport(0, 0, r->width, r->height);
    glClearColor(r->clear_col[0], r->clear_col[1], r->clear_col[2], r->clear_col[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    r->vertex_count = 0;
    r->hud_vertex_count = 0;
    return true;
}

void ivp_renderer_end_frame(ivp_renderer_t *r)
{
    /* Draw 3D lines */
    if (r->vertex_count > 0) {
        glBindVertexArray(r->vao);
        glBindBuffer(GL_ARRAY_BUFFER, r->vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                          r->vertex_count * (GLsizeiptr)sizeof(line_vertex_t),
                          r->vertices);
        glUseProgram(r->shader);
        glUniformMatrix4fv(r->u_mvp, 1, GL_FALSE, r->vp_mat);
        glDrawArrays(GL_LINES, 0, r->vertex_count);
        glBindVertexArray(0);
    }

    /* Draw 2D HUD overlay (no depth test) */
    if (r->hud_vertex_count > 0) {
        glDisable(GL_DEPTH_TEST);
        float ortho[16];
        mat4_ortho(ortho, 0.0f, (float)r->width, (float)r->height, 0.0f, -1.0f, 1.0f);

        glBindVertexArray(r->vao_2d);
        glBindBuffer(GL_ARRAY_BUFFER, r->vbo_2d);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                          r->hud_vertex_count * (GLsizeiptr)sizeof(hud_vertex_t),
                          r->hud_vertices);
        glUseProgram(r->shader_2d);
        glUniformMatrix4fv(r->u_proj_2d, 1, GL_FALSE, ortho);
        glDrawArrays(GL_LINES, 0, r->hud_vertex_count);
        glBindVertexArray(0);
        glEnable(GL_DEPTH_TEST);
    }

    /* Pre-swap callback (e.g. Nuklear render) */
    if (r->pre_swap_cb) r->pre_swap_cb(r->pre_swap_cb_ud);

    SDL_GL_SwapWindow(r->window);
}

/* ── Camera ─────────────────────────────────────────────────────────────── */

void ivp_camera_init(ivp_camera_t *cam)
{
    memset(cam, 0, sizeof(*cam));
    cam->target[1]    = -3.0f;     /* Y-down physics, look a bit above ground */
    cam->up[1]        = -1.0f;     /* Y-down world */
    cam->fov_deg      = 60.0f;
    cam->near_plane   = 0.1f;
    cam->far_plane    = 500.0f;
    cam->orbit_distance = 20.0f;
    cam->orbit_yaw     = 45.0f;
    cam->orbit_pitch   = -30.0f;
}

void ivp_camera_update_orbit(ivp_camera_t *cam)
{
    float yr = cam->orbit_yaw   * PI_F / 180.0f;
    float pr = cam->orbit_pitch * PI_F / 180.0f;
    float cp = cosf(pr), sp = sinf(pr);
    float cy = cosf(yr), sy = sinf(yr);

    /* IVP uses Y-down: eye position relative to target */
    cam->eye[0] = cam->target[0] + cam->orbit_distance * cp * sy;
    cam->eye[1] = cam->target[1] + cam->orbit_distance * sp;  /* up is -Y */
    cam->eye[2] = cam->target[2] + cam->orbit_distance * cp * cy;
}

/* ── Internal: push a line segment ──────────────────────────────────────── */

static void push_line(ivp_renderer_t *r,
                        const float a[3], const float b[3],
                        const float col[3])
{
    if (r->vertex_count + 2 > MAX_VERTICES) return;
    line_vertex_t *v = &r->vertices[r->vertex_count];
    memcpy(v[0].pos, a, 3 * sizeof(float));
    memcpy(v[0].col, col, 3 * sizeof(float));
    memcpy(v[1].pos, b, 3 * sizeof(float));
    memcpy(v[1].col, col, 3 * sizeof(float));
    r->vertex_count += 2;
}

/* ── Drawing primitives ─────────────────────────────────────────────────── */

void ivp_draw_line(ivp_renderer_t *r,
                     const float a[3], const float b[3],
                     const float color[3])
{
    push_line(r, a, b, color);
}

void ivp_draw_wire_box(ivp_renderer_t *r,
                         const float pos[3],
                         const float hs[3],
                         const float *rot,
                         const float col[3])
{
    /* 8 corners of a box */
    float corners[8][3];
    int idx = 0;
    for (int z = -1; z <= 1; z += 2)
    for (int y = -1; y <= 1; y += 2)
    for (int x = -1; x <= 1; x += 2) {
        float lx = (float)x * hs[0];
        float ly = (float)y * hs[1];
        float lz = (float)z * hs[2];
        if (rot) {
            /* Column-major rotation: [col0 col1 col2] * local */
            corners[idx][0] = pos[0] + rot[0]*lx + rot[3]*ly + rot[6]*lz;
            corners[idx][1] = pos[1] + rot[1]*lx + rot[4]*ly + rot[7]*lz;
            corners[idx][2] = pos[2] + rot[2]*lx + rot[5]*ly + rot[8]*lz;
        } else {
            corners[idx][0] = pos[0] + lx;
            corners[idx][1] = pos[1] + ly;
            corners[idx][2] = pos[2] + lz;
        }
        idx++;
    }
    /* 12 edges of a box: iterate adjacent pairs
     * Ordering: 0(-,-,-) 1(+,-,-) 2(-,+,-) 3(+,+,-) 4(-,-,+) 5(+,-,+) 6(-,+,+) 7(+,+,+) */
    static const int edges[12][2] = {
        {0,1},{2,3},{4,5},{6,7},  /* X edges */
        {0,2},{1,3},{4,6},{5,7},  /* Y edges */
        {0,4},{1,5},{2,6},{3,7}   /* Z edges */
    };
    for (int i = 0; i < 12; i++) {
        push_line(r, corners[edges[i][0]], corners[edges[i][1]], col);
    }
}

void ivp_draw_wire_sphere_ex(ivp_renderer_t *r,
                            const float pos[3],
                            float radius,
                            const float col[3], int segments)
{
    if (segments < 4) segments = 4;
    /* Three orthogonal circles */
    for (int ring = 0; ring < 3; ring++) {
        for (int i = 0; i < segments; i++) {
            float a0 = (float)i       / (float)segments * 2.0f * PI_F;
            float a1 = (float)(i + 1) / (float)segments * 2.0f * PI_F;
            float p0[3], p1[3];
            if (ring == 0) {  /* XY */
                p0[0] = pos[0] + cosf(a0)*radius; p0[1] = pos[1] + sinf(a0)*radius; p0[2] = pos[2];
                p1[0] = pos[0] + cosf(a1)*radius; p1[1] = pos[1] + sinf(a1)*radius; p1[2] = pos[2];
            } else if (ring == 1) { /* XZ */
                p0[0] = pos[0] + cosf(a0)*radius; p0[1] = pos[1]; p0[2] = pos[2] + sinf(a0)*radius;
                p1[0] = pos[0] + cosf(a1)*radius; p1[1] = pos[1]; p1[2] = pos[2] + sinf(a1)*radius;
            } else { /* YZ */
                p0[0] = pos[0]; p0[1] = pos[1] + cosf(a0)*radius; p0[2] = pos[2] + sinf(a0)*radius;
                p1[0] = pos[0]; p1[1] = pos[1] + cosf(a1)*radius; p1[2] = pos[2] + sinf(a1)*radius;
            }
            push_line(r, p0, p1, col);
        }
    }
}

void ivp_draw_wire_sphere(ivp_renderer_t *r, const float pos[3],
                          float radius, const float col[3])
{
    ivp_draw_wire_sphere_ex(r, pos, radius, col, SPHERE_SEGS);
}

void ivp_draw_ground_grid(ivp_renderer_t *r,
                            float extent,
                            float spacing,
                            const float col[3])
{
    for (float x = -extent; x <= extent + 0.01f; x += spacing) {
        float a[3] = {x, 0.0f, -extent};
        float b[3] = {x, 0.0f,  extent};
        push_line(r, a, b, col);
    }
    for (float z = -extent; z <= extent + 0.01f; z += spacing) {
        float a[3] = {-extent, 0.0f, z};
        float b[3] = { extent, 0.0f, z};
        push_line(r, a, b, col);
    }
}

/* ── New drawing primitives ─────────────────────────────────────────────── */

void ivp_draw_thick_line(ivp_renderer_t *r,
                           const float a[3], const float b[3],
                           const float color[3], float thickness)
{
    push_line(r, a, b, color);
    float offset = thickness * 0.002f;
    for (int axis = 0; axis < 3; axis++) {
        float a2[3], b2[3];
        memcpy(a2, a, sizeof(float)*3);
        memcpy(b2, b, sizeof(float)*3);
        a2[axis] += offset; b2[axis] += offset;
        push_line(r, a2, b2, color);
        a2[axis] -= 2*offset; b2[axis] -= 2*offset;
        push_line(r, a2, b2, color);
    }
}

void ivp_draw_arrow(ivp_renderer_t *r,
                      const float from[3], const float to[3],
                      const float color[3], float head_size)
{
    push_line(r, from, to, color);
    float dir[3];
    vec3_sub(dir, from, to);
    float len = sqrtf(vec3_dot(dir, dir));
    if (len < 1e-6f) return;
    dir[0] /= len; dir[1] /= len; dir[2] /= len;

    float perp[3];
    if (fabsf(dir[1]) < 0.9f) {
        float tmp[3] = {0, 1, 0}; vec3_cross(perp, dir, tmp);
    } else {
        float tmp[3] = {1, 0, 0}; vec3_cross(perp, dir, tmp);
    }
    vec3_normalize(perp);
    float perp2[3];
    vec3_cross(perp2, dir, perp);

    for (int i = 0; i < 4; i++) {
        float angle = (float)i * PI_F * 0.5f;
        float c = cosf(angle), s = sinf(angle);
        float tip[3];
        tip[0] = to[0] + (dir[0] + perp[0]*c + perp2[0]*s) * head_size * 0.3f;
        tip[1] = to[1] + (dir[1] + perp[1]*c + perp2[1]*s) * head_size * 0.3f;
        tip[2] = to[2] + (dir[2] + perp[2]*c + perp2[2]*s) * head_size * 0.3f;
        push_line(r, to, tip, color);
    }
}

void ivp_draw_wire_cylinder(ivp_renderer_t *r,
                              const float base[3],
                              const float axis[3],
                              float height,
                              float radius,
                              int segments,
                              const float color[3])
{
    if (segments < 4) segments = 4;
    float u[3], v[3];
    if (fabsf(axis[1]) < 0.9f) {
        float tmp[3] = {0, 1, 0}; vec3_cross(u, axis, tmp);
    } else {
        float tmp[3] = {1, 0, 0}; vec3_cross(u, axis, tmp);
    }
    vec3_normalize(u);
    vec3_cross(v, axis, u);

    float top[3] = {
        base[0] + axis[0]*height,
        base[1] + axis[1]*height,
        base[2] + axis[2]*height
    };

    for (int i = 0; i < segments; i++) {
        float a0 = (float)i       / (float)segments * 2.0f * PI_F;
        float a1 = (float)(i + 1) / (float)segments * 2.0f * PI_F;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);

        float b0[3], b1[3], t0[3], t1[3];
        for (int k = 0; k < 3; k++) {
            b0[k] = base[k] + (u[k]*c0 + v[k]*s0) * radius;
            b1[k] = base[k] + (u[k]*c1 + v[k]*s1) * radius;
            t0[k] = top[k]  + (u[k]*c0 + v[k]*s0) * radius;
            t1[k] = top[k]  + (u[k]*c1 + v[k]*s1) * radius;
        }
        push_line(r, b0, b1, color);
        push_line(r, t0, t1, color);
        push_line(r, b0, t0, color);
    }
}

void ivp_draw_wire_quad(ivp_renderer_t *r,
                          const float corners[4][3],
                          const float color[3])
{
    push_line(r, corners[0], corners[1], color);
    push_line(r, corners[1], corners[2], color);
    push_line(r, corners[2], corners[3], color);
    push_line(r, corners[3], corners[0], color);
    push_line(r, corners[0], corners[2], color);
}

void ivp_draw_axes(ivp_renderer_t *r,
                     const float pos[3],
                     const float *rot,
                     float scale)
{
    const float red[3]   = {1,0,0};
    const float green[3] = {0,1,0};
    const float blue[3]  = {0,0,1};
    float x[3], y[3], z[3];
    if (rot) {
        x[0] = pos[0]+rot[0]*scale; x[1] = pos[1]+rot[1]*scale; x[2] = pos[2]+rot[2]*scale;
        y[0] = pos[0]+rot[3]*scale; y[1] = pos[1]+rot[4]*scale; y[2] = pos[2]+rot[5]*scale;
        z[0] = pos[0]+rot[6]*scale; z[1] = pos[1]+rot[7]*scale; z[2] = pos[2]+rot[8]*scale;
    } else {
        x[0] = pos[0]+scale; x[1] = pos[1]; x[2] = pos[2];
        y[0] = pos[0]; y[1] = pos[1]+scale; y[2] = pos[2];
        z[0] = pos[0]; z[1] = pos[1]; z[2] = pos[2]+scale;
    }
    push_line(r, pos, x, red);
    push_line(r, pos, y, green);
    push_line(r, pos, z, blue);
}

/* ── HUD helpers ────────────────────────────────────────────────────────── */

static void push_hud_line(ivp_renderer_t *r,
                            float x0, float y0, float x1, float y1,
                            const float col[3])
{
    if (r->hud_vertex_count + 2 > MAX_HUD_VERTICES) return;
    hud_vertex_t *v = &r->hud_vertices[r->hud_vertex_count];
    v[0].pos[0] = x0; v[0].pos[1] = y0;
    memcpy(v[0].col, col, 3 * sizeof(float));
    v[1].pos[0] = x1; v[1].pos[1] = y1;
    memcpy(v[1].col, col, 3 * sizeof(float));
    r->hud_vertex_count += 2;
}

/* ── Minimal 5x7 bitmap font ───────────────────────────────────────────── */

#define FONT_FIRST 32
#define FONT_LAST  126
#define FONT_W     5
#define FONT_H     7
#define FONT_SCALE 2

static const unsigned char s_font[][FONT_H] = {
    /* 32 ' ' */ {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    /* 33 '!' */ {0x04,0x04,0x04,0x04,0x00,0x04,0x00},
    /* 34 '"' */ {0x0A,0x0A,0x00,0x00,0x00,0x00,0x00},
    /* 35 '#' */ {0x0A,0x1F,0x0A,0x0A,0x1F,0x0A,0x00},
    /* 36 '$' */ {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04},
    /* 37 '%' */ {0x19,0x19,0x02,0x04,0x08,0x13,0x13},
    /* 38 '&' */ {0x08,0x14,0x14,0x08,0x15,0x12,0x0D},
    /* 39 ''' */ {0x04,0x04,0x00,0x00,0x00,0x00,0x00},
    /* 40 '(' */ {0x02,0x04,0x08,0x08,0x08,0x04,0x02},
    /* 41 ')' */ {0x08,0x04,0x02,0x02,0x02,0x04,0x08},
    /* 42 '*' */ {0x04,0x15,0x0E,0x1F,0x0E,0x15,0x04},
    /* 43 '+' */ {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
    /* 44 ',' */ {0x00,0x00,0x00,0x00,0x04,0x04,0x08},
    /* 45 '-' */ {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    /* 46 '.' */ {0x00,0x00,0x00,0x00,0x00,0x04,0x00},
    /* 47 '/' */ {0x01,0x01,0x02,0x04,0x08,0x10,0x10},
    /* 48 '0' */ {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    /* 49 '1' */ {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    /* 50 '2' */ {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},
    /* 51 '3' */ {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    /* 52 '4' */ {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    /* 53 '5' */ {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    /* 54 '6' */ {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    /* 55 '7' */ {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    /* 56 '8' */ {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    /* 57 '9' */ {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
    /* 58 ':' */ {0x00,0x04,0x00,0x00,0x04,0x00,0x00},
    /* 59 ';' */ {0x00,0x04,0x00,0x00,0x04,0x04,0x08},
    /* 60 '<' */ {0x02,0x04,0x08,0x10,0x08,0x04,0x02},
    /* 61 '=' */ {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
    /* 62 '>' */ {0x08,0x04,0x02,0x01,0x02,0x04,0x08},
    /* 63 '?' */ {0x0E,0x11,0x01,0x02,0x04,0x00,0x04},
    /* 64 '@' */ {0x0E,0x11,0x17,0x15,0x17,0x10,0x0E},
    /* 65 'A' */ {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* 66 'B' */ {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    /* 67 'C' */ {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    /* 68 'D' */ {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
    /* 69 'E' */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    /* 70 'F' */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    /* 71 'G' */ {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},
    /* 72 'H' */ {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* 73 'I' */ {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    /* 74 'J' */ {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
    /* 75 'K' */ {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
    /* 76 'L' */ {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    /* 77 'M' */ {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
    /* 78 'N' */ {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    /* 79 'O' */ {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* 80 'P' */ {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    /* 81 'Q' */ {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
    /* 82 'R' */ {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    /* 83 'S' */ {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E},
    /* 84 'T' */ {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    /* 85 'U' */ {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* 86 'V' */ {0x11,0x11,0x11,0x0A,0x0A,0x04,0x04},
    /* 87 'W' */ {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
    /* 88 'X' */ {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    /* 89 'Y' */ {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
    /* 90 'Z' */ {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
    /* 91 '[' */ {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E},
    /* 92 '\' */ {0x10,0x10,0x08,0x04,0x02,0x01,0x01},
    /* 93 ']' */ {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E},
    /* 94 '^' */ {0x04,0x0A,0x11,0x00,0x00,0x00,0x00},
    /* 95 '_' */ {0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
    /* 96 '`' */ {0x08,0x04,0x00,0x00,0x00,0x00,0x00},
    /* 97 'a' */ {0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F},
    /* 98 'b' */ {0x10,0x10,0x1E,0x11,0x11,0x11,0x1E},
    /* 99 'c' */ {0x00,0x00,0x0E,0x11,0x10,0x11,0x0E},
    /*100 'd' */ {0x01,0x01,0x0F,0x11,0x11,0x11,0x0F},
    /*101 'e' */ {0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E},
    /*102 'f' */ {0x06,0x08,0x1C,0x08,0x08,0x08,0x08},
    /*103 'g' */ {0x00,0x00,0x0F,0x11,0x0F,0x01,0x0E},
    /*104 'h' */ {0x10,0x10,0x1E,0x11,0x11,0x11,0x11},
    /*105 'i' */ {0x04,0x00,0x0C,0x04,0x04,0x04,0x0E},
    /*106 'j' */ {0x02,0x00,0x06,0x02,0x02,0x12,0x0C},
    /*107 'k' */ {0x10,0x10,0x12,0x14,0x18,0x14,0x12},
    /*108 'l' */ {0x0C,0x04,0x04,0x04,0x04,0x04,0x0E},
    /*109 'm' */ {0x00,0x00,0x1A,0x15,0x15,0x11,0x11},
    /*110 'n' */ {0x00,0x00,0x1E,0x11,0x11,0x11,0x11},
    /*111 'o' */ {0x00,0x00,0x0E,0x11,0x11,0x11,0x0E},
    /*112 'p' */ {0x00,0x00,0x1E,0x11,0x1E,0x10,0x10},
    /*113 'q' */ {0x00,0x00,0x0F,0x11,0x0F,0x01,0x01},
    /*114 'r' */ {0x00,0x00,0x16,0x19,0x10,0x10,0x10},
    /*115 's' */ {0x00,0x00,0x0F,0x10,0x0E,0x01,0x1E},
    /*116 't' */ {0x08,0x08,0x1C,0x08,0x08,0x09,0x06},
    /*117 'u' */ {0x00,0x00,0x11,0x11,0x11,0x13,0x0D},
    /*118 'v' */ {0x00,0x00,0x11,0x11,0x0A,0x0A,0x04},
    /*119 'w' */ {0x00,0x00,0x11,0x11,0x15,0x15,0x0A},
    /*120 'x' */ {0x00,0x00,0x11,0x0A,0x04,0x0A,0x11},
    /*121 'y' */ {0x00,0x00,0x11,0x11,0x0F,0x01,0x0E},
    /*122 'z' */ {0x00,0x00,0x1F,0x02,0x04,0x08,0x1F},
    /*123 '{' */ {0x02,0x04,0x04,0x08,0x04,0x04,0x02},
    /*124 '|' */ {0x04,0x04,0x04,0x04,0x04,0x04,0x04},
    /*125 '}' */ {0x08,0x04,0x04,0x02,0x04,0x04,0x08},
    /*126 '~' */ {0x00,0x00,0x08,0x15,0x02,0x00,0x00},
};

void ivp_draw_text_2d(ivp_renderer_t *r,
                        float x, float y,
                        const char *text,
                        const float color[3])
{
    float cx = x, cy = y;
    float sc = (float)FONT_SCALE;

    for (const char *p = text; *p; p++) {
        if (*p == '\n') { cx = x; cy += (FONT_H + 1) * sc; continue; }
        int ch = (int)(unsigned char)*p;
        if (ch < FONT_FIRST || ch > FONT_LAST) { cx += (FONT_W + 1)*sc; continue; }

        const unsigned char *g = s_font[ch - FONT_FIRST];
        for (int row = 0; row < FONT_H; row++) {
            unsigned char bits = g[row];
            for (int col = 0; col < FONT_W; col++) {
                if (bits & (1 << (FONT_W - 1 - col))) {
                    float px = cx + (float)col * sc;
                    float py = cy + (float)row * sc;
                    push_hud_line(r, px, py, px + sc, py, color);
                }
            }
        }
        cx += (FONT_W + 1) * sc;
    }
}

double ivp_renderer_get_time(const ivp_renderer_t *r)
{
    Uint64 now = SDL_GetPerformanceCounter();
    return (double)(now - r->start_ticks) / (double)SDL_GetPerformanceFrequency();
}

float ivp_renderer_get_dt(const ivp_renderer_t *r)
{
    return r->frame_dt;
}

void ivp_renderer_get_size(const ivp_renderer_t *r, int *w, int *h)
{
    if (w) *w = r->width;
    if (h) *h = r->height;
}

void ivp_renderer_set_clear_color(ivp_renderer_t *r,
                                    float red,
                                    float green,
                                    float blue,
                                    float alpha)
{
    if (!r) return;
    if (red < 0.0f) red = 0.0f; if (red > 1.0f) red = 1.0f;
    if (green < 0.0f) green = 0.0f; if (green > 1.0f) green = 1.0f;
    if (blue < 0.0f) blue = 0.0f; if (blue > 1.0f) blue = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f; if (alpha > 1.0f) alpha = 1.0f;
    r->clear_col[0] = red;
    r->clear_col[1] = green;
    r->clear_col[2] = blue;
    r->clear_col[3] = alpha;
}

/* ── Callback setters ──────────────────────────────────────────────────── */

void ivp_renderer_set_event_callback(ivp_renderer_t *r, ivp_event_callback_t cb, void *userdata)
{
    if (!r) return;
    r->event_cb = cb;
    r->event_cb_ud = userdata;
}

void ivp_renderer_set_pre_events_callback(ivp_renderer_t *r, ivp_frame_callback_t cb, void *userdata)
{
    if (!r) return;
    r->pre_events_cb = cb;
    r->pre_events_cb_ud = userdata;
}

void ivp_renderer_set_post_events_callback(ivp_renderer_t *r, ivp_frame_callback_t cb, void *userdata)
{
    if (!r) return;
    r->post_events_cb = cb;
    r->post_events_cb_ud = userdata;
}

void ivp_renderer_set_pre_swap_callback(ivp_renderer_t *r, ivp_frame_callback_t cb, void *userdata)
{
    if (!r) return;
    r->pre_swap_cb = cb;
    r->pre_swap_cb_ud = userdata;
}

bool ivp_renderer_project(const ivp_renderer_t *r,
                          const float world[3], float screen[2])
{
    const float *m = r->vp_mat;
    float x = m[0]*world[0] + m[4]*world[1] + m[8]*world[2]  + m[12];
    float y = m[1]*world[0] + m[5]*world[1] + m[9]*world[2]  + m[13];
    float w = m[3]*world[0] + m[7]*world[1] + m[11]*world[2] + m[15];
    if (w <= 0.0001f) return false;
    float inv_w = 1.0f / w;
    float ndc_x = x * inv_w;
    float ndc_y = y * inv_w;
    screen[0] = (ndc_x * 0.5f + 0.5f) * (float)r->width;
    screen[1] = (1.0f - (ndc_y * 0.5f + 0.5f)) * (float)r->height;
    return true;
}

const float *ivp_renderer_get_vp_matrix(const ivp_renderer_t *r)
{
    return r->vp_mat;
}

const float *ivp_renderer_get_view_matrix(const ivp_renderer_t *r)
{
    return r->view_mat;
}

SDL_Window *ivp_renderer_get_window(const ivp_renderer_t *r)
{
    return r ? r->window : NULL;
}
