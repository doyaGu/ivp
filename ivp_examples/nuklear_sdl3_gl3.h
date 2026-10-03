/*
 * Nuklear SDL3 + OpenGL 3.3 backend
 *
 * Ported from Nuklear/demo/sdl_opengl3/nuklear_sdl_gl3.h (SDL2)
 * with SDL3 API changes based on Nuklear/demo/sdl3_renderer/.
 *
 * Public domain — no warranty implied; use at your own risk.
 */
#ifndef NK_SDL3_GL3_H_
#define NK_SDL3_GL3_H_

#include <SDL3/SDL.h>

#ifdef __cplusplus
extern "C" {
#endif

NK_API struct nk_context*   nk_sdl_init(SDL_Window *win);
NK_API void                 nk_sdl_font_stash_begin(struct nk_font_atlas **atlas);
NK_API void                 nk_sdl_font_stash_end(void);
NK_API int                  nk_sdl_handle_event(SDL_Event *evt);
NK_API void                 nk_sdl_render(enum nk_anti_aliasing, int max_vertex_buffer, int max_element_buffer);
NK_API void                 nk_sdl_shutdown(void);
NK_API void                 nk_sdl_device_destroy(void);
NK_API void                 nk_sdl_device_create(void);

#ifdef __cplusplus
}
#endif

#endif /* NK_SDL3_GL3_H_ */

/*
 * ==============================================================
 *
 *                          IMPLEMENTATION
 *
 * ===============================================================
 */
#ifdef NK_SDL3_GL3_IMPLEMENTATION

#include <stddef.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

/* glad provides GL functions for our context */
#include <glad/glad.h>

struct nk_sdl_device {
    struct nk_buffer cmds;
    struct nk_draw_null_texture tex_null;
    GLuint vbo, vao, ebo;
    GLuint prog;
    GLuint vert_shdr;
    GLuint frag_shdr;
    GLint attrib_pos;
    GLint attrib_uv;
    GLint attrib_col;
    GLint uniform_tex;
    GLint uniform_proj;
    GLuint font_tex;
};

struct nk_sdl_vertex {
    float position[2];
    float uv[2];
    nk_byte col[4];
};

static struct nk_sdl {
    SDL_Window *win;
    struct nk_sdl_device ogl;
    struct nk_context ctx;
    struct nk_font_atlas atlas;
    Uint64 time_of_last_frame;
} sdl;

#define NK_SHADER_VERSION "#version 330 core\n"

NK_API void
nk_sdl_device_create(void)
{
    GLint status;
    static const GLchar *vertex_shader =
        NK_SHADER_VERSION
        "uniform mat4 ProjMtx;\n"
        "in vec2 Position;\n"
        "in vec2 TexCoord;\n"
        "in vec4 Color;\n"
        "out vec2 Frag_UV;\n"
        "out vec4 Frag_Color;\n"
        "void main() {\n"
        "   Frag_UV = TexCoord;\n"
        "   Frag_Color = Color;\n"
        "   gl_Position = ProjMtx * vec4(Position.xy, 0, 1);\n"
        "}\n";
    static const GLchar *fragment_shader =
        NK_SHADER_VERSION
        "precision mediump float;\n"
        "uniform sampler2D Texture;\n"
        "in vec2 Frag_UV;\n"
        "in vec4 Frag_Color;\n"
        "out vec4 Out_Color;\n"
        "void main(){\n"
        "   Out_Color = Frag_Color * texture(Texture, Frag_UV.st);\n"
        "}\n";

    struct nk_sdl_device *dev = &sdl.ogl;
    nk_buffer_init_default(&dev->cmds);
    dev->prog = glCreateProgram();
    dev->vert_shdr = glCreateShader(GL_VERTEX_SHADER);
    dev->frag_shdr = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(dev->vert_shdr, 1, &vertex_shader, 0);
    glShaderSource(dev->frag_shdr, 1, &fragment_shader, 0);
    glCompileShader(dev->vert_shdr);
    glCompileShader(dev->frag_shdr);
    glGetShaderiv(dev->vert_shdr, GL_COMPILE_STATUS, &status);
    assert(status == GL_TRUE);
    glGetShaderiv(dev->frag_shdr, GL_COMPILE_STATUS, &status);
    assert(status == GL_TRUE);
    glAttachShader(dev->prog, dev->vert_shdr);
    glAttachShader(dev->prog, dev->frag_shdr);
    glLinkProgram(dev->prog);
    glGetProgramiv(dev->prog, GL_LINK_STATUS, &status);
    assert(status == GL_TRUE);

    dev->uniform_tex = glGetUniformLocation(dev->prog, "Texture");
    dev->uniform_proj = glGetUniformLocation(dev->prog, "ProjMtx");
    dev->attrib_pos = glGetAttribLocation(dev->prog, "Position");
    dev->attrib_uv = glGetAttribLocation(dev->prog, "TexCoord");
    dev->attrib_col = glGetAttribLocation(dev->prog, "Color");

    {
        GLsizei vs = sizeof(struct nk_sdl_vertex);
        size_t vp = offsetof(struct nk_sdl_vertex, position);
        size_t vt = offsetof(struct nk_sdl_vertex, uv);
        size_t vc = offsetof(struct nk_sdl_vertex, col);

        glGenBuffers(1, &dev->vbo);
        glGenBuffers(1, &dev->ebo);
        glGenVertexArrays(1, &dev->vao);

        glBindVertexArray(dev->vao);
        glBindBuffer(GL_ARRAY_BUFFER, dev->vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, dev->ebo);

        glEnableVertexAttribArray((GLuint)dev->attrib_pos);
        glEnableVertexAttribArray((GLuint)dev->attrib_uv);
        glEnableVertexAttribArray((GLuint)dev->attrib_col);

        glVertexAttribPointer((GLuint)dev->attrib_pos, 2, GL_FLOAT, GL_FALSE, vs, (void*)vp);
        glVertexAttribPointer((GLuint)dev->attrib_uv, 2, GL_FLOAT, GL_FALSE, vs, (void*)vt);
        glVertexAttribPointer((GLuint)dev->attrib_col, 4, GL_UNSIGNED_BYTE, GL_TRUE, vs, (void*)vc);
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

NK_INTERN void
nk_sdl_device_upload_atlas(const void *image, int width, int height)
{
    struct nk_sdl_device *dev = &sdl.ogl;
    glGenTextures(1, &dev->font_tex);
    glBindTexture(GL_TEXTURE_2D, dev->font_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, (GLsizei)width, (GLsizei)height, 0,
                GL_RGBA, GL_UNSIGNED_BYTE, image);
}

NK_API void
nk_sdl_device_destroy(void)
{
    struct nk_sdl_device *dev = &sdl.ogl;
    glDetachShader(dev->prog, dev->vert_shdr);
    glDetachShader(dev->prog, dev->frag_shdr);
    glDeleteShader(dev->vert_shdr);
    glDeleteShader(dev->frag_shdr);
    glDeleteProgram(dev->prog);
    glDeleteTextures(1, &dev->font_tex);
    glDeleteBuffers(1, &dev->vbo);
    glDeleteBuffers(1, &dev->ebo);
    nk_buffer_free(&dev->cmds);
}

NK_API void
nk_sdl_render(enum nk_anti_aliasing AA, int max_vertex_buffer, int max_element_buffer)
{
    struct nk_sdl_device *dev = &sdl.ogl;
    int width, height;
    int display_width, display_height;
    struct nk_vec2 scale;
    GLfloat ortho[4][4] = {
        {  2.0f,  0.0f,  0.0f, 0.0f },
        {  0.0f, -2.0f,  0.0f, 0.0f },
        {  0.0f,  0.0f, -1.0f, 0.0f },
        { -1.0f,  1.0f,  0.0f, 1.0f },
    };

    /* SDL3: SDL_GetTicks returns Uint64 ms (no SDL_GetTicks64 needed) */
    Uint64 now = SDL_GetTicks();
    sdl.ctx.delta_time_seconds = (float)(now - sdl.time_of_last_frame) / 1000.0f;
    sdl.time_of_last_frame = now;

    SDL_GetWindowSize(sdl.win, &width, &height);
    /* SDL3: SDL_GetWindowSizeInPixels replaces SDL_GL_GetDrawableSize */
    SDL_GetWindowSizeInPixels(sdl.win, &display_width, &display_height);
    ortho[0][0] /= (GLfloat)width;
    ortho[1][1] /= (GLfloat)height;

    scale.x = (float)display_width/(float)width;
    scale.y = (float)display_height/(float)height;

    /* save GL state that we modify */
    GLint last_prog; glGetIntegerv(GL_CURRENT_PROGRAM, &last_prog);
    GLint last_tex;  glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_tex);
    GLint last_abo;  glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_abo);
    GLint last_ebo;  glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &last_ebo);
    GLint last_vao;  glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vao);
    GLint last_blend_src, last_blend_dst;
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &last_blend_src);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &last_blend_dst);
    GLboolean last_blend = glIsEnabled(GL_BLEND);
    GLboolean last_cull  = glIsEnabled(GL_CULL_FACE);
    GLboolean last_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean last_scissor = glIsEnabled(GL_SCISSOR_TEST);
    GLint last_viewport[4]; glGetIntegerv(GL_VIEWPORT, last_viewport);

    /* setup global state */
    glViewport(0, 0, display_width, display_height);
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);
    glActiveTexture(GL_TEXTURE0);

    /* setup program */
    glUseProgram(dev->prog);
    glUniform1i(dev->uniform_tex, 0);
    glUniformMatrix4fv(dev->uniform_proj, 1, GL_FALSE, &ortho[0][0]);
    {
        const struct nk_draw_command *cmd;
        void *vertices, *elements;
        const nk_draw_index *offset = NULL;
        struct nk_buffer vbuf, ebuf;

        glBindVertexArray(dev->vao);
        glBindBuffer(GL_ARRAY_BUFFER, dev->vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, dev->ebo);

        glBufferData(GL_ARRAY_BUFFER, max_vertex_buffer, NULL, GL_STREAM_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, max_element_buffer, NULL, GL_STREAM_DRAW);

        vertices = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
        elements = glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY);
        {
            struct nk_convert_config config;
            static const struct nk_draw_vertex_layout_element vertex_layout[] = {
                {NK_VERTEX_POSITION, NK_FORMAT_FLOAT, NK_OFFSETOF(struct nk_sdl_vertex, position)},
                {NK_VERTEX_TEXCOORD, NK_FORMAT_FLOAT, NK_OFFSETOF(struct nk_sdl_vertex, uv)},
                {NK_VERTEX_COLOR, NK_FORMAT_R8G8B8A8, NK_OFFSETOF(struct nk_sdl_vertex, col)},
                {NK_VERTEX_LAYOUT_END}
            };
            memset(&config, 0, sizeof(config));
            config.vertex_layout = vertex_layout;
            config.vertex_size = sizeof(struct nk_sdl_vertex);
            config.vertex_alignment = NK_ALIGNOF(struct nk_sdl_vertex);
            config.tex_null = dev->tex_null;
            config.circle_segment_count = 22;
            config.curve_segment_count = 22;
            config.arc_segment_count = 22;
            config.global_alpha = 1.0f;
            config.shape_AA = AA;
            config.line_AA = AA;

            nk_buffer_init_fixed(&vbuf, vertices, (nk_size)max_vertex_buffer);
            nk_buffer_init_fixed(&ebuf, elements, (nk_size)max_element_buffer);
            nk_convert(&sdl.ctx, &dev->cmds, &vbuf, &ebuf, &config);
        }
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);

        nk_draw_foreach(cmd, &sdl.ctx, &dev->cmds) {
            if (!cmd->elem_count) continue;
            glBindTexture(GL_TEXTURE_2D, (GLuint)cmd->texture.id);
            glScissor((GLint)(cmd->clip_rect.x * scale.x),
                (GLint)((height - (GLint)(cmd->clip_rect.y + cmd->clip_rect.h)) * scale.y),
                (GLint)(cmd->clip_rect.w * scale.x),
                (GLint)(cmd->clip_rect.h * scale.y));
            glDrawElements(GL_TRIANGLES, (GLsizei)cmd->elem_count,
#ifdef NK_UINT_DRAW_INDEX
                GL_UNSIGNED_INT,
#else
                GL_UNSIGNED_SHORT,
#endif
                offset);
            offset += cmd->elem_count;
        }
        nk_clear(&sdl.ctx);
        nk_buffer_clear(&dev->cmds);
    }

    /* restore GL state */
    glUseProgram((GLuint)last_prog);
    glBindTexture(GL_TEXTURE_2D, (GLuint)last_tex);
    glBindBuffer(GL_ARRAY_BUFFER, (GLuint)last_abo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, (GLuint)last_ebo);
    glBindVertexArray((GLuint)last_vao);
    if (last_blend) glEnable(GL_BLEND); else glDisable(GL_BLEND);
    if (last_cull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    if (last_depth) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (last_scissor) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST);
    glBlendFunc((GLenum)last_blend_src, (GLenum)last_blend_dst);
    glViewport(last_viewport[0], last_viewport[1], last_viewport[2], last_viewport[3]);
}

static void
nk_sdl_clipboard_paste(nk_handle usr, struct nk_text_edit *edit)
{
    /* SDL3: SDL_GetClipboardText returns empty string on failure, never NULL */
    char *text = SDL_GetClipboardText();
    if (text && text[0] != '\0') {
        nk_textedit_paste(edit, text, nk_strlen(text));
    }
    SDL_free(text);
    (void)usr;
}

static void
nk_sdl_clipboard_copy(nk_handle usr, const char *text, int len)
{
    char *str = 0;
    (void)usr;
    if (!len) return;
    str = (char*)malloc((size_t)len+1);
    if (!str) return;
    memcpy(str, text, (size_t)len);
    str[len] = '\0';
    SDL_SetClipboardText(str);
    free(str);
}

NK_API struct nk_context*
nk_sdl_init(SDL_Window *win)
{
    sdl.win = win;
    nk_init_default(&sdl.ctx, 0);
    sdl.ctx.clip.copy = nk_sdl_clipboard_copy;
    sdl.ctx.clip.paste = nk_sdl_clipboard_paste;
    sdl.ctx.clip.userdata = nk_handle_ptr(0);
    nk_sdl_device_create();
    sdl.time_of_last_frame = SDL_GetTicks();
    return &sdl.ctx;
}

NK_API void
nk_sdl_font_stash_begin(struct nk_font_atlas **atlas)
{
    nk_font_atlas_init_default(&sdl.atlas);
    nk_font_atlas_begin(&sdl.atlas);
    *atlas = &sdl.atlas;
}

NK_API void
nk_sdl_font_stash_end(void)
{
    const void *image; int w, h;
    image = nk_font_atlas_bake(&sdl.atlas, &w, &h, NK_FONT_ATLAS_RGBA32);
    nk_sdl_device_upload_atlas(image, w, h);
    nk_font_atlas_end(&sdl.atlas, nk_handle_id((int)sdl.ogl.font_tex), &sdl.ogl.tex_null);
    if (sdl.atlas.default_font)
        nk_style_set_font(&sdl.ctx, &sdl.atlas.default_font->handle);
}

NK_API int
nk_sdl_handle_event(SDL_Event *evt)
{
    struct nk_context *ctx = &sdl.ctx;
    /* SDL3: use SDL_KMOD_CTRL instead of KMOD_CTRL */
    int ctrl_down = SDL_GetModState() & SDL_KMOD_CTRL;

    switch(evt->type)
    {
        /* SDL3: SDL_EVENT_KEY_UP / SDL_EVENT_KEY_DOWN */
        case SDL_EVENT_KEY_UP:
        case SDL_EVENT_KEY_DOWN:
            {
                int down = evt->type == SDL_EVENT_KEY_DOWN;
                /* SDL3: use evt->key.scancode (scancodes preferred in SDL3) */
                switch(evt->key.scancode)
                {
                    case SDL_SCANCODE_RSHIFT:
                    case SDL_SCANCODE_LSHIFT:    nk_input_key(ctx, NK_KEY_SHIFT, down); break;
                    case SDL_SCANCODE_DELETE:     nk_input_key(ctx, NK_KEY_DEL, down); break;
                    case SDL_SCANCODE_RETURN:
                    case SDL_SCANCODE_KP_ENTER:  nk_input_key(ctx, NK_KEY_ENTER, down); break;
                    case SDL_SCANCODE_TAB:        nk_input_key(ctx, NK_KEY_TAB, down); break;
                    case SDL_SCANCODE_BACKSPACE:  nk_input_key(ctx, NK_KEY_BACKSPACE, down); break;
                    case SDL_SCANCODE_HOME:       nk_input_key(ctx, NK_KEY_TEXT_START, down);
                                                  nk_input_key(ctx, NK_KEY_SCROLL_START, down); break;
                    case SDL_SCANCODE_END:        nk_input_key(ctx, NK_KEY_TEXT_END, down);
                                                  nk_input_key(ctx, NK_KEY_SCROLL_END, down); break;
                    case SDL_SCANCODE_PAGEDOWN:   nk_input_key(ctx, NK_KEY_SCROLL_DOWN, down); break;
                    case SDL_SCANCODE_PAGEUP:     nk_input_key(ctx, NK_KEY_SCROLL_UP, down); break;
                    case SDL_SCANCODE_Z:          nk_input_key(ctx, NK_KEY_TEXT_UNDO, down && ctrl_down); break;
                    case SDL_SCANCODE_R:          nk_input_key(ctx, NK_KEY_TEXT_REDO, down && ctrl_down); break;
                    case SDL_SCANCODE_C:          nk_input_key(ctx, NK_KEY_COPY, down && ctrl_down); break;
                    case SDL_SCANCODE_V:          nk_input_key(ctx, NK_KEY_PASTE, down && ctrl_down); break;
                    case SDL_SCANCODE_X:          nk_input_key(ctx, NK_KEY_CUT, down && ctrl_down); break;
                    case SDL_SCANCODE_B:          nk_input_key(ctx, NK_KEY_TEXT_LINE_START, down && ctrl_down); break;
                    case SDL_SCANCODE_E:          nk_input_key(ctx, NK_KEY_TEXT_LINE_END, down && ctrl_down); break;
                    case SDL_SCANCODE_UP:         nk_input_key(ctx, NK_KEY_UP, down); break;
                    case SDL_SCANCODE_DOWN:       nk_input_key(ctx, NK_KEY_DOWN, down); break;
                    case SDL_SCANCODE_A:
                        if (ctrl_down)
                            nk_input_key(ctx, NK_KEY_TEXT_SELECT_ALL, down);
                        break;
                    case SDL_SCANCODE_LEFT:
                        if (ctrl_down)
                            nk_input_key(ctx, NK_KEY_TEXT_WORD_LEFT, down);
                        else nk_input_key(ctx, NK_KEY_LEFT, down);
                        break;
                    case SDL_SCANCODE_RIGHT:
                        if (ctrl_down)
                            nk_input_key(ctx, NK_KEY_TEXT_WORD_RIGHT, down);
                        else nk_input_key(ctx, NK_KEY_RIGHT, down);
                        break;
                    default: break;
                }
            }
            return 1;

        /* SDL3: SDL_EVENT_MOUSE_BUTTON_UP / DOWN */
        case SDL_EVENT_MOUSE_BUTTON_UP:
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            {
                /* SDL3: button coords are float; cast to int for nk_input_button */
                int down = evt->button.down;
                const int x = (int)evt->button.x, y = (int)evt->button.y;
                switch(evt->button.button)
                {
                    case SDL_BUTTON_LEFT:
                        if (evt->button.clicks > 1)
                            nk_input_button(ctx, NK_BUTTON_DOUBLE, x, y, down);
                        nk_input_button(ctx, NK_BUTTON_LEFT, x, y, down); break;
                    case SDL_BUTTON_MIDDLE: nk_input_button(ctx, NK_BUTTON_MIDDLE, x, y, down); break;
                    case SDL_BUTTON_RIGHT:  nk_input_button(ctx, NK_BUTTON_RIGHT, x, y, down); break;
                    default: break;
                }
            }
            return 1;

        /* SDL3: SDL_EVENT_MOUSE_MOTION */
        case SDL_EVENT_MOUSE_MOTION:
            if (ctx->input.mouse.grabbed) {
                int x = (int)ctx->input.mouse.prev.x, y = (int)ctx->input.mouse.prev.y;
                nk_input_motion(ctx, x + (int)evt->motion.xrel, y + (int)evt->motion.yrel);
            }
            else nk_input_motion(ctx, (int)evt->motion.x, (int)evt->motion.y);
            return 1;

        /* SDL3: SDL_EVENT_TEXT_INPUT */
        case SDL_EVENT_TEXT_INPUT:
            {
                nk_glyph glyph;
                memcpy(glyph, evt->text.text, NK_UTF_SIZE);
                nk_input_glyph(ctx, glyph);
            }
            return 1;

        /* SDL3: SDL_EVENT_MOUSE_WHEEL */
        case SDL_EVENT_MOUSE_WHEEL:
            nk_input_scroll(ctx, nk_vec2(evt->wheel.x, evt->wheel.y));
            return 1;
    }
    return 0;
}

NK_API
void nk_sdl_shutdown(void)
{
    nk_font_atlas_clear(&sdl.atlas);
    nk_free(&sdl.ctx);
    nk_sdl_device_destroy();
    memset(&sdl, 0, sizeof(sdl));
}

#endif /* NK_SDL3_GL3_IMPLEMENTATION */
