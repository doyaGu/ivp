/* Inject shader failures into the real renderer initialization path. */
#include <SDL3/SDL.h>
#include <glad/glad.h>
#include <stdio.h>
#include <string.h>

static int fail_compile, fail_link, links;
static int shaders_deleted, programs_deleted, windows_deleted, contexts_deleted, quits;
static char fake_window, fake_context;
static bool test_init(SDL_InitFlags flags) { (void)flags; return true; }
static bool test_attribute(SDL_GLAttr attr, int value) { (void)attr; (void)value; return true; }
static SDL_Window *test_window(const char *title, int w, int h, SDL_WindowFlags flags)
{ (void)title; (void)w; (void)h; (void)flags; return (SDL_Window *)&fake_window; }
static SDL_GLContext test_context(SDL_Window *win) { (void)win; return (SDL_GLContext)&fake_context; }
static void test_destroy_window(SDL_Window *win) { (void)win; ++windows_deleted; }
static bool test_destroy_context(SDL_GLContext context) { (void)context; ++contexts_deleted; return true; }
static void test_quit(void) { ++quits; }
static bool test_swap(int interval) { (void)interval; return true; }
static int test_loader(GLADloadproc proc) { (void)proc; return 1; }

#define SDL_Init test_init
#define SDL_GL_SetAttribute test_attribute
#define SDL_CreateWindow test_window
#define SDL_GL_CreateContext test_context
#define SDL_DestroyWindow test_destroy_window
#define SDL_GL_DestroyContext test_destroy_context
#define SDL_Quit test_quit
#define SDL_GL_SetSwapInterval test_swap
#define gladLoadGLLoader test_loader
#include "../../ivp_examples/render/render.c"

static GLuint APIENTRY create_shader(GLenum type) { (void)type; return 10; }
static void APIENTRY shader_source(GLuint shader, GLsizei count, const GLchar *const *strings, const GLint *lengths)
{ (void)shader; (void)count; (void)strings; (void)lengths; }
static void APIENTRY compile(GLuint shader) { (void)shader; }
static void APIENTRY shader_status(GLuint shader, GLenum pname, GLint *status)
{ (void)shader; (void)pname; *status = !fail_compile; }
static void APIENTRY info_log(GLuint object, GLsizei size, GLsizei *length, GLchar *log)
{ (void)object; (void)length; if (size) log[0] = '\0'; }
static void APIENTRY delete_shader(GLuint shader) { if (shader) ++shaders_deleted; }
static GLuint APIENTRY create_program(void) { return 20 + links; }
static void APIENTRY attach(GLuint program, GLuint shader) { (void)program; (void)shader; }
static void APIENTRY link(GLuint program) { (void)program; ++links; }
static void APIENTRY program_status(GLuint program, GLenum pname, GLint *status)
{ (void)program; (void)pname; *status = links != fail_link; }
static void APIENTRY delete_program(GLuint program) { if (program) ++programs_deleted; }
static void APIENTRY delete_buffers(GLsizei n, const GLuint *buffers) { (void)n; (void)buffers; }

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
    return 1; } } while (0)

int main(void)
{
    glad_glCreateShader = create_shader;
    glad_glShaderSource = shader_source;
    glad_glCompileShader = compile;
    glad_glGetShaderiv = shader_status;
    glad_glGetShaderInfoLog = info_log;
    glad_glDeleteShader = delete_shader;
    glad_glCreateProgram = create_program;
    glad_glAttachShader = attach;
    glad_glLinkProgram = link;
    glad_glGetProgramiv = program_status;
    glad_glGetProgramInfoLog = info_log;
    glad_glDeleteProgram = delete_program;
    glad_glDeleteBuffers = delete_buffers;
    glad_glDeleteVertexArrays = delete_buffers;

    ivp_render_config_t cfg = {640, 480, "failure test", false};
    fail_compile = 1;
    CHECK(ivp_renderer_create(&cfg) == NULL);
    CHECK(shaders_deleted == 2 && programs_deleted == 1);
    CHECK(windows_deleted == 1 && contexts_deleted == 1 && quits == 1);

    fail_compile = 0;
    fail_link = 1;
    CHECK(ivp_renderer_create(&cfg) == NULL);
    CHECK(shaders_deleted == 4 && programs_deleted == 2);
    CHECK(windows_deleted == 2 && contexts_deleted == 2 && quits == 2);

    links = 0;
    fail_link = 2; /* HUD program fails after the 3D program succeeds. */
    CHECK(ivp_renderer_create(&cfg) == NULL);
    CHECK(shaders_deleted == 8 && programs_deleted == 4);
    CHECK(windows_deleted == 3 && contexts_deleted == 3 && quits == 3);
    return 0;
}
