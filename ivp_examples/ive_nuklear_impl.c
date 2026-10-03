/* ive_nuklear_impl.c -- Nuklear implementation compilation unit
 *
 * This file provides the single-translation-unit definitions for
 * Nuklear and the SDL3+GL3 backend. Must be compiled as C.
 */

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_UINT_DRAW_INDEX

#define NK_IMPLEMENTATION
#include "nuklear.h"

#define NK_SDL3_GL3_IMPLEMENTATION
#include "nuklear_sdl3_gl3.h"
