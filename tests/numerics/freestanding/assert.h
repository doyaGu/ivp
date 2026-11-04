#ifndef IVP_FREESTANDING_ASSERT_INCLUDED
#define IVP_FREESTANDING_ASSERT_INCLUDED

#ifdef NDEBUG
#define assert(expression) ((void)0)
#else
#define assert(expression) ((expression) ? (void)0 : __builtin_trap())
#endif

#endif
