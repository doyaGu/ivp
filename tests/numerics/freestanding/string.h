#ifndef IVP_FREESTANDING_STRING_INCLUDED
#define IVP_FREESTANDING_STRING_INCLUDED

#include <stddef.h>

extern "C" void *memcpy(void *destination, const void *source, size_t size);
extern "C" void *memset(void *destination, int value, size_t size);

#endif
