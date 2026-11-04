#include "softfloat_arithmetic_cases.hxx"
#include "x87_precision_cases.hxx"
#include <stddef.h>

extern "C" int ivp_softfloat_arithmetic()
{
    return ivp_check_softfloat_arithmetic() + ivp_check_x87_precision();
}

// Freestanding test runtime only. The production library does not supply or
// replace the platform's memory routines.
extern "C" void *memcpy(void *destination, const void *source, size_t size)
{
    volatile unsigned char *out = static_cast<volatile unsigned char *>(destination);
    const volatile unsigned char *in = static_cast<const volatile unsigned char *>(source);
    for(size_t i = 0; i < size; ++i) out[i] = in[i];
    return destination;
}

extern "C" void *memset(void *destination, int value, size_t size)
{
    volatile unsigned char *out = static_cast<volatile unsigned char *>(destination);
    for(size_t i = 0; i < size; ++i) out[i] = static_cast<unsigned char>(value);
    return destination;
}
