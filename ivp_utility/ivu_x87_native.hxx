#ifndef IVP_U_X87_NATIVE_INCLUDED
#define IVP_U_X87_NATIVE_INCLUDED

// Host x87 evaluation for the HARD selections (ivu_x87_config.hxx), used for
// 64-bit precision with nearest rounding while the executing FPU uses the
// same control. IVP_U_Float80 stores the significand at offset 0 and the
// sign/exponent at offset 8, the x87 memory format, so values move with
// fld/fstp tbyte.
//
// IVP_X87_ARITHMETIC HARD covers the operations every x87 implementation
// defines identically: load, store, add, subtract, multiply, divide, square
// root, round to integer and scale. IVP_X87_ARITHMETIC_TRANSCENDENTAL HARD
// covers fsin, fcos, fpatan, f2xm1 and fyl2x, whose last bit differs between
// processor vendors; results then follow the executing processor, as the
// original build's do.
//
// 32-bit MSVC uses __asm, which it does not inline; GCC and Clang (-m32) use
// long double.

#if !(defined(_M_IX86) || defined(__i386__))
#error "HARD x87 selections require a 32-bit x86 target"
#endif

#include <string.h>

#if defined(_MSC_VER)

static IVP_U_Float80 ivp_x87_native_load_float(float value)
{
    IVP_U_Float80 result;
    __asm fld value
    __asm fstp tbyte ptr result
    return result;
}

static IVP_U_Float80 ivp_x87_native_load_double(double value)
{
    IVP_U_Float80 result;
    __asm fld value
    __asm fstp tbyte ptr result
    return result;
}

static float ivp_x87_native_store_float(const IVP_U_Float80 &value)
{
    float result;
    const IVP_U_Float80 *source = &value;
    __asm mov eax, source
    __asm fld tbyte ptr [eax]
    __asm fstp result
    return result;
}

static double ivp_x87_native_store_double(const IVP_U_Float80 &value)
{
    double result;
    const IVP_U_Float80 *source = &value;
    __asm mov eax, source
    __asm fld tbyte ptr [eax]
    __asm fstp result
    return result;
}

// Loads first then second (ST1 = first, ST0 = second) for the operation.
#define IVP_X87_NATIVE_LOAD_PAIR(FIRST, SECOND)                                                                     \
    const IVP_U_Float80 *first_source = &(FIRST), *second_source = &(SECOND);                                        \
    __asm { mov eax, first_source }                                                                                  \
    __asm { fld tbyte ptr [eax] }                                                                                    \
    __asm { mov eax, second_source }                                                                                 \
    __asm { fld tbyte ptr [eax] }

static IVP_U_Float80 ivp_x87_native_add(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    IVP_U_Float80 result;
    IVP_X87_NATIVE_LOAD_PAIR(first, second)
    __asm faddp st(1), st
    __asm fstp tbyte ptr result
    return result;
}

static IVP_U_Float80 ivp_x87_native_subtract(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    IVP_U_Float80 result;
    IVP_X87_NATIVE_LOAD_PAIR(first, second)
    __asm fsubp st(1), st
    __asm fstp tbyte ptr result
    return result;
}

static IVP_U_Float80 ivp_x87_native_multiply(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    IVP_U_Float80 result;
    IVP_X87_NATIVE_LOAD_PAIR(first, second)
    __asm fmulp st(1), st
    __asm fstp tbyte ptr result
    return result;
}

static IVP_U_Float80 ivp_x87_native_divide(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    IVP_U_Float80 result;
    IVP_X87_NATIVE_LOAD_PAIR(first, second)
    __asm fdivp st(1), st
    __asm fstp tbyte ptr result
    return result;
}

// atan(y / x)
static IVP_U_Float80 ivp_x87_native_arctangent(const IVP_U_Float80 &y, const IVP_U_Float80 &x)
{
    IVP_U_Float80 result;
    IVP_X87_NATIVE_LOAD_PAIR(y, x)
    __asm fpatan
    __asm fstp tbyte ptr result
    return result;
}

// y * log2(x)
static IVP_U_Float80 ivp_x87_native_log2_multiply(const IVP_U_Float80 &x, const IVP_U_Float80 &y)
{
    IVP_U_Float80 result;
    IVP_X87_NATIVE_LOAD_PAIR(y, x)
    __asm fyl2x
    __asm fstp tbyte ptr result
    return result;
}

static IVP_U_Float80 ivp_x87_native_scale(const IVP_U_Float80 &value, const IVP_U_Float80 &exponent)
{
    IVP_U_Float80 result;
    IVP_X87_NATIVE_LOAD_PAIR(exponent, value)
    __asm fscale
    __asm fstp tbyte ptr result
    __asm fstp st(0)
    return result;
}

#define IVP_X87_NATIVE_UNARY(NAME, INSTRUCTION)                                                                      \
    static IVP_U_Float80 NAME(const IVP_U_Float80 &value)                                                            \
    {                                                                                                                \
        IVP_U_Float80 result;                                                                                        \
        const IVP_U_Float80 *source = &value;                                                                        \
        __asm { mov eax, source }                                                                                    \
        __asm { fld tbyte ptr [eax] }                                                                                \
        __asm { INSTRUCTION }                                                                                        \
        __asm { fstp tbyte ptr result }                                                                              \
        return result;                                                                                               \
    }

#else

inline long double ivp_x87_native_in(const IVP_U_Float80 &value)
{
    long double result = 0;
    memcpy(&result, &value, 10);
    return result;
}

inline IVP_U_Float80 ivp_x87_native_out(long double value)
{
    IVP_U_Float80 result;
    memcpy(&result, &value, 10);
    return result;
}

inline IVP_U_Float80 ivp_x87_native_load_float(float value) { return ivp_x87_native_out(value); }
inline IVP_U_Float80 ivp_x87_native_load_double(double value) { return ivp_x87_native_out(value); }

inline float ivp_x87_native_store_float(const IVP_U_Float80 &value)
{
    volatile float result = static_cast<float>(ivp_x87_native_in(value)); // one fstp: no double rounding
    return result;
}

inline double ivp_x87_native_store_double(const IVP_U_Float80 &value)
{
    volatile double result = static_cast<double>(ivp_x87_native_in(value));
    return result;
}

inline IVP_U_Float80 ivp_x87_native_add(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_native_out(ivp_x87_native_in(first) + ivp_x87_native_in(second));
}

inline IVP_U_Float80 ivp_x87_native_subtract(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_native_out(ivp_x87_native_in(first) - ivp_x87_native_in(second));
}

inline IVP_U_Float80 ivp_x87_native_multiply(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_native_out(ivp_x87_native_in(first) * ivp_x87_native_in(second));
}

inline IVP_U_Float80 ivp_x87_native_divide(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_native_out(ivp_x87_native_in(first) / ivp_x87_native_in(second));
}

// atan(y / x)
inline IVP_U_Float80 ivp_x87_native_arctangent(const IVP_U_Float80 &y, const IVP_U_Float80 &x)
{
    long double result;
    __asm__("fpatan" : "=t"(result) : "0"(ivp_x87_native_in(x)), "u"(ivp_x87_native_in(y)) : "st(1)");
    return ivp_x87_native_out(result);
}

// y * log2(x)
inline IVP_U_Float80 ivp_x87_native_log2_multiply(const IVP_U_Float80 &x, const IVP_U_Float80 &y)
{
    long double result;
    __asm__("fyl2x" : "=t"(result) : "0"(ivp_x87_native_in(x)), "u"(ivp_x87_native_in(y)) : "st(1)");
    return ivp_x87_native_out(result);
}

inline IVP_U_Float80 ivp_x87_native_scale(const IVP_U_Float80 &value, const IVP_U_Float80 &exponent)
{
    long double result;
    __asm__("fscale" : "=t"(result) : "0"(ivp_x87_native_in(value)), "u"(ivp_x87_native_in(exponent)));
    return ivp_x87_native_out(result);
}

#define IVP_X87_NATIVE_UNARY(NAME, INSTRUCTION)                                                                      \
    inline IVP_U_Float80 NAME(const IVP_U_Float80 &value)                                                            \
    {                                                                                                                \
        long double operand = ivp_x87_native_in(value);                                                              \
        __asm__(#INSTRUCTION : "+t"(operand));                                                                       \
        return ivp_x87_native_out(operand);                                                                          \
    }

#endif

IVP_X87_NATIVE_UNARY(ivp_x87_native_square_root, fsqrt)
IVP_X87_NATIVE_UNARY(ivp_x87_native_round_integer, frndint)
IVP_X87_NATIVE_UNARY(ivp_x87_native_sine, fsin)
IVP_X87_NATIVE_UNARY(ivp_x87_native_cosine, fcos)
IVP_X87_NATIVE_UNARY(ivp_x87_native_power2_minus1, f2xm1)

#undef IVP_X87_NATIVE_UNARY

#endif
