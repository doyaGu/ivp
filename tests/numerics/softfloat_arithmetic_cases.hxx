#ifndef IVP_NUMERICS_SOFTFLOAT_ARITHMETIC_CASES_INCLUDED
#define IVP_NUMERICS_SOFTFLOAT_ARITHMETIC_CASES_INCLUDED

#include <x87_softfloat.hxx>


static softfloat_status_t make_status(unsigned precision)
{
    softfloat_status_t status = {};
    status.extF80_roundingPrecision = static_cast<uint8_t>(precision);
    status.softfloat_exceptionMasks = softfloat_all_exceptions_mask;
    status.softfloat_roundingMode = softfloat_round_near_even;
    return status;
}

static bool same(extFloat80_t first, extFloat80_t second)
{
    return first.signExp == second.signExp && first.signif == second.signif;
}

static int check_rounding(unsigned precision)
{
    const unsigned bits = precision == 32 ? 24 : precision == 64 ? 53 : 64;
    softfloat_status_t status = make_status(precision);
    const extFloat80_t one = i32_to_extF80(1);
    extFloat80_t increment = one;
    increment.signExp -= static_cast<uint16_t>(bits + 1);
    int failures = !same(extF80_add(one, increment, &status), one);
    status.softfloat_roundingMode = softfloat_round_max;
    extFloat80_t next = one;
    next.signif += UINT64_C(1) << (64 - bits);
    failures += !same(extF80_add(one, increment, &status), next);
    status.softfloat_roundingMode = softfloat_round_min;
    failures += !same(extF80_add(one, increment, &status), one);
    status.softfloat_roundingMode = softfloat_round_minMag;
    failures += !same(extF80_add(one, increment, &status), one);
    return failures;
}

static int check_conversions()
{
    softfloat_status_t status = make_status(80);
    uint64_t state = UINT64_C(0x23865a73a429bc17);
    int failures = 0;
    for(unsigned i = 0; i < 4096; ++i)
    {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        const uint32_t single = static_cast<uint32_t>(state);
        if((single & 0x7f800000) != 0x7f800000)
            failures += extF80_to_f32(f32_to_extF80(single, &status), &status) != single;
        if((state & UINT64_C(0x7ff0000000000000)) != UINT64_C(0x7ff0000000000000))
            failures += extF80_to_f64(f64_to_extF80(state, &status), &status) != state;
    }
    return failures;
}

static int check_special_values()
{
    softfloat_status_t status = make_status(80);
    const extFloat80_t zero = i32_to_extF80(0);
    const extFloat80_t one = i32_to_extF80(1);
    const extFloat80_t infinity = extF80_div(one, zero, &status);
    int failures = infinity.signExp != 0x7fff || infinity.signif != UINT64_C(0x8000000000000000);
    failures += (status.softfloat_exceptionFlags & softfloat_flag_divbyzero) == 0;
    status = make_status(80);
    const extFloat80_t nan = extF80_div(zero, zero, &status);
    failures += !extF80_isNaN(nan);
    failures += (status.softfloat_exceptionFlags & softfloat_flag_invalid) == 0;
    failures += extF80_compare_quiet(nan, one, &status) != softfloat_relation_unordered;
    status = make_status(80);
    extFloat80_t largest = one;
    largest.signExp = 0x7ffe;
    largest.signif = UINT64_C(0xffffffffffffffff);
    failures += extF80_to_f32(largest, &status) != 0x7f800000;
    failures += (status.softfloat_exceptionFlags & softfloat_flag_overflow) == 0;
    return failures;
}

static int ivp_check_softfloat_arithmetic()
{
    int failures = check_conversions() + check_special_values();
    for(unsigned precision = 32; precision <= 80; precision += precision == 32 ? 32 : 16)
    {
        softfloat_status_t status = make_status(precision);
        failures += check_rounding(precision);
        const extFloat80_t one = i32_to_extF80(1);
        const extFloat80_t two = i32_to_extF80(2);
        const extFloat80_t four = i32_to_extF80(4);
        const extFloat80_t zero = i32_to_extF80(0);
        failures += !same(extF80_add(one, one, &status), two);
        failures += !same(extF80_sub(two, one, &status), one);
        failures += !same(extF80_mul(two, two, &status), four);
        failures += !same(extF80_div(four, two, &status), two);
        failures += !same(extF80_sqrt(four, &status), two);
        extFloat80_t sine = zero;
        failures += fsin(sine, status) != 0 || !same(sine, zero);
        extFloat80_t cosine = zero;
        failures += fcos(cosine, status) != 0 || !same(cosine, one);
        failures += !same(f2xm1(zero, status), zero);
        failures += !same(fyl2x(two, two, status), two);
        const extFloat80_t third = extF80_div(one, i32_to_extF80(3), &status);
        const uint64_t expected = precision == 32 ? UINT64_C(0xaaaaab0000000000)
            : precision == 64 ? UINT64_C(0xaaaaaaaaaaaaa800)
            : UINT64_C(0xaaaaaaaaaaaaaaab);
        failures += third.signExp != 0x3ffd || third.signif != expected;
    }
    return failures;
}


#endif
