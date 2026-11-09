#ifndef IVP_U_X87_FAST_INCLUDED
#define IVP_U_X87_FAST_INCLUDED

// Integer evaluation of the common extended-arithmetic case: 64-bit
// significands, nearest-even rounding, normal or zero operands and a normal or
// zero result. Each function returns false and leaves its result untouched for
// any other case (denormal, infinity, NaN, unnormal or pseudo-denormal
// operands, division by zero, overflow and underflow); the caller then uses
// the general implementation. A rounded result sets `inexact`. Results are
// identical to the general implementation for every case they accept.
//
// Included by ivu_x87.hxx after IVP_U_Float80; usable from C++98.

#include <stdint.h>
#include <string.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

struct IVP_X87_Fast_U128
{
    uint64_t hi;
    uint64_t lo;
};

inline IVP_X87_Fast_U128 ivp_x87_fast_u128(uint64_t hi, uint64_t lo)
{
    IVP_X87_Fast_U128 result;
    result.hi = hi;
    result.lo = lo;
    return result;
}

inline IVP_X87_Fast_U128 ivp_x87_fast_multiply64(uint64_t a, uint64_t b)
{
#if defined(_MSC_VER) && defined(_M_X64)
    IVP_X87_Fast_U128 result;
    result.lo = _umul128(a, b, &result.hi);
    return result;
#elif defined(__SIZEOF_INT128__) && (defined(__x86_64__) || defined(__aarch64__))
    // Native 64x64->128 multiplication on these targets; no runtime helper.
    __extension__ typedef unsigned __int128 ivp_x87_fast_wide;
    const ivp_x87_fast_wide product = static_cast<ivp_x87_fast_wide>(a) * b;
    return ivp_x87_fast_u128(static_cast<uint64_t>(product >> 64), static_cast<uint64_t>(product));
#else
    const uint64_t a_lo = static_cast<uint32_t>(a), a_hi = a >> 32;
    const uint64_t b_lo = static_cast<uint32_t>(b), b_hi = b >> 32;
    const uint64_t lo_lo = a_lo * b_lo, lo_hi = a_lo * b_hi, hi_lo = a_hi * b_lo, hi_hi = a_hi * b_hi;
    const uint64_t middle = (lo_lo >> 32) + static_cast<uint32_t>(lo_hi) + static_cast<uint32_t>(hi_lo);
    return ivp_x87_fast_u128(hi_hi + (lo_hi >> 32) + (hi_lo >> 32) + (middle >> 32),
        (middle << 32) | static_cast<uint32_t>(lo_lo));
#endif
}

inline int ivp_x87_fast_leading_zeros(uint64_t value) // value != 0
{
#if defined(_MSC_VER) && defined(_M_X64)
    unsigned long index;
    _BitScanReverse64(&index, value);
    return 63 - static_cast<int>(index);
#elif defined(_MSC_VER)
    unsigned long index;
    if(_BitScanReverse(&index, static_cast<unsigned long>(value >> 32))) return 31 - static_cast<int>(index);
    _BitScanReverse(&index, static_cast<unsigned long>(value));
    return 63 - static_cast<int>(index);
#elif defined(__GNUC__)
    return __builtin_clzll(value);
#else
    int count = 0;
    while(!(value & (UINT64_C(1) << 63)))
    {
        value <<= 1;
        ++count;
    }
    return count;
#endif
}

// (hi:lo) / divisor for a divisor with its top bit set and hi < divisor.
inline uint64_t ivp_x87_fast_divide128(uint64_t hi, uint64_t lo, uint64_t divisor, uint64_t *remainder)
{
#if defined(_MSC_VER) && defined(_M_X64) && _MSC_VER >= 1920
    return _udiv128(hi, lo, divisor, remainder);
#elif defined(__GNUC__) && defined(__x86_64__)
    uint64_t quotient, rest;
    __asm__("divq %4" : "=a"(quotient), "=d"(rest) : "a"(lo), "d"(hi), "rm"(divisor));
    *remainder = rest;
    return quotient;
#else
    // Two 64/32 steps with 32-bit digits (Knuth algorithm D).
    const uint64_t base = UINT64_C(1) << 32;
    const uint64_t d1 = divisor >> 32, d0 = divisor & 0xffffffffu;
    const uint64_t n1 = lo >> 32, n0 = lo & 0xffffffffu;
    uint64_t q1 = hi / d1, rhat = hi - q1 * d1;
    while(q1 >= base || q1 * d0 > base * rhat + n1)
    {
        --q1;
        rhat += d1;
        if(rhat >= base) break;
    }
    const uint64_t partial = hi * base + n1 - q1 * divisor;
    uint64_t q0 = partial / d1;
    rhat = partial - q0 * d1;
    while(q0 >= base || q0 * d0 > base * rhat + n0)
    {
        --q0;
        rhat += d1;
        if(rhat >= base) break;
    }
    *remainder = partial * base + n0 - q0 * divisor;
    return q1 * base + q0;
#endif
}

// value >> count, every bit shifted out ORed into bit 0.
inline IVP_X87_Fast_U128 ivp_x87_fast_shift_right_jam(IVP_X87_Fast_U128 value, int count)
{
    if(count == 0) return value;
    if(count < 64)
        return ivp_x87_fast_u128(value.hi >> count, (value.lo >> count) | (value.hi << (64 - count))
            | static_cast<uint64_t>((value.lo << (64 - count)) != 0));
    if(count == 64) return ivp_x87_fast_u128(0, value.hi | static_cast<uint64_t>(value.lo != 0));
    if(count < 128)
        return ivp_x87_fast_u128(0, (value.hi >> (count - 64))
            | static_cast<uint64_t>(((value.hi << (128 - count)) | value.lo) != 0));
    return ivp_x87_fast_u128(0, static_cast<uint64_t>((value.hi | value.lo) != 0));
}

inline bool ivp_x87_fast_is_normal(const IVP_U_Float80 &value)
{
    return static_cast<unsigned>((value.get_sign_exponent() & 0x7fff) - 1) < 0x7ffeu
        && (value.get_significand() >> 63) != 0;
}

inline bool ivp_x87_fast_is_zero(const IVP_U_Float80 &value)
{
    return (value.get_sign_exponent() & 0x7fff) == 0 && value.get_significand() == 0;
}

// Normal or zero: the operand classes whose results the fast path can form.
inline bool ivp_x87_fast_is_simple(const IVP_U_Float80 &value)
{
    return ivp_x87_fast_is_normal(value) || ivp_x87_fast_is_zero(value);
}

// Rounds `significand` with the 64 bits below it in `rest` (top bit = one half)
// and packs it; false outside the normal exponent range.
inline bool ivp_x87_fast_round_pack(unsigned sign, int exponent, uint64_t significand, uint64_t rest,
    IVP_U_Float80 *result, bool *inexact)
{
    if(exponent <= 0) return false;
    const uint64_t half = UINT64_C(1) << 63;
    // Branch-free: the decision is data-dependent and poorly predicted.
    const uint64_t up = static_cast<uint64_t>(rest > half) | (static_cast<uint64_t>(rest == half) & significand);
    significand += up;
    const uint64_t carry = static_cast<uint64_t>(significand == 0) & up; // all ones + 1: next binade
    significand |= carry << 63;
    exponent += static_cast<int>(carry);
    if(exponent >= 0x7fff) return false;
    *result = IVP_U_Float80::from_bits(static_cast<uint16_t>(exponent | (sign << 15)), significand);
    *inexact = *inexact || rest != 0;
    return true;
}

// a + (b with its sign flipped when negate_second): operands normal.
inline bool ivp_x87_fast_add(const IVP_U_Float80 &a, const IVP_U_Float80 &b, unsigned negate_second,
    IVP_U_Float80 *result, bool *inexact)
{
    const uint16_t a_se = a.get_sign_exponent();
    const uint16_t b_se = static_cast<uint16_t>(b.get_sign_exponent() ^ (negate_second << 15));
    if(!ivp_x87_fast_is_normal(a) || !ivp_x87_fast_is_normal(b))
    {
        // x + 0 = x; zero sums keep a common sign and are +0 otherwise.
        if(!ivp_x87_fast_is_simple(a) || !ivp_x87_fast_is_simple(b)) return false;
        if(ivp_x87_fast_is_zero(b))
            *result = ivp_x87_fast_is_zero(a)
                ? IVP_U_Float80::from_bits(static_cast<uint16_t>(a_se & b_se & 0x8000), 0) : a;
        else
            *result = IVP_U_Float80::from_bits(b_se, b.get_significand());
        return true;
    }
    const int a_exp = a_se & 0x7fff, b_exp = b_se & 0x7fff;
    const uint64_t a_sig = a.get_significand(), b_sig = b.get_significand();
    // Order by magnitude without a branch.
    const uint64_t swap = 0 - static_cast<uint64_t>((a_exp < b_exp) | ((a_exp == b_exp) & (a_sig < b_sig)));
    const uint64_t big_sig = (a_sig & ~swap) | (b_sig & swap), small_sig = (b_sig & ~swap) | (a_sig & swap);
    const int big_exp = static_cast<int>((a_exp & ~swap) | (b_exp & swap));
    const int small_exp = static_cast<int>((b_exp & ~swap) | (a_exp & swap));
    const unsigned sign = static_cast<unsigned>(((a_se & ~swap) | (b_se & swap)) >> 15) & 1;
    const int gap = big_exp - small_exp;
    if(!((a_se ^ b_se) & 0x8000))
    {
        IVP_X87_Fast_U128 aligned;
        if(gap < 64)
        {
            // Branch-free alignment: (x << 1) << (63 - gap) is x << (64 - gap), and 0 for gap 0.
            const uint64_t lo_in = small_sig << 63, hi_in = small_sig >> 1;
            aligned.hi = hi_in >> gap;
            aligned.lo = (lo_in >> gap) | ((hi_in << 1) << (63 - gap))
                | static_cast<uint64_t>(((lo_in << 1) << (63 - gap)) != 0);
        }
        else
            aligned = ivp_x87_fast_shift_right_jam(ivp_x87_fast_u128(small_sig >> 1, small_sig << 63), gap);
        const uint64_t lo = (big_sig << 63) + aligned.lo;
        const uint64_t hi = (big_sig >> 1) + aligned.hi + static_cast<uint64_t>(lo < (big_sig << 63));
        const uint64_t carry = hi >> 63, keep = carry - 1; // keep: no carry, shift left by one
        const uint64_t significand = (hi & ~keep) | (((hi << 1) | (lo >> 63)) & keep);
        const uint64_t rest = (lo & ~keep) | ((lo << 1) & keep);
        return ivp_x87_fast_round_pack(sign, big_exp + static_cast<int>(carry), significand, rest, result, inexact);
    }
    if(gap == 0 && big_sig == small_sig)
    {
        *result = IVP_U_Float80::from_bits(0, 0); // exact cancellation: +0 when rounding to nearest
        return true;
    }
    IVP_X87_Fast_U128 aligned;
    if(gap < 64)
    {
        aligned.hi = small_sig >> gap;
        aligned.lo = (small_sig << 1) << (63 - gap);
    }
    else
        aligned = ivp_x87_fast_shift_right_jam(ivp_x87_fast_u128(small_sig, 0), gap);
    uint64_t lo = 0 - aligned.lo;
    uint64_t hi = big_sig - aligned.hi - static_cast<uint64_t>(aligned.lo != 0);
    const int shift = hi ? ivp_x87_fast_leading_zeros(hi) : 64 + ivp_x87_fast_leading_zeros(lo);
    if(shift >= 64)
    {
        hi = lo << (shift - 64);
        lo = 0;
    }
    else if(shift)
    {
        hi = (hi << shift) | (lo >> (64 - shift));
        lo <<= shift;
    }
    return ivp_x87_fast_round_pack(sign, big_exp - shift, hi, lo, result, inexact);
}

inline bool ivp_x87_fast_multiply(const IVP_U_Float80 &a, const IVP_U_Float80 &b, IVP_U_Float80 *result,
    bool *inexact)
{
    const unsigned sign = static_cast<unsigned>((a.get_sign_exponent() ^ b.get_sign_exponent()) >> 15);
    if(!ivp_x87_fast_is_normal(a) || !ivp_x87_fast_is_normal(b))
    {
        if(!ivp_x87_fast_is_simple(a) || !ivp_x87_fast_is_simple(b)) return false;
        *result = IVP_U_Float80::from_bits(static_cast<uint16_t>(sign << 15), 0); // a zero factor
        return true;
    }
    const int exponent = (a.get_sign_exponent() & 0x7fff) + (b.get_sign_exponent() & 0x7fff) - 0x3fff;
    const IVP_X87_Fast_U128 product = ivp_x87_fast_multiply64(a.get_significand(), b.get_significand());
    const uint64_t carry = product.hi >> 63, keep = carry - 1;
    const uint64_t significand = (product.hi & ~keep) | (((product.hi << 1) | (product.lo >> 63)) & keep);
    const uint64_t rest = (product.lo & ~keep) | ((product.lo << 1) & keep);
    return ivp_x87_fast_round_pack(sign, exponent + static_cast<int>(carry), significand, rest, result, inexact);
}

inline bool ivp_x87_fast_divide(const IVP_U_Float80 &a, const IVP_U_Float80 &b, IVP_U_Float80 *result,
    bool *inexact)
{
    const unsigned sign = static_cast<unsigned>((a.get_sign_exponent() ^ b.get_sign_exponent()) >> 15);
    if(!ivp_x87_fast_is_normal(b)) return false;
    if(!ivp_x87_fast_is_normal(a))
    {
        if(!ivp_x87_fast_is_zero(a)) return false;
        *result = IVP_U_Float80::from_bits(static_cast<uint16_t>(sign << 15), 0); // 0 / x
        return true;
    }
    int exponent = (a.get_sign_exponent() & 0x7fff) - (b.get_sign_exponent() & 0x7fff) + 0x3fff;
    const uint64_t a_sig = a.get_significand(), b_sig = b.get_significand();
    uint64_t hi, lo;
    if(a_sig >= b_sig)
    {
        hi = a_sig >> 1;
        lo = a_sig << 63;
    }
    else
    {
        hi = a_sig;
        lo = 0;
        --exponent;
    }
    uint64_t remainder;
    const uint64_t quotient = ivp_x87_fast_divide128(hi, lo, b_sig, &remainder);
    const uint64_t half = UINT64_C(1) << 63;
    const uint64_t rest = remainder == 0 ? 0
        : remainder > b_sig - remainder ? half + 1
        : remainder == b_sig - remainder ? half : 1;
    return ivp_x87_fast_round_pack(sign, exponent, quotient, rest, result, inexact);
}

inline bool ivp_x87_fast_load_float(float value, IVP_U_Float80 *result)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    const unsigned exponent = (bits >> 23) & 0xff;
    if(!(bits & 0x7fffffffu))
    {
        *result = IVP_U_Float80::from_bits(static_cast<uint16_t>((bits >> 31) << 15), 0);
        return true;
    }
    if(exponent - 1 >= 0xfe) return false;
    *result = IVP_U_Float80::from_bits(static_cast<uint16_t>((exponent - 127 + 0x3fff) | ((bits >> 31) << 15)),
        static_cast<uint64_t>((bits & 0x7fffffu) | 0x800000u) << 40);
    return true;
}

inline bool ivp_x87_fast_load_double(double value, IVP_U_Float80 *result)
{
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    const unsigned exponent = static_cast<unsigned>(bits >> 52) & 0x7ff;
    if(!(bits << 1))
    {
        *result = IVP_U_Float80::from_bits(static_cast<uint16_t>((bits >> 63) << 15), 0);
        return true;
    }
    if(exponent - 1 >= 0x7fe) return false;
    *result = IVP_U_Float80::from_bits(static_cast<uint16_t>((exponent - 1023 + 0x3fff) | ((bits >> 63) << 15)),
        ((bits & UINT64_C(0xfffffffffffff)) | (UINT64_C(1) << 52)) << 11);
    return true;
}

// Rounds the significand to `bits` bits for an IEEE format with the given bias
// and maximum biased exponent; false outside its normal range.
inline bool ivp_x87_fast_narrow(const IVP_U_Float80 &value, int bits, int bias, int maximum, uint64_t *mantissa,
    int *exponent, bool *inexact)
{
    if(!ivp_x87_fast_is_normal(value)) return false;
    int biased = (value.get_sign_exponent() & 0x7fff) - 0x3fff + bias;
    if(biased <= 0) return false;
    const uint64_t significand = value.get_significand();
    uint64_t kept = significand >> (64 - bits);
    const uint64_t rest = significand << bits;
    const uint64_t half = UINT64_C(1) << 63;
    kept += static_cast<uint64_t>(rest > half) | (static_cast<uint64_t>(rest == half) & kept);
    if(kept >> bits)
    {
        kept >>= 1;
        ++biased;
    }
    if(biased >= maximum) return false;
    *mantissa = kept;
    *exponent = biased;
    *inexact = *inexact || rest != 0;
    return true;
}

inline bool ivp_x87_fast_store_float(const IVP_U_Float80 &value, float *result, bool *inexact)
{
    uint64_t mantissa;
    int exponent;
    bool rounded = false;
    if(ivp_x87_fast_is_zero(value))
    {
        const uint32_t bits = static_cast<uint32_t>(value.get_sign_exponent() >> 15) << 31;
        memcpy(result, &bits, sizeof(bits));
        return true;
    }
    if(!ivp_x87_fast_narrow(value, 24, 127, 0xff, &mantissa, &exponent, &rounded)) return false;
    const uint32_t bits = (static_cast<uint32_t>(value.get_sign_exponent() >> 15) << 31)
        | (static_cast<uint32_t>(exponent) << 23) | static_cast<uint32_t>(mantissa & 0x7fffffu);
    memcpy(result, &bits, sizeof(bits));
    *inexact = *inexact || rounded;
    return true;
}

inline bool ivp_x87_fast_store_double(const IVP_U_Float80 &value, double *result, bool *inexact)
{
    uint64_t mantissa;
    int exponent;
    bool rounded = false;
    if(ivp_x87_fast_is_zero(value))
    {
        const uint64_t bits = static_cast<uint64_t>(value.get_sign_exponent() >> 15) << 63;
        memcpy(result, &bits, sizeof(bits));
        return true;
    }
    if(!ivp_x87_fast_narrow(value, 53, 1023, 0x7ff, &mantissa, &exponent, &rounded)) return false;
    const uint64_t bits = (static_cast<uint64_t>(value.get_sign_exponent() >> 15) << 63)
        | (static_cast<uint64_t>(exponent) << 52) | (mantissa & UINT64_C(0xfffffffffffff));
    memcpy(result, &bits, sizeof(bits));
    *inexact = *inexact || rounded;
    return true;
}

#endif
