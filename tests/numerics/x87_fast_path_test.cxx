// The inline fast path of IVP_X87_Arithmetic against the SoftFloat operations
// it short-cuts: results and exception flags must be identical for every
// operand class, including the cases the fast path declines.
// rpcndr.h defines this macro in Windows consumers of the public headers.
#define small char
#include <ivu_x87.hxx>
#undef small
#include <x87_softfloat.hxx>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#if IVP_X87_ARITHMETIC == IVP_X87_HARD
// The host x87 raises its own exceptions, not the object's: compare values.
#define IVP_FLAGS_MATCH(fp, status) true
#else
#define IVP_FLAGS_MATCH(fp, status) \
    ((fp).get_exception_flags() == (static_cast<unsigned>((status).softfloat_exceptionFlags) & 0x3f))
#endif

namespace
{
uint64_t state = 0x9e3779b97f4a7c15ull;

uint64_t next()
{
    uint64_t z = (state += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

uint16_t random_sign() { return (next() & 1) ? 0x8000 : 0; }

int clamp_exponent(int exponent) { return exponent < 1 ? 1 : exponent > 0x7ffe ? 0x7ffe : exponent; }

// Ordinary values, near-cancellation partners, values loaded from float and
// double (exact results and ties), exponent-range ends and special encodings.
IVP_U_Float80 operand(const IVP_U_Float80 *partner)
{
    const uint64_t kind = next() % 100;
    if(partner && kind < 25)
    {
        const int bits = 1 + static_cast<int>(next() % 64);
        const uint64_t mask = bits == 64 ? ~0ull : (1ull << bits) - 1;
        const int exponent = clamp_exponent((partner->get_sign_exponent() & 0x7fff) + static_cast<int>(next() % 3) - 1);
        const uint16_t sign = static_cast<uint16_t>((partner->get_sign_exponent() & 0x8000) ^ ((next() & 3) ? 0 : 0x8000));
        return IVP_U_Float80::from_bits(static_cast<uint16_t>(exponent | sign),
            (partner->get_significand() ^ (next() & mask)) | (1ull << 63));
    }
    if(kind < 55)
        return IVP_U_Float80::from_bits(static_cast<uint16_t>((0x3fff - 64 + next() % 128) | random_sign()),
            next() | (1ull << 63));
    if(kind < 70)
    {
        const uint64_t mantissa = next() >> 11;
        return IVP_U_Float80::from_bits(static_cast<uint16_t>((0x3fff - 30 + next() % 60) | random_sign()),
            (mantissa | (1ull << 52)) << 11);
    }
    if(kind < 80)
        return IVP_U_Float80::from_bits(static_cast<uint16_t>((0x3fff - 20 + next() % 40) | random_sign()),
            ((next() >> 41) | (1ull << 23)) << 40);
    if(kind < 90)
    {
        static const int ends[] = {1, 30, 0x3fff - 1022, 0x3fff - 126, 0x3fff + 1023, 0x3fff + 127, 0x7ffe - 30, 0x7ffe};
        uint64_t significand = next() | (1ull << 63);
        if(next() & 1) significand |= 0x7ff;
        if(next() & 1) significand &= ~0xffffffffffull;
        return IVP_U_Float80::from_bits(
            static_cast<uint16_t>(clamp_exponent(ends[next() % 8] + static_cast<int>(next() % 9) - 4) | random_sign()),
            significand);
    }
    switch(next() % 7)
    {
    case 0: return IVP_U_Float80::from_bits(random_sign(), 0);
    case 1: return IVP_U_Float80::from_bits(random_sign(), next() >> 1);
    case 2: return IVP_U_Float80::from_bits(static_cast<uint16_t>(0x7fff | random_sign()), 1ull << 63);
    case 3: return IVP_U_Float80::from_bits(static_cast<uint16_t>(0x7fff | random_sign()), next() | (3ull << 62));
    case 4: return IVP_U_Float80::from_bits(static_cast<uint16_t>(0x7fff | random_sign()), (next() >> 2) | (1ull << 63) | 1);
    case 5: return IVP_U_Float80::from_bits(static_cast<uint16_t>((1 + next() % 0x7ffe) | random_sign()), next() >> 1);
    default: return IVP_U_Float80::from_bits(random_sign(), next() | (1ull << 63));
    }
}

extFloat80_t raw(const IVP_U_Float80 &value)
{
    extFloat80_t result;
    result.signif = value.get_significand();
    result.signExp = value.get_sign_exponent();
    return result;
}

bool same(const IVP_U_Float80 &value, extFloat80_t expected)
{
    return value.get_significand() == expected.signif && value.get_sign_exponent() == expected.signExp;
}

softfloat_status_t reference_status()
{
    softfloat_status_t status;
    memset(&status, 0, sizeof(status));
    status.softfloat_roundingMode = softfloat_round_near_even;
    status.softfloat_exceptionMasks = softfloat_all_exceptions_mask;
    status.extF80_roundingPrecision = 80;
    return status;
}

int failures = 0;

void check(bool ok, const char *operation, const IVP_U_Float80 &a, const IVP_U_Float80 &b)
{
    if(ok) return;
    if(++failures <= 10)
        std::printf("%s mismatch: %04x:%016llx %04x:%016llx\n", operation, a.get_sign_exponent(),
            static_cast<unsigned long long>(a.get_significand()), b.get_sign_exponent(),
            static_cast<unsigned long long>(b.get_significand()));
}
} // namespace

int main(int argc, char **argv)
{
    const int cases = argc > 1 ? std::atoi(argv[1]) : 400000;
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
    // The host path needs the executing FPU at the same selection.
    const IVP_X87_Control control = {IVP_X87_Arithmetic::PRECISION_64, IVP_X87_Arithmetic::ROUND_NEAREST};
    IVP_X87_Control::set(control);
#endif
    IVP_X87_Arithmetic fp(IVP_X87_Arithmetic::PRECISION_64, IVP_X87_Arithmetic::ROUND_NEAREST);
    for(int i = 0; i < cases; ++i)
    {
        const IVP_U_Float80 a = operand(0);
        const IVP_U_Float80 b = operand(&a);
        softfloat_status_t status;

#define IVP_CHECK_BINARY(NAME, CALL, REFERENCE)                                                      \
        status = reference_status();                                                                 \
        fp.clear_exception_flags();                                                                  \
        check(same(fp.CALL(a, b), REFERENCE(raw(a), raw(b), &status))                                \
            && IVP_FLAGS_MATCH(fp, status), \
            NAME, a, b);
        IVP_CHECK_BINARY("add", add, extF80_add)
        IVP_CHECK_BINARY("subtract", subtract, extF80_sub)
        IVP_CHECK_BINARY("multiply", multiply, extF80_mul)
        IVP_CHECK_BINARY("divide", divide, extF80_div)
#undef IVP_CHECK_BINARY

        status = reference_status();
        fp.clear_exception_flags();
        const float narrow = fp.store_float(a);
        const uint32_t narrow_expected = extF80_to_f32(raw(a), &status);
        uint32_t narrow_bits;
        memcpy(&narrow_bits, &narrow, sizeof(narrow_bits));
        check(narrow_bits == narrow_expected
            && IVP_FLAGS_MATCH(fp, status),
            "store_float", a, a);

        status = reference_status();
        fp.clear_exception_flags();
        const double wide = fp.store_double(a);
        const uint64_t wide_expected = extF80_to_f64(raw(a), &status);
        uint64_t wide_bits;
        memcpy(&wide_bits, &wide, sizeof(wide_bits));
        check(wide_bits == wide_expected
            && IVP_FLAGS_MATCH(fp, status),
            "store_double", a, a);

        const uint32_t single_bits = static_cast<uint32_t>(next());
        float single;
        memcpy(&single, &single_bits, sizeof(single));
        status = reference_status();
        fp.clear_exception_flags();
        check(same(fp.load(single), f32_to_extF80(single_bits, &status))
            && IVP_FLAGS_MATCH(fp, status),
            "load(float)", a, a);

        const uint64_t double_bits = next();
        double value;
        memcpy(&value, &double_bits, sizeof(value));
        status = reference_status();
        fp.clear_exception_flags();
        check(same(fp.load(value), f64_to_extF80(double_bits, &status))
            && IVP_FLAGS_MATCH(fp, status),
            "load(double)", a, a);
    }

    // Other precision and rounding selections do not take the fast path.
    IVP_X87_Arithmetic narrow_precision(IVP_X87_Arithmetic::PRECISION_53, IVP_X87_Arithmetic::ROUND_NEAREST);
    IVP_X87_Arithmetic upward(IVP_X87_Arithmetic::PRECISION_64, IVP_X87_Arithmetic::ROUND_UP);
    for(int i = 0; i < cases / 10; ++i)
    {
        const IVP_U_Float80 a = operand(0);
        const IVP_U_Float80 b = operand(&a);
        softfloat_status_t status = reference_status();
        status.extF80_roundingPrecision = 64;
        check(same(narrow_precision.add(a, b), extF80_add(raw(a), raw(b), &status)), "add (53-bit)", a, b);
        status = reference_status();
        status.softfloat_roundingMode = softfloat_round_up;
        check(same(upward.multiply(a, b), extF80_mul(raw(a), raw(b), &status)), "multiply (upward)", a, b);
    }

    std::printf("fast-path failures: %d\n", failures);
    return failures ? 1 : 0;
}
