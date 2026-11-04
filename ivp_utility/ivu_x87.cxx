// Extended-precision arithmetic, rounding and exception status.

#include <ivu_x87.hxx>

#include <ivp_softfloat.hxx>
#include <assert.h>
#include <new>
#include <string.h>
#if defined(_MSC_VER) && defined(_M_IX86)
#include <float.h>
#endif

typedef char IVP_X87_Status_Size_Check[sizeof(softfloat_status_t) <= 16 ? 1 : -1];

static thread_local bool ivp_x87_control_is_explicit = false;
static thread_local IVP_X87_Control ivp_x87_control =
    {IVP_X87_Arithmetic::PRECISION_64, IVP_X87_Arithmetic::ROUND_NEAREST};

IVP_X87_Control IVP_X87_Control::current()
{
    if(ivp_x87_control_is_explicit) return ivp_x87_control;
    IVP_X87_Control result =
        {IVP_X87_Arithmetic::PRECISION_64, IVP_X87_Arithmetic::ROUND_NEAREST};
#if defined(_MSC_VER) && defined(_M_IX86)
    unsigned int native_control = 0;
    _controlfp_s(&native_control, 0, 0);
    const unsigned int precision = native_control & _MCW_PC;
    result.precision = precision == _PC_24 ? IVP_X87_Arithmetic::PRECISION_24
        : precision == _PC_53 ? IVP_X87_Arithmetic::PRECISION_53 : IVP_X87_Arithmetic::PRECISION_64;
    const unsigned int rounding = native_control & _MCW_RC;
    result.rounding = rounding == _RC_DOWN ? IVP_X87_Arithmetic::ROUND_DOWN
        : rounding == _RC_UP ? IVP_X87_Arithmetic::ROUND_UP
        : rounding == _RC_CHOP ? IVP_X87_Arithmetic::ROUND_TO_ZERO : IVP_X87_Arithmetic::ROUND_NEAREST;
#endif
    return result;
}

void IVP_X87_Control::set(const IVP_X87_Control &control)
{
    ivp_x87_control = control;
    ivp_x87_control_is_explicit = true;
}

void IVP_X87_Control::reset()
{
    ivp_x87_control_is_explicit = false;
}

static extFloat80_t ivp_x87_input(const IVP_U_Float80 &value)
{
    extFloat80_t result;
    result.signif = value.get_significand();
    result.signExp = value.get_sign_exponent();
    return result;
}

static IVP_U_Float80 ivp_x87_output(extFloat80_t value)
{
    return IVP_U_Float80::from_bits(value.signExp, value.signif);
}

IVP_U_Float80::IVP_U_Float80() : significand(0), sign_exponent(0) {}

IVP_U_Float80 IVP_U_Float80::from_bits(uint16_t exponent, uint64_t fraction)
{
    IVP_U_Float80 result;
    result.sign_exponent = exponent;
    result.significand = fraction;
    return result;
}

IVP_U_Float80 IVP_U_Float80::from_bytes(const unsigned char *bytes)
{
    uint64_t fraction = 0;
    for(unsigned i = 0; i < 8; ++i)
        fraction |= static_cast<uint64_t>(bytes[i]) << (i * 8);
    const uint16_t exponent = static_cast<uint16_t>(bytes[8] | (bytes[9] << 8));
    return from_bits(exponent, fraction);
}

void IVP_U_Float80::to_bytes(unsigned char *bytes) const
{
    for(unsigned i = 0; i < 8; ++i)
        bytes[i] = static_cast<unsigned char>(significand >> (i * 8));
    bytes[8] = static_cast<unsigned char>(sign_exponent);
    bytes[9] = static_cast<unsigned char>(sign_exponent >> 8);
}

IVP_X87_Arithmetic::IVP_X87_Arithmetic(Precision precision, Rounding rounding)
{
    softfloat_status_t *status = new(get_status()) softfloat_status_t();
    status->softfloat_exceptionMasks = softfloat_all_exceptions_mask;
    set_precision(precision);
    set_rounding(rounding);
}

IVP_X87_Arithmetic::IVP_X87_Arithmetic(const IVP_X87_Control &control)
    : IVP_X87_Arithmetic(control.precision, control.rounding)
{
}

IVP_X87_Arithmetic::~IVP_X87_Arithmetic()
{
    static_cast<softfloat_status_t *>(get_status())->~softfloat_status_t();
}

void IVP_X87_Arithmetic::set_precision(Precision precision)
{
    assert(precision == PRECISION_24 || precision == PRECISION_53 || precision == PRECISION_64);
    static_cast<softfloat_status_t *>(get_status())->extF80_roundingPrecision =
        precision == PRECISION_24 ? 32 : precision == PRECISION_53 ? 64 : 80;
}

void IVP_X87_Arithmetic::set_rounding(Rounding rounding)
{
    assert(rounding >= ROUND_NEAREST && rounding <= ROUND_TO_ZERO);
    static_cast<softfloat_status_t *>(get_status())->softfloat_roundingMode =
        static_cast<uint8_t>(rounding);
}

unsigned IVP_X87_Arithmetic::get_exception_flags() const
{
    return static_cast<unsigned>(static_cast<const softfloat_status_t *>(get_status())->softfloat_exceptionFlags) & 0x3f;
}

void IVP_X87_Arithmetic::clear_exception_flags()
{
    static_cast<softfloat_status_t *>(get_status())->softfloat_exceptionFlags = 0;
}

IVP_U_Float80 IVP_X87_Arithmetic::load(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return ivp_x87_output(f32_to_extF80(bits, static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::load(double value)
{
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return ivp_x87_output(f64_to_extF80(bits, static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::load(int32_t value)
{
    return ivp_x87_output(i32_to_extF80(value));
}

float IVP_X87_Arithmetic::store_float(const IVP_U_Float80 &value)
{
    const uint32_t bits = extF80_to_f32(ivp_x87_input(value), static_cast<softfloat_status_t *>(get_status()));
    float result;
    memcpy(&result, &bits, sizeof(result));
    return result;
}

double IVP_X87_Arithmetic::store_double(const IVP_U_Float80 &value)
{
    const uint64_t bits = extF80_to_f64(ivp_x87_input(value), static_cast<softfloat_status_t *>(get_status()));
    double result;
    memcpy(&result, &bits, sizeof(result));
    return result;
}

IVP_U_Float80 IVP_X87_Arithmetic::add(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_output(extF80_add(ivp_x87_input(first), ivp_x87_input(second),
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::subtract(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_output(extF80_sub(ivp_x87_input(first), ivp_x87_input(second),
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::multiply(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_output(extF80_mul(ivp_x87_input(first), ivp_x87_input(second),
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::divide(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_output(extF80_div(ivp_x87_input(first), ivp_x87_input(second),
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::square_root(const IVP_U_Float80 &value)
{
    return ivp_x87_output(extF80_sqrt(ivp_x87_input(value), static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::absolute(const IVP_U_Float80 &value)
{
    return IVP_U_Float80::from_bits(value.get_sign_exponent() & 0x7fff, value.get_significand());
}

IVP_U_Float80 IVP_X87_Arithmetic::negate(const IVP_U_Float80 &value)
{
    return IVP_U_Float80::from_bits(value.get_sign_exponent() ^ 0x8000, value.get_significand());
}

IVP_X87_Arithmetic::Relation IVP_X87_Arithmetic::compare(
    const IVP_U_Float80 &first, const IVP_U_Float80 &second, bool quiet)
{
    return static_cast<Relation>(extF80_compare(ivp_x87_input(first), ivp_x87_input(second), quiet,
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::sine(const IVP_U_Float80 &value, bool *out_of_range)
{
    extFloat80_t result = ivp_x87_input(value);
    const int range = fsin(result, *static_cast<softfloat_status_t *>(get_status()));
    if(out_of_range) *out_of_range = range != 0;
    return ivp_x87_output(result);
}

IVP_U_Float80 IVP_X87_Arithmetic::cosine(const IVP_U_Float80 &value, bool *out_of_range)
{
    extFloat80_t result = ivp_x87_input(value);
    const int range = fcos(result, *static_cast<softfloat_status_t *>(get_status()));
    if(out_of_range) *out_of_range = range != 0;
    return ivp_x87_output(result);
}

IVP_U_Float80 IVP_X87_Arithmetic::arctangent(const IVP_U_Float80 &y, const IVP_U_Float80 &x)
{
    return ivp_x87_output(fpatan(ivp_x87_input(x), ivp_x87_input(y),
        *static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::power2_minus1(const IVP_U_Float80 &value)
{
    return ivp_x87_output(f2xm1(ivp_x87_input(value), *static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::log2_multiply(const IVP_U_Float80 &x, const IVP_U_Float80 &y)
{
    return ivp_x87_output(fyl2x(ivp_x87_input(x), ivp_x87_input(y),
        *static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::round_integer(const IVP_U_Float80 &value)
{
    return ivp_x87_output(extF80_roundToInt(ivp_x87_input(value), static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::scale(const IVP_U_Float80 &value, const IVP_U_Float80 &exponent)
{
    return ivp_x87_output(extF80_scale(ivp_x87_input(value), ivp_x87_input(exponent),
        static_cast<softfloat_status_t *>(get_status())));
}

// Constant loads round the internal constants according to RC, independently
// of arithmetic precision, and do not raise the inexact-result flag.
IVP_U_Float80 IVP_X87_Arithmetic::natural_logarithm_of_two() const
{
    const unsigned mode = static_cast<const softfloat_status_t *>(get_status())->softfloat_roundingMode;
    const uint64_t significand = UINT64_C(0xb17217f7d1cf79ac) -
        (mode == ROUND_DOWN || mode == ROUND_TO_ZERO);
    return IVP_U_Float80::from_bits(0x3ffe, significand);
}

IVP_U_Float80 IVP_X87_Arithmetic::logarithm2_of_e() const
{
    const unsigned mode = static_cast<const softfloat_status_t *>(get_status())->softfloat_roundingMode;
    const uint64_t significand = UINT64_C(0xb8aa3b295c17f0bc) -
        (mode == ROUND_DOWN || mode == ROUND_TO_ZERO);
    return IVP_U_Float80::from_bits(0x3fff, significand);
}

IVP_U_Float80 IVP_X87_Arithmetic::exponential(const IVP_U_Float80 &value)
{
    const IVP_U_Float80 exponent = multiply(value, logarithm2_of_e());
    const IVP_U_Float80 integral = round_integer(exponent);
    return scale(add(power2_minus1(subtract(exponent, integral)), load(1.0)), integral);
}
