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
#if defined(_MSC_VER) && defined(_M_IX86)
static thread_local unsigned int ivp_x87_saved_native_control = 0;
#endif

IVP_X87_Control IVP_X87_Control::current()
{
    IVP_X87_Control result = ivp_x87_control;
    if (!ivp_x87_control_is_explicit)
    {
        result.precision = IVP_X87_Arithmetic::PRECISION_64;
        result.rounding = IVP_X87_Arithmetic::ROUND_NEAREST;
    }
#if defined(_MSC_VER) && defined(_M_IX86)
    // Native callers can change the control word after a software selection.
    // The executing FPU remains authoritative for the surrounding expressions.
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
#if defined(_MSC_VER) && defined(_M_IX86)
    // Ordinary expressions in the surrounding engine still execute on the
    // native FPU. Keep their precision and rounding in step with the software.
    unsigned int ignored = 0;
    if (!ivp_x87_control_is_explicit)
        _controlfp_s(&ivp_x87_saved_native_control, 0, 0);
    const unsigned int precision = control.precision == IVP_X87_Arithmetic::PRECISION_24 ? _PC_24 :
        control.precision == IVP_X87_Arithmetic::PRECISION_53 ? _PC_53 : _PC_64;
    const unsigned int rounding = control.rounding == IVP_X87_Arithmetic::ROUND_DOWN ? _RC_DOWN :
        control.rounding == IVP_X87_Arithmetic::ROUND_UP ? _RC_UP :
        control.rounding == IVP_X87_Arithmetic::ROUND_TO_ZERO ? _RC_CHOP : _RC_NEAR;
    _controlfp_s(&ignored, precision | rounding, _MCW_PC | _MCW_RC);
#endif
    ivp_x87_control = control;
    ivp_x87_control_is_explicit = true;
}

void IVP_X87_Control::reset()
{
#if defined(_MSC_VER) && defined(_M_IX86)
    if (ivp_x87_control_is_explicit)
    {
        unsigned int ignored = 0;
        _controlfp_s(&ignored, ivp_x87_saved_native_control, _MCW_PC | _MCW_RC);
    }
#endif
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
    : fast_path(false), fast_inexact(false), native_path(false)
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

void IVP_X87_Arithmetic::set_precision(Precision precision)
{
    assert(precision == PRECISION_24 || precision == PRECISION_53 || precision == PRECISION_64);
    static_cast<softfloat_status_t *>(get_status())->extF80_roundingPrecision =
        precision == PRECISION_24 ? 32 : precision == PRECISION_53 ? 64 : 80;
    update_fast_path();
}

void IVP_X87_Arithmetic::set_rounding(Rounding rounding)
{
    assert(rounding >= ROUND_NEAREST && rounding <= ROUND_TO_ZERO);
    static_cast<softfloat_status_t *>(get_status())->softfloat_roundingMode =
        static_cast<uint8_t>(rounding);
    update_fast_path();
}

void IVP_X87_Arithmetic::update_fast_path()
{
    const softfloat_status_t *status = static_cast<const softfloat_status_t *>(get_status());
    fast_path = status->extF80_roundingPrecision == 80
        && status->softfloat_roundingMode == softfloat_round_near_even;
#if (IVP_X87_ARITHMETIC == IVP_X87_HARD || IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD)
    // The host evaluates with its own control word: use it only when it agrees.
    unsigned short control;
#if defined(_MSC_VER)
    __asm fnstcw control
#else
    __asm__ __volatile__("fnstcw %0" : "=m"(control));
#endif
    native_path = fast_path && (control & 0x0f00) == 0x0300; // PC = 64-bit, RC = nearest
#endif
}

unsigned IVP_X87_Arithmetic::get_exception_flags() const
{
    return (static_cast<unsigned>(static_cast<const softfloat_status_t *>(get_status())->softfloat_exceptionFlags) & 0x3f)
        | (fast_inexact ? static_cast<unsigned>(EXCEPTION_INEXACT_RESULT) : 0u);
}

void IVP_X87_Arithmetic::clear_exception_flags()
{
    static_cast<softfloat_status_t *>(get_status())->softfloat_exceptionFlags = 0;
    fast_inexact = false;
}

IVP_U_Float80 IVP_X87_Arithmetic::load_general(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return ivp_x87_output(f32_to_extF80(bits, static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::load_general(double value)
{
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return ivp_x87_output(f64_to_extF80(bits, static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::load(int32_t value)
{
    return ivp_x87_output(i32_to_extF80(value));
}

float IVP_X87_Arithmetic::store_float_general(const IVP_U_Float80 &value)
{
    const uint32_t bits = extF80_to_f32(ivp_x87_input(value), static_cast<softfloat_status_t *>(get_status()));
    float result;
    memcpy(&result, &bits, sizeof(result));
    return result;
}

double IVP_X87_Arithmetic::store_double_general(const IVP_U_Float80 &value)
{
    const uint64_t bits = extF80_to_f64(ivp_x87_input(value), static_cast<softfloat_status_t *>(get_status()));
    double result;
    memcpy(&result, &bits, sizeof(result));
    return result;
}

#if defined(_MSC_VER) && defined(_M_IX86)
// Only transport uses the native ABI; arithmetic remains in the integer
// numerical backend. The byte encoding avoids depending on object padding.
static __declspec(naked) double __cdecl ivp_x87_return_double(const unsigned char *)
{
    __asm {
        mov eax, [esp+4]
        fld tbyte ptr [eax]
        ret
    }
}

static __declspec(naked) float __cdecl ivp_x87_return_float(const unsigned char *)
{
    __asm {
        mov eax, [esp+4]
        fld tbyte ptr [eax]
        ret
    }
}
#endif

double IVP_X87_Arithmetic::return_double(const IVP_U_Float80 &value)
{
#if defined(_MSC_VER) && defined(_M_IX86)
    unsigned char bytes[10];
    value.to_bytes(bytes);
    return ivp_x87_return_double(bytes);
#else
    return store_double(value);
#endif
}

float IVP_X87_Arithmetic::return_float(const IVP_U_Float80 &value)
{
#if defined(_MSC_VER) && defined(_M_IX86)
    unsigned char bytes[10];
    value.to_bytes(bytes);
    return ivp_x87_return_float(bytes);
#else
    return store_float(value);
#endif
}

IVP_U_Float80 IVP_X87_Arithmetic::add_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_output(extF80_add(ivp_x87_input(first), ivp_x87_input(second),
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::subtract_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_output(extF80_sub(ivp_x87_input(first), ivp_x87_input(second),
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::multiply_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_output(extF80_mul(ivp_x87_input(first), ivp_x87_input(second),
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::divide_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return ivp_x87_output(extF80_div(ivp_x87_input(first), ivp_x87_input(second),
        static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::square_root(const IVP_U_Float80 &value)
{
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
    if(native_path) return ivp_x87_native_square_root(value);
#endif
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
#if IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
    if(native_path)
    {
        if(out_of_range) *out_of_range = (value.get_sign_exponent() & 0x7fff) >= 0x3fff + 63;
        return ivp_x87_native_sine(value);
    }
#endif
    extFloat80_t result = ivp_x87_input(value);
    const int range = fsin(result, *static_cast<softfloat_status_t *>(get_status()));
    if(out_of_range) *out_of_range = range != 0;
    return ivp_x87_output(result);
}

IVP_U_Float80 IVP_X87_Arithmetic::cosine(const IVP_U_Float80 &value, bool *out_of_range)
{
#if IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
    if(native_path)
    {
        if(out_of_range) *out_of_range = (value.get_sign_exponent() & 0x7fff) >= 0x3fff + 63;
        return ivp_x87_native_cosine(value);
    }
#endif
    extFloat80_t result = ivp_x87_input(value);
    const int range = fcos(result, *static_cast<softfloat_status_t *>(get_status()));
    if(out_of_range) *out_of_range = range != 0;
    return ivp_x87_output(result);
}

IVP_U_Float80 IVP_X87_Arithmetic::arctangent(const IVP_U_Float80 &y, const IVP_U_Float80 &x)
{
#if IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
    if(native_path) return ivp_x87_native_arctangent(y, x);
#endif
    return ivp_x87_output(fpatan(ivp_x87_input(x), ivp_x87_input(y),
        *static_cast<softfloat_status_t *>(get_status())));
}

// The runtime classifies the binary64 copy of its argument and evaluates the
// extended argument itself. +-1 and the default NaN are its stored constants.
enum IVP_X87_Domain { IVP_X87_INSIDE, IVP_X87_PLUS_ONE, IVP_X87_MINUS_ONE, IVP_X87_NAN, IVP_X87_OUTSIDE };

static IVP_X87_Domain ivp_x87_unit_domain(IVP_X87_Arithmetic &fp, const IVP_U_Float80 &value)
{
    const double rounded = fp.store_double(value);
    uint64_t bits;
    memcpy(&bits, &rounded, sizeof(bits));
    const unsigned exponent = static_cast<unsigned>(bits >> 52) & 0x7ff;
    const uint64_t fraction = bits & UINT64_C(0xfffffffffffff);
    if(exponent < 0x3ff) return IVP_X87_INSIDE;
    if(exponent == 0x3ff && fraction == 0) return (bits >> 63) ? IVP_X87_MINUS_ONE : IVP_X87_PLUS_ONE;
    if(exponent == 0x7ff && fraction != 0) return IVP_X87_NAN;
    return IVP_X87_OUTSIDE;
}

static IVP_U_Float80 ivp_x87_unit_root(IVP_X87_Arithmetic &fp, const IVP_U_Float80 &value)
{
    const IVP_U_Float80 one = fp.load(1.0);
    return fp.square_root(fp.multiply(fp.add(one, value), fp.subtract(one, value)));
}

static IVP_U_Float80 ivp_x87_quiet(const IVP_U_Float80 &value)
{
    return IVP_U_Float80::from_bits(value.get_sign_exponent(), value.get_significand() | (UINT64_C(1) << 62));
}

static const uint64_t ivp_x87_pi_significand = UINT64_C(0xc90fdaa22168c235);

// The runtime evaluates with the caller's precision and nearest rounding. Unless
// the caller's control word is the default (53-bit, nearest), its exit hands an
// inexact result to the exception path, which stores and reloads it as
// binary64; the precision flag is sticky, so a running program always takes it.
static IVP_X87_Arithmetic::Precision ivp_x87_precision(const softfloat_status_t *status)
{
    return status->extF80_roundingPrecision == 32 ? IVP_X87_Arithmetic::PRECISION_24
        : status->extF80_roundingPrecision == 64 ? IVP_X87_Arithmetic::PRECISION_53 : IVP_X87_Arithmetic::PRECISION_64;
}

static IVP_U_Float80 ivp_x87_runtime_result(const softfloat_status_t *caller, IVP_X87_Arithmetic &runtime,
    const IVP_U_Float80 &result)
{
    if(caller->extF80_roundingPrecision == 64 && caller->softfloat_roundingMode == softfloat_round_near_even)
        return result;
    return runtime.load(runtime.store_double(result));
}

IVP_U_Float80 IVP_X87_Arithmetic::arcsine(const IVP_U_Float80 &value)
{
    const softfloat_status_t *status = static_cast<const softfloat_status_t *>(get_status());
    IVP_X87_Arithmetic runtime(ivp_x87_precision(status), ROUND_NEAREST);
    IVP_U_Float80 result;
    switch(ivp_x87_unit_domain(runtime, value))
    {
    case IVP_X87_INSIDE: result = runtime.arctangent(value, ivp_x87_unit_root(runtime, value)); break;
    case IVP_X87_PLUS_ONE: result = IVP_U_Float80::from_bits(0x3fff, ivp_x87_pi_significand); break;
    case IVP_X87_MINUS_ONE: result = IVP_U_Float80::from_bits(0xbfff, ivp_x87_pi_significand); break;
    case IVP_X87_NAN: return ivp_x87_quiet(value);
    default: return IVP_U_Float80::from_bits(0xffff, UINT64_C(0xc000000000000000));
    }
    return ivp_x87_runtime_result(status, runtime, result);
}

IVP_U_Float80 IVP_X87_Arithmetic::arccosine(const IVP_U_Float80 &value)
{
    const softfloat_status_t *status = static_cast<const softfloat_status_t *>(get_status());
    IVP_X87_Arithmetic runtime(ivp_x87_precision(status), ROUND_NEAREST);
    IVP_U_Float80 result;
    switch(ivp_x87_unit_domain(runtime, value))
    {
    case IVP_X87_INSIDE: result = runtime.arctangent(ivp_x87_unit_root(runtime, value), value); break;
    case IVP_X87_PLUS_ONE: return IVP_U_Float80::from_bits(0, 0);
    case IVP_X87_MINUS_ONE: result = IVP_U_Float80::from_bits(0x4000, ivp_x87_pi_significand); break;
    case IVP_X87_NAN: return ivp_x87_quiet(value);
    default: return IVP_U_Float80::from_bits(0xffff, UINT64_C(0xc000000000000000));
    }
    return ivp_x87_runtime_result(status, runtime, result);
}

IVP_U_Float80 IVP_X87_Arithmetic::power2_minus1(const IVP_U_Float80 &value)
{
#if IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
    if(native_path) return ivp_x87_native_power2_minus1(value);
#endif
    return ivp_x87_output(f2xm1(ivp_x87_input(value), *static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::log2_multiply(const IVP_U_Float80 &x, const IVP_U_Float80 &y)
{
#if IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
    if(native_path) return ivp_x87_native_log2_multiply(x, y);
#endif
    return ivp_x87_output(fyl2x(ivp_x87_input(x), ivp_x87_input(y),
        *static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::round_integer(const IVP_U_Float80 &value)
{
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
    if(native_path) return ivp_x87_native_round_integer(value);
#endif
    return ivp_x87_output(extF80_roundToInt(ivp_x87_input(value), static_cast<softfloat_status_t *>(get_status())));
}

IVP_U_Float80 IVP_X87_Arithmetic::scale(const IVP_U_Float80 &value, const IVP_U_Float80 &exponent)
{
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
    if(native_path) return ivp_x87_native_scale(value, exponent);
#endif
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
