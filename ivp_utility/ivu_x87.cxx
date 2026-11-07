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
static thread_local unsigned short ivp_x87_saved_host_control = 0;
static IVP_Control_Word_Reader ivp_x87_read_host_control = 0;
static IVP_Control_Word_Writer ivp_x87_write_host_control = 0;

void ivp_set_fpu_control_word_callbacks(IVP_Control_Word_Reader reader,
                                      IVP_Control_Word_Writer writer)
{
    const IVP_Control_Word_Reader selected_reader = reader && writer ? reader : 0;
    const IVP_Control_Word_Writer selected_writer = reader && writer ? writer : 0;
    if (selected_reader == ivp_x87_read_host_control && selected_writer == ivp_x87_write_host_control)
        return;
    ivp_x87_control_is_explicit = false;
    ivp_x87_read_host_control = selected_reader;
    ivp_x87_write_host_control = selected_writer;
}

static thread_local IVP_X87_Control ivp_x87_control =
    {IVP_X87_Arithmetic::PRECISION_64, IVP_X87_Arithmetic::ROUND_NEAREST};
#if defined(_MSC_VER) && defined(_M_IX86)
static thread_local unsigned int ivp_x87_saved_native_control = 0;
#endif

IVP_X87_Control IVP_X87_Control::current()
{
    if (ivp_x87_read_host_control)
    {
        const unsigned short word = ivp_x87_read_host_control();
        const unsigned precision = word & 0x0300;
        IVP_X87_Control result = {precision == 0 ? IVP_X87_Arithmetic::PRECISION_24 :
            precision == 0x0200 ? IVP_X87_Arithmetic::PRECISION_53 : IVP_X87_Arithmetic::PRECISION_64,
            static_cast<IVP_X87_Arithmetic::Rounding>((word >> 10) & 3)};
        return result;
    }
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
    if (ivp_x87_write_host_control)
    {
        const unsigned short previous = ivp_x87_read_host_control();
        if (!ivp_x87_control_is_explicit)
            ivp_x87_saved_host_control = previous;
        const unsigned host_precision = control.precision == IVP_X87_Arithmetic::PRECISION_24 ? 0 :
            control.precision == IVP_X87_Arithmetic::PRECISION_53 ? 0x0200 : 0x0300;
        ivp_x87_write_host_control(static_cast<unsigned short>(
            (previous & ~0x0f00) | host_precision | (control.rounding << 10)));
        ivp_x87_control_is_explicit = true;
        ivp_x87_control = control;
        return;
    }
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
    if (ivp_x87_write_host_control)
    {
        if (ivp_x87_control_is_explicit)
            ivp_x87_write_host_control(ivp_x87_saved_host_control);
        ivp_x87_control_is_explicit = false;
        return;
    }
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
