#ifndef IVP_U_X87_INCLUDED
#define IVP_U_X87_INCLUDED

#include <stdint.h>
#include <ivu_x87_config.hxx>

class IVP_X87_Arithmetic;
struct IVP_X87_Control;

// An extended value has no dependency on the host floating-point ABI.
class IVP_U_Float80
{
    uint64_t significand;
    uint16_t sign_exponent;

public:
    IVP_U_Float80() : significand(0), sign_exponent(0) {}
    static IVP_U_Float80 from_bits(uint16_t exponent, uint64_t fraction)
    {
        IVP_U_Float80 result;
        result.sign_exponent = exponent;
        result.significand = fraction;
        return result;
    }
    static IVP_U_Float80 from_bytes(const unsigned char *bytes);
    void to_bytes(unsigned char *bytes) const;
    uint64_t get_significand() const { return significand; }
    uint16_t get_sign_exponent() const { return sign_exponent; }
};

#include <ivu_x87_fast.hxx>
#if (IVP_X87_ARITHMETIC == IVP_X87_HARD || IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD)
#include <ivu_x87_native.hxx>
#endif

// Each numerical kernel has explicit precision, rounding and exception status.
// Its storage is fixed; constructing or operating on it never allocates.
class IVP_X87_Arithmetic
{
    union StatusStorage
    {
        uint64_t alignment;
        unsigned char bytes[16];
    } status_storage;
    // 64-bit precision with nearest-even rounding: normal cases of the
    // inline operations use ivu_x87_fast.hxx. Their inexact results are
    // merged into the exception flags.
    bool fast_path;
    bool fast_inexact;
    // HARD selections: fast_path while the executing FPU also uses 64-bit
    // precision and nearest rounding. Those operations then run on the host
    // x87 and raise its exceptions, not the object's.
    bool native_path;

    IVP_X87_Arithmetic(const IVP_X87_Arithmetic &);
    IVP_X87_Arithmetic &operator=(const IVP_X87_Arithmetic &);
    void *get_status() { return status_storage.bytes; }
    const void *get_status() const { return status_storage.bytes; }
    void update_fast_path();

    IVP_U_Float80 load_general(float value);
    IVP_U_Float80 load_general(double value);
    float store_float_general(const IVP_U_Float80 &value);
    double store_double_general(const IVP_U_Float80 &value);
    IVP_U_Float80 add_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 subtract_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 multiply_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 divide_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);

public:
    enum Precision { PRECISION_24 = 24, PRECISION_53 = 53, PRECISION_64 = 64 };
    enum Rounding { ROUND_NEAREST = 0, ROUND_DOWN = 1, ROUND_UP = 2, ROUND_TO_ZERO = 3 };
    enum Relation { LESS = -1, EQUAL = 0, GREATER = 1, UNORDERED = 2 };
    enum Exception
    {
        EXCEPTION_INVALID_OPERATION = 0x01,
        EXCEPTION_DENORMAL_INPUT = 0x02,
        EXCEPTION_DIVISION_BY_ZERO = 0x04,
        EXCEPTION_OVERFLOW = 0x08,
        EXCEPTION_UNDERFLOW = 0x10,
        EXCEPTION_INEXACT_RESULT = 0x20
    };

    explicit IVP_X87_Arithmetic(Precision precision = PRECISION_64,
        Rounding rounding = ROUND_NEAREST);
    explicit IVP_X87_Arithmetic(const IVP_X87_Control &control);
    void set_precision(Precision precision);
    void set_rounding(Rounding rounding);
    unsigned get_exception_flags() const;
    void clear_exception_flags();

    IVP_U_Float80 load(float value)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_load_float(value);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_load_float(value, &result)) return result;
        return load_general(value);
    }
    IVP_U_Float80 load(double value)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_load_double(value);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_load_double(value, &result)) return result;
        return load_general(value);
    }
    IVP_U_Float80 load(int32_t value);
    float store_float(const IVP_U_Float80 &value)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_store_float(value);
#endif
        float result;
        if(fast_path && ivp_x87_fast_store_float(value, &result, &fast_inexact)) return result;
        return store_float_general(value);
    }
    double store_double(const IVP_U_Float80 &value)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_store_double(value);
#endif
        double result;
        if(fast_path && ivp_x87_fast_store_double(value, &result, &fast_inexact)) return result;
        return store_double_general(value);
    }
    // Scalar API returns use the host ABI. On a native x87 ABI this retains
    // the extended result; internal portable call chains use IVP_U_Float80.
    float return_float(const IVP_U_Float80 &value);
    double return_double(const IVP_U_Float80 &value);

    IVP_U_Float80 add(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_add(first, second);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_add(first, second, 0, &result, &fast_inexact)) return result;
        return add_general(first, second);
    }
    IVP_U_Float80 subtract(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_subtract(first, second);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_add(first, second, 1, &result, &fast_inexact)) return result;
        return subtract_general(first, second);
    }
    IVP_U_Float80 multiply(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_multiply(first, second);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_multiply(first, second, &result, &fast_inexact)) return result;
        return multiply_general(first, second);
    }
    IVP_U_Float80 divide(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_divide(first, second);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_divide(first, second, &result, &fast_inexact)) return result;
        return divide_general(first, second);
    }
    IVP_U_Float80 square_root(const IVP_U_Float80 &value);
    IVP_U_Float80 absolute(const IVP_U_Float80 &value);
    IVP_U_Float80 negate(const IVP_U_Float80 &value);
    Relation compare(const IVP_U_Float80 &first, const IVP_U_Float80 &second,
        bool quiet = false);

    IVP_U_Float80 sine(const IVP_U_Float80 &value, bool *out_of_range = 0);
    IVP_U_Float80 cosine(const IVP_U_Float80 &value, bool *out_of_range = 0);
    IVP_U_Float80 arctangent(const IVP_U_Float80 &y, const IVP_U_Float80 &x);
    // The C runtime's x87 asin and acos: for |x| < 1, an arctangent of x and
    // the square root of (1 + x)(1 - x) at the current precision with nearest
    // rounding; stored constants for x = +-1; the default NaN outside the
    // domain. Unless the control is 53-bit nearest, the runtime returns the
    // result rounded to binary64.
    IVP_U_Float80 arcsine(const IVP_U_Float80 &value);
    IVP_U_Float80 arccosine(const IVP_U_Float80 &value);
    IVP_U_Float80 power2_minus1(const IVP_U_Float80 &value);
    IVP_U_Float80 log2_multiply(const IVP_U_Float80 &x, const IVP_U_Float80 &y);
    IVP_U_Float80 round_integer(const IVP_U_Float80 &value);
    IVP_U_Float80 scale(const IVP_U_Float80 &value, const IVP_U_Float80 &exponent);
    IVP_U_Float80 natural_logarithm_of_two() const;
    IVP_U_Float80 logarithm2_of_e() const;
    IVP_U_Float80 exponential(const IVP_U_Float80 &value);
};

// Precision and rounding belong to the executing thread, as in an x87 control
// word. An explicit setting also works on processors without a native x87 FPU.
struct IVP_X87_Control
{
    IVP_X87_Arithmetic::Precision precision;
    IVP_X87_Arithmetic::Rounding rounding;

    static IVP_X87_Control current();
    static void set(const IVP_X87_Control &control);
    static void reset();
};

#endif
