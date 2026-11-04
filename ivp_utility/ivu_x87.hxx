#ifndef IVP_U_X87_INCLUDED
#define IVP_U_X87_INCLUDED

#include <stdint.h>

class IVP_X87_Arithmetic;
struct IVP_X87_Control;

// An extended value has no dependency on the host floating-point ABI.
class IVP_U_Float80
{
    uint64_t significand;
    uint16_t sign_exponent;

public:
    IVP_U_Float80();
    static IVP_U_Float80 from_bits(uint16_t sign_exponent, uint64_t significand);
    static IVP_U_Float80 from_bytes(const unsigned char *bytes);
    void to_bytes(unsigned char *bytes) const;
    uint64_t get_significand() const { return significand; }
    uint16_t get_sign_exponent() const { return sign_exponent; }
};

// Each numerical kernel has explicit precision, rounding and exception status.
// Its storage is fixed; constructing or operating on it never allocates.
class IVP_X87_Arithmetic
{
    union StatusStorage
    {
        uint64_t alignment;
        unsigned char bytes[16];
    } status_storage;

    IVP_X87_Arithmetic(const IVP_X87_Arithmetic &);
    IVP_X87_Arithmetic &operator=(const IVP_X87_Arithmetic &);
    void *get_status() { return status_storage.bytes; }
    const void *get_status() const { return status_storage.bytes; }

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
    ~IVP_X87_Arithmetic();
    void set_precision(Precision precision);
    void set_rounding(Rounding rounding);
    unsigned get_exception_flags() const;
    void clear_exception_flags();

    IVP_U_Float80 load(float value);
    IVP_U_Float80 load(double value);
    IVP_U_Float80 load(int32_t value);
    float store_float(const IVP_U_Float80 &value);
    double store_double(const IVP_U_Float80 &value);

    IVP_U_Float80 add(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 subtract(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 multiply(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 divide(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 square_root(const IVP_U_Float80 &value);
    IVP_U_Float80 absolute(const IVP_U_Float80 &value);
    IVP_U_Float80 negate(const IVP_U_Float80 &value);
    Relation compare(const IVP_U_Float80 &first, const IVP_U_Float80 &second,
        bool quiet = false);

    IVP_U_Float80 sine(const IVP_U_Float80 &value, bool *out_of_range = 0);
    IVP_U_Float80 cosine(const IVP_U_Float80 &value, bool *out_of_range = 0);
    IVP_U_Float80 arctangent(const IVP_U_Float80 &y, const IVP_U_Float80 &x);
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
