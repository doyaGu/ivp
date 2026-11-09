#include <ivu_x87.hxx>
#include <cstdio>

#if IVP_X87_ARITHMETIC == IVP_X87_HARD || IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
static unsigned short host_control()
{
    unsigned short word;
#if defined(_MSC_VER)
    __asm fnstcw word
#else
    __asm__ __volatile__("fnstcw %0" : "=m"(word));
#endif
    return word;
}

static void host_control(unsigned short word)
{
#if defined(_MSC_VER)
    __asm fldcw word
#else
    __asm__ __volatile__("fldcw %0" : : "m"(word));
#endif
}
#endif

static void clear_flags(IVP_X87_Arithmetic &fp)
{
    fp.clear_exception_flags();
#if IVP_X87_ARITHMETIC == IVP_X87_HARD || IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
#if defined(_MSC_VER)
    __asm fnclex
#else
    __asm__ __volatile__("fnclex");
#endif
#endif
}

static unsigned flags(const IVP_X87_Arithmetic &fp)
{
    unsigned result = fp.get_exception_flags();
#if IVP_X87_ARITHMETIC == IVP_X87_HARD || IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
    unsigned short status;
#if defined(_MSC_VER)
    __asm fnstsw status
#else
    __asm__ __volatile__("fnstsw %0" : "=m"(status));
#endif
    result |= status & 0x3f;
#endif
    return result;
}

static int check(bool condition, const char *message)
{
    if(condition) return 0;
    std::printf("FAIL: %s\n", message);
    return 1;
}

int main()
{
    int failures = 0;
    const IVP_U_Float80 zero = IVP_U_Float80::from_bits(0, 0);
    const IVP_U_Float80 one = IVP_U_Float80::from_bits(0x3fff, UINT64_C(0x8000000000000000));
    const IVP_U_Float80 half = IVP_U_Float80::from_bits(0x3ffe, UINT64_C(0x8000000000000000));
    const IVP_U_Float80 below_one = IVP_U_Float80::from_bits(0x3ffe, UINT64_C(0xffffffffffffffff));
    const IVP_U_Float80 two = IVP_U_Float80::from_bits(0x4000, UINT64_C(0x8000000000000000));
    const IVP_U_Float80 snan = IVP_U_Float80::from_bits(0x7fff, UINT64_C(0x8000000000000001));
    const IVP_U_Float80 qnan = IVP_U_Float80::from_bits(0x7fff, UINT64_C(0xc000000000000001));
    const IVP_X87_Arithmetic::Precision precisions[] = {
        IVP_X87_Arithmetic::PRECISION_24, IVP_X87_Arithmetic::PRECISION_53, IVP_X87_Arithmetic::PRECISION_64};
    for(unsigned p = 0; p < sizeof(precisions) / sizeof(precisions[0]); ++p)
    {
        for(int r = IVP_X87_Arithmetic::ROUND_NEAREST; r <= IVP_X87_Arithmetic::ROUND_TO_ZERO; ++r)
        {
            IVP_X87_Arithmetic fp(precisions[p], static_cast<IVP_X87_Arithmetic::Rounding>(r));
            for(int cosine = 0; cosine != 2; ++cosine)
            {
                clear_flags(fp);
                if(cosine) fp.arccosine(half); else fp.arcsine(half);
                failures += check((flags(fp) & IVP_X87_Arithmetic::EXCEPTION_INEXACT_RESULT) != 0,
                    "asin/acos must preserve inexact");

                clear_flags(fp);
                if(cosine) fp.arccosine(below_one); else fp.arcsine(below_one);
                failures += check((flags(fp) & IVP_X87_Arithmetic::EXCEPTION_INEXACT_RESULT) != 0,
                    "asin/acos endpoint shortcuts must preserve argument-conversion flags");

                clear_flags(fp);
                if(cosine) fp.arccosine(two); else fp.arcsine(two);
                failures += check((flags(fp) & IVP_X87_Arithmetic::EXCEPTION_INVALID_OPERATION) != 0,
                    "asin/acos outside the domain must raise invalid");

                clear_flags(fp);
                if(cosine) fp.arccosine(snan); else fp.arcsine(snan);
                failures += check((flags(fp) & IVP_X87_Arithmetic::EXCEPTION_INVALID_OPERATION) != 0,
                    "asin/acos of a signaling NaN must preserve invalid");

                clear_flags(fp);
                if(cosine) fp.arccosine(qnan); else fp.arcsine(qnan);
                failures += check(flags(fp) == 0, "asin/acos of a quiet NaN must not raise exceptions");

                clear_flags(fp);
                fp.compare(snan, one);
                if(cosine) fp.arccosine(one); else fp.arcsine(zero);
                failures += check((flags(fp) & IVP_X87_Arithmetic::EXCEPTION_INVALID_OPERATION) != 0,
                    "asin/acos must preserve existing exceptions");
                clear_flags(fp);
                failures += check(flags(fp) == 0, "clearing exceptions must clear merged runtime flags");
            }
        }
    }

#if IVP_X87_ARITHMETIC == IVP_X87_HARD || IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
    const unsigned short saved = host_control();
    const unsigned short nearest64 = static_cast<unsigned short>((saved & ~0x0f00) | 0x0300);
    host_control(nearest64);
    IVP_X87_Arithmetic fp;
    const IVP_U_Float80 increment = IVP_U_Float80::from_bits(0x3fc3, UINT64_C(0x8000000000000000));
    const unsigned short alternatives[] = {0x0200, 0x0000, 0x0700, 0x0b00, 0x0f00};
    for(unsigned i = 0; i < sizeof(alternatives) / sizeof(alternatives[0]); ++i)
    {
        host_control(static_cast<unsigned short>((saved & ~0x0f00) | alternatives[i]));
        clear_flags(fp);
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        const IVP_U_Float80 sum = fp.add(one, increment);
        failures += check(sum.get_sign_exponent() == 0x3fff
            && sum.get_significand() == UINT64_C(0x8000000000000008),
            "HARD must retain the object's precision after the host control changes");
#endif
#if IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD
        fp.sine(half);
        failures += check((fp.get_exception_flags() & IVP_X87_Arithmetic::EXCEPTION_INEXACT_RESULT) != 0,
            "HARD transcendentals must use software status when the host control differs");
#endif
        failures += check(host_control() == static_cast<unsigned short>((saved & ~0x0f00) | alternatives[i]),
            "falling back must preserve the host control word");
    }
    host_control(saved);
#endif
    std::printf("runtime-state failures: %d\n", failures);
    return failures ? 1 : 0;
}
