#ifndef IVP_NUMERICS_X87_PRECISION_CASES_INCLUDED
#define IVP_NUMERICS_X87_PRECISION_CASES_INCLUDED

#include <ivu_x87.hxx>

static bool ivp_float80_equal(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
{
    return first.get_sign_exponent() == second.get_sign_exponent()
        && first.get_significand() == second.get_significand();
}

static int ivp_check_x87_precision()
{
    IVP_X87_Arithmetic fp;
    const IVP_U_Float80 one = fp.load(1);
    const IVP_U_Float80 two = fp.load(2);
    const IVP_U_Float80 zero = fp.load(0);
    int failures = 0;
    const IVP_U_Float80 increment = IVP_U_Float80::from_bits(0x3fc0, UINT64_C(0x8000000000000000));
    const IVP_U_Float80 live_sum = fp.add(one, increment);
    failures += live_sum.get_significand() != UINT64_C(0x8000000000000001);
    failures += fp.store_double(live_sum) != 1.0;
    failures += !ivp_float80_equal(fp.subtract(live_sum, one), increment);
    unsigned char bytes[10];
    live_sum.to_bytes(bytes);
    failures += bytes[0] != 1 || bytes[7] != 0x80 || bytes[8] != 0xff || bytes[9] != 0x3f;
    failures += !ivp_float80_equal(IVP_U_Float80::from_bytes(bytes), live_sum);

    IVP_X87_Arithmetic single(IVP_X87_Arithmetic::PRECISION_24, IVP_X87_Arithmetic::ROUND_UP);
    const IVP_U_Float80 rounded = single.add(one, increment);
    failures += rounded.get_significand() != UINT64_C(0x8000010000000000);
    failures += !ivp_float80_equal(fp.add(one, increment), live_sum);
#if IVP_X87_ARITHMETIC != IVP_X87_HARD // host x87 operations raise the host's exceptions
    fp.divide(one, zero);
    failures += (fp.get_exception_flags() & IVP_X87_Arithmetic::EXCEPTION_DIVISION_BY_ZERO) == 0;
    failures += (single.get_exception_flags() & IVP_X87_Arithmetic::EXCEPTION_DIVISION_BY_ZERO) != 0;
    fp.clear_exception_flags();
    failures += fp.get_exception_flags() != 0;
#endif

    bool outside = true;
    failures += !ivp_float80_equal(fp.sine(zero, &outside), zero) || outside;
    failures += !ivp_float80_equal(fp.cosine(zero, &outside), one) || outside;
    failures += !ivp_float80_equal(fp.square_root(fp.load(4)), two);
    failures += !ivp_float80_equal(fp.power2_minus1(zero), zero);
    failures += !ivp_float80_equal(fp.log2_multiply(two, two), two);
    failures += !ivp_float80_equal(fp.scale(one, fp.load(3)), fp.load(8));
    failures += !ivp_float80_equal(fp.round_integer(fp.load(1.5)), two);
    failures += !ivp_float80_equal(fp.round_integer(fp.load(-1.5)), fp.load(-2));
    failures += fp.store_double(fp.arctangent(one, zero)) != 1.5707963267948966;
    failures += !ivp_float80_equal(fp.arctangent(zero, one), zero);
    failures += fp.compare(one, two) != IVP_X87_Arithmetic::LESS;
    failures += fp.compare(two, one) != IVP_X87_Arithmetic::GREATER;
    failures += fp.compare(one, one) != IVP_X87_Arithmetic::EQUAL;

#if IVP_X87_ARITHMETIC_TRANSCENDENTAL != IVP_X87_HARD // host fpatan's last bit depends on the processor vendor
    // asin/acos as msvcrt.dll's _CIasin/_CIacos return them: rounded to
    // binary64 under 64-bit precision, the register value under 53-bit.
    struct UnitCase { double x; uint16_t asin_se[2]; uint64_t asin_sig[2]; uint16_t acos_se[2]; uint64_t acos_sig[2]; };
    static const UnitCase unit_cases[] = {
        {0.5, {0x3ffe, 0x3ffe}, {UINT64_C(0x860a91c16b9b3000), UINT64_C(0x860a91c16b9b2df2)},
            {0x3fff, 0x3fff}, {UINT64_C(0x860a91c16b9b3000), UINT64_C(0x860a91c16b9b2b3c)}},
        {-0.25, {0xbffd, 0xbffd}, {UINT64_C(0x815f4e705c3e6800), UINT64_C(0x815f4e705c3e67b9)},
            {0x3fff, 0x3fff}, {UINT64_C(0xe967ae3e38786000), UINT64_C(0xe967ae3e38785c23)}},
        {0.75, {0x3ffe, 0x3ffe}, {UINT64_C(0xd91a98ae3406e000), UINT64_C(0xd91a98ae3406de8e)},
            {0x3ffe, 0x3ffe}, {UINT64_C(0xb9051c960ecaa800), UINT64_C(0xb9051c960ecaa5db)}},
        {1.0, {0x3fff, 0x3fff}, {UINT64_C(0xc90fdaa22168c000), UINT64_C(0xc90fdaa22168c235)},
            {0, 0}, {0, 0}},
        {-1.0, {0xbfff, 0xbfff}, {UINT64_C(0xc90fdaa22168c000), UINT64_C(0xc90fdaa22168c235)},
            {0x4000, 0x4000}, {UINT64_C(0xc90fdaa22168c000), UINT64_C(0xc90fdaa22168c235)}},
    };
    IVP_X87_Arithmetic extended(IVP_X87_Arithmetic::PRECISION_64, IVP_X87_Arithmetic::ROUND_NEAREST);
    IVP_X87_Arithmetic standard(IVP_X87_Arithmetic::PRECISION_53, IVP_X87_Arithmetic::ROUND_NEAREST);
    for(unsigned i = 0; i < sizeof(unit_cases) / sizeof(unit_cases[0]); ++i)
    {
        const UnitCase &c = unit_cases[i];
        IVP_X87_Arithmetic *selection[2] = {&extended, &standard};
        for(int p = 0; p < 2; ++p)
        {
            const IVP_U_Float80 x = selection[p]->load(c.x);
            failures += !ivp_float80_equal(selection[p]->arcsine(x), IVP_U_Float80::from_bits(c.asin_se[p], c.asin_sig[p]));
            failures += !ivp_float80_equal(selection[p]->arccosine(x), IVP_U_Float80::from_bits(c.acos_se[p], c.acos_sig[p]));
        }
    }
    failures += !ivp_float80_equal(extended.arcsine(two), IVP_U_Float80::from_bits(0xffff, UINT64_C(0xc000000000000000)));
#endif

    const IVP_X87_Control control =
        {IVP_X87_Arithmetic::PRECISION_24, IVP_X87_Arithmetic::ROUND_UP};
    IVP_X87_Control::set(control);
    IVP_X87_Arithmetic selected(IVP_X87_Control::current());
    failures += selected.add(one, increment).get_significand() != UINT64_C(0x8000010000000000);
    IVP_X87_Control::reset();
    return failures;
}

#endif
