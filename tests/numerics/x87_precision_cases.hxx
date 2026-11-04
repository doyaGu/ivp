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
    fp.divide(one, zero);
    failures += (fp.get_exception_flags() & IVP_X87_Arithmetic::EXCEPTION_DIVISION_BY_ZERO) == 0;
    failures += (single.get_exception_flags() & IVP_X87_Arithmetic::EXCEPTION_DIVISION_BY_ZERO) != 0;
    fp.clear_exception_flags();
    failures += fp.get_exception_flags() != 0;

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
    const IVP_X87_Control control =
        {IVP_X87_Arithmetic::PRECISION_24, IVP_X87_Arithmetic::ROUND_UP};
    IVP_X87_Control::set(control);
    IVP_X87_Arithmetic selected(IVP_X87_Control::current());
    failures += selected.add(one, increment).get_significand() != UINT64_C(0x8000010000000000);
    IVP_X87_Control::reset();
    return failures;
}

#endif
