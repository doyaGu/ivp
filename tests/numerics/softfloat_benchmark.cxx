#ifdef IVP_BENCHMARK_ARITHMETIC
#include <ivu_x87.hxx>
#else
#include <ivp_softfloat.hxx>
#endif
#include <chrono>
#include <cstdio>
#include <cstdlib>


int main(int argc, char **argv)
{
    const unsigned count = argc > 1 ? static_cast<unsigned>(std::strtoul(argv[1], 0, 10)) : 1000000;
    if(!count) return 2;
#ifdef IVP_BENCHMARK_ARITHMETIC
    IVP_X87_Arithmetic fp;
    IVP_U_Float80 value = fp.load(1);
    const IVP_U_Float80 increment = fp.divide(value, fp.load(1024));
#else
    softfloat_status_t status = {};
    status.extF80_roundingPrecision = 80;
    status.softfloat_roundingMode = softfloat_round_near_even;
    status.softfloat_exceptionMasks = softfloat_all_exceptions_mask;
    extFloat80_t value = i32_to_extF80(1);
    const extFloat80_t increment = extF80_div(value, i32_to_extF80(1024), &status);
#endif
    const std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for(unsigned i = 0; i < count; ++i)
    {
#ifdef IVP_BENCHMARK_ARITHMETIC
        value = fp.add(value, increment);
        value = fp.multiply(value, increment);
        value = fp.add(value, increment);
#else
        value = extF80_add(value, increment, &status);
        value = extF80_mul(value, increment, &status);
        value = extF80_add(value, increment, &status);
#endif
    }
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
#ifdef IVP_BENCHMARK_ARITHMETIC
    const unsigned exponent = value.get_sign_exponent();
    const unsigned long long significand = value.get_significand();
#else
    const unsigned exponent = value.signExp;
    const unsigned long long significand = value.signif;
#endif
    std::printf("operations=%u seconds=%.6f mop_per_second=%.3f checksum=%04x:%016llx\n",
        count * 3, seconds, count * 3 / seconds / 1000000.0,
        exponent, significand);
    return 0;
}
