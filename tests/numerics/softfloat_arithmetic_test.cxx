#include "softfloat_arithmetic_cases.hxx"
#include <cstdio>

int main()
{
    const int failures = ivp_check_softfloat_arithmetic();
    std::printf("software arithmetic/transcendental/precision failures: %d\n", failures);
    return failures ? 1 : 0;
}
