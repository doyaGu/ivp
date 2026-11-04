#include "x87_precision_cases.hxx"
#include <cstdio>

int main()
{
    const int failures = ivp_check_x87_precision();
    std::printf("extended-value/control/status failures: %d\n", failures);
    return failures ? 1 : 0;
}
