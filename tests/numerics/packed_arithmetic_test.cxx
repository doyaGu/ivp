#include <stdint.h>
#include <stdio.h>

#pragma pack(push, 1)
#include <ivu_x87.hxx>
struct IVP_Packed_Following
{
    char prefix;
    uint32_t value;
};
#pragma pack(pop)

void ivp_numerics_layout(unsigned *layout);

int main()
{
    unsigned normal[8];
    ivp_numerics_layout(normal);
    const unsigned packed[] = {
        sizeof(IVP_U_Float80), __alignof(IVP_U_Float80),
        sizeof(IVP_X87_Arithmetic), __alignof(IVP_X87_Arithmetic),
        sizeof(IVP_X87_Control), __alignof(IVP_X87_Control),
        sizeof(IVP_FPU_Precision_Scope), __alignof(IVP_FPU_Precision_Scope)};
    unsigned failures = sizeof(IVP_Packed_Following) != 5;
    for (unsigned i = 0; i < 8; ++i)
    {
        if (packed[i] != normal[i])
        {
            printf("layout field %u: packed=%u library=%u\n", i, packed[i], normal[i]);
            ++failures;
        }
    }
    // Do not call a library with incompatible object layouts.
    if (failures)
        return 1;
    IVP_X87_Arithmetic arithmetic;
    const IVP_U_Float80 first = arithmetic.load(16777217);
    const IVP_U_Float80 second = arithmetic.load(0.25);
    const IVP_U_Float80 sum = arithmetic.add(first, second);
    failures += arithmetic.store_double(sum) != 16777217.25;
    printf("packed arithmetic failures: %u\n", failures);
    return failures ? 1 : 0;
}
