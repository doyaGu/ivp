#include <ivu_x87.hxx>

void ivp_numerics_layout(unsigned *layout)
{
    layout[0] = sizeof(IVP_U_Float80);
    layout[1] = __alignof(IVP_U_Float80);
    layout[2] = sizeof(IVP_X87_Arithmetic);
    layout[3] = __alignof(IVP_X87_Arithmetic);
    layout[4] = sizeof(IVP_X87_Control);
    layout[5] = __alignof(IVP_X87_Control);
    layout[6] = sizeof(IVP_FPU_Precision_Scope);
    layout[7] = __alignof(IVP_FPU_Precision_Scope);
}
