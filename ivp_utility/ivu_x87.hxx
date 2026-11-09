#ifndef IVP_U_X87_INCLUDED
#define IVP_U_X87_INCLUDED

#include <ivu_fpu.hxx>
#include <ivu_x87_config.hxx>
#include <libx87/x87.hxx>

#pragma pack(push, 8)

typedef X87_Float80 IVP_U_Float80;
struct IVP_X87_Control;

class IVP_X87_Arithmetic : public X87_Arithmetic
{
public:
    explicit IVP_X87_Arithmetic(Precision precision = PRECISION_64,
        Rounding rounding = ROUND_NEAREST)
        : X87_Arithmetic(precision, rounding,
            IVP_X87_ARITHMETIC == IVP_X87_HARD,
            IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD) {}
    explicit IVP_X87_Arithmetic(const IVP_X87_Control &control);
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

#pragma pack(pop)

#endif
