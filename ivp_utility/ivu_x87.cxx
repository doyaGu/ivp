// IVP thread control and the shared arithmetic interface.
#include <ivu_x87.hxx>

static thread_local bool ivp_x87_control_is_explicit = false;
static thread_local unsigned short ivp_x87_saved_control = 0;

IVP_X87_Control IVP_X87_Control::current()
{
    const unsigned short word = ivp_get_fpu_control_word();
    const unsigned precision = word & 0x0300;
    IVP_X87_Control result = {precision == 0 ? IVP_X87_Arithmetic::PRECISION_24 :
        precision == 0x0200 ? IVP_X87_Arithmetic::PRECISION_53 : IVP_X87_Arithmetic::PRECISION_64,
        static_cast<IVP_X87_Arithmetic::Rounding>((word >> 10) & 3)};
    return result;
}

void IVP_X87_Control::set(const IVP_X87_Control &control)
{
    const unsigned short previous = ivp_get_fpu_control_word();
    if (!ivp_x87_control_is_explicit)
        ivp_x87_saved_control = previous;
    const unsigned precision = control.precision == IVP_X87_Arithmetic::PRECISION_24 ? 0 :
        control.precision == IVP_X87_Arithmetic::PRECISION_53 ? 0x0200 : 0x0300;
    ivp_set_fpu_control_word(static_cast<unsigned short>(
        (previous & ~0x0f00) | precision | (control.rounding << 10)));
    ivp_x87_control_is_explicit = true;
}

void IVP_X87_Control::reset()
{
    if (ivp_x87_control_is_explicit)
    {
        const unsigned short current = ivp_get_fpu_control_word();
        ivp_set_fpu_control_word(static_cast<unsigned short>(
            (current & ~0x0f00) | (ivp_x87_saved_control & 0x0f00)));
    }
    ivp_x87_control_is_explicit = false;
}

IVP_X87_Arithmetic::IVP_X87_Arithmetic(const IVP_X87_Control &control)
    : X87_Arithmetic(control.precision, control.rounding,
        IVP_X87_ARITHMETIC == IVP_X87_HARD,
        IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD)
{
}
