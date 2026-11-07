#include "x87_precision_cases.hxx"
#include <cstdio>

static unsigned short host_control_word = 0x037f;
static unsigned short read_host_control_word() { return host_control_word; }
static void write_host_control_word(unsigned short word) { host_control_word = word; }

static int check_host_control_word()
{
    ivp_set_fpu_control_word_callbacks(read_host_control_word, write_host_control_word);
    IVP_X87_Control initial = IVP_X87_Control::current();
    int failures = initial.precision != IVP_X87_Arithmetic::PRECISION_64;
    IVP_X87_Control selected = {IVP_X87_Arithmetic::PRECISION_24, IVP_X87_Arithmetic::ROUND_UP};
    IVP_X87_Control::set(selected);
    failures += host_control_word != 0x087f;
    const IVP_X87_Control current = IVP_X87_Control::current();
    failures += current.precision != selected.precision || current.rounding != selected.rounding;
    IVP_X87_Control::reset();
    failures += host_control_word != 0x037f;
    ivp_set_fpu_control_word_callbacks(0, 0);
    return failures;
}

static int check_precision_scope()
{
    ivp_set_fpu_control_word_callbacks(read_host_control_word, write_host_control_word);
    host_control_word = 0x087f;
    int failures = 0;
    {
        IVP_FPU_Precision_Scope outer(64);
        failures += host_control_word != 0x0b7f;
        {
            IVP_FPU_Precision_Scope inner(53);
            failures += host_control_word != 0x0a7f;
            host_control_word = 0x067f;
        }
        failures += host_control_word != 0x077f;
    }
    failures += host_control_word != 0x047f;
    IVP_X87_Control::reset();
    failures += host_control_word != 0x047f;

    const IVP_X87_Control selected = {IVP_X87_Arithmetic::PRECISION_53, IVP_X87_Arithmetic::ROUND_NEAREST};
    IVP_X87_Control::set(selected);
    host_control_word |= 0x1000;
    ivp_set_fpu_control_word_callbacks(0, 0);
    failures += host_control_word != 0x147f;
    return failures;
}

int main()
{
    // The checks share a thread environment and must run in this order.
    const int host_failures = check_host_control_word();
    const int scope_failures = check_precision_scope();
    const int arithmetic_failures = ivp_check_x87_precision();
    const int failures = host_failures + scope_failures + arithmetic_failures;
    std::printf("control: %d, scopes: %d, arithmetic: %d\n", host_failures, scope_failures, arithmetic_failures);
    std::printf("extended-value/control/status failures: %d\n", failures);
    return failures ? 1 : 0;
}
