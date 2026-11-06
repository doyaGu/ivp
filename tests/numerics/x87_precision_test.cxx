#include "x87_precision_cases.hxx"
#include <cstdio>

static unsigned short host_control_word = 0x037f;
static unsigned short read_host_control_word() { return host_control_word; }
static void write_host_control_word(unsigned short word) { host_control_word = word; }

static int check_host_control_word()
{
    ivp_set_x87_control_word_callbacks(read_host_control_word, write_host_control_word);
    IVP_X87_Control initial = IVP_X87_Control::current();
    int failures = initial.precision != IVP_X87_Arithmetic::PRECISION_64;
    IVP_X87_Control selected = {IVP_X87_Arithmetic::PRECISION_24, IVP_X87_Arithmetic::ROUND_UP};
    IVP_X87_Control::set(selected);
    failures += host_control_word != 0x087f;
    const IVP_X87_Control current = IVP_X87_Control::current();
    failures += current.precision != selected.precision || current.rounding != selected.rounding;
    IVP_X87_Control::reset();
    failures += host_control_word != 0x037f;
    ivp_set_x87_control_word_callbacks(0, 0);
    return failures;
}

int main()
{
    const int failures = check_host_control_word() + ivp_check_x87_precision();
    std::printf("extended-value/control/status failures: %d\n", failures);
    return failures ? 1 : 0;
}
