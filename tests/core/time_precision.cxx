#include <ivp_physics.hxx>
#include <ivp_environment.hxx>
#include <ivp_time.hxx>
#include <ivu_fpu.hxx>
#if IVP_ENABLE_X87_EMULATION
#include <ivu_x87.hxx>
#endif

#include <stdio.h>
#if defined(_MSC_VER)
#include <stdint.h>
#include <stdlib.h>
#endif

static unsigned short host_control_word = 0x087f;
static unsigned callbacks = 0;
static unsigned invalid_parameters = 0;

static unsigned short read_control_word()
{
    ++callbacks;
    return host_control_word;
}

static void write_control_word(unsigned short word)
{
    ++callbacks;
    host_control_word = word;
}

#if defined(_MSC_VER)
static void invalid_parameter(const wchar_t *, const wchar_t *, const wchar_t *, unsigned, uintptr_t)
{
    ++invalid_parameters;
}
#endif

class Precision_Event_Manager : public IVP_Event_Manager
{
public:
    unsigned calls;
    unsigned failures;
    bool throw_on_next;

    Precision_Event_Manager() : calls(0), failures(0), throw_on_next(false) { mode = 0; }

    void check_precision()
    {
        ++calls;
#if IVP_ENABLE_X87_EMULATION
        const IVP_X87_Control control = IVP_X87_Control::current();
        failures += control.precision != IVP_X87_Arithmetic::PRECISION_64;
        failures += control.rounding != IVP_X87_Arithmetic::ROUND_UP;
#endif
        if (throw_on_next)
            throw 1;
    }

    virtual void simulate_time_events(IVP_Time_Manager *, IVP_Environment *, IVP_Time)
    {
        check_precision();
    }

    virtual void simulate_variable_time_step(IVP_Time_Manager *, IVP_Environment *, IVP_Time_Event_PSI *, IVP_FLOAT)
    {
        check_precision();
    }
};

int main()
{
#if defined(_MSC_VER)
    const _invalid_parameter_handler previous_handler =
        _set_thread_local_invalid_parameter_handler(invalid_parameter);
#endif
    IVP_Application_Environment application;
    IVP_Environment *environment = IVP_Environment_Manager::get_environment_manager()->create_environment(
        &application, "time-precision-test", 0);
    IVP_Time_Manager *time = environment->get_time_manager();
    delete time->event_manager;
    Precision_Event_Manager *events = new Precision_Event_Manager;
    time->event_manager = events;

    ivp_set_fpu_control_word_callbacks(read_control_word, write_control_word);
    time->event_loop(environment, IVP_Time(0));
    time->simulate_variable_time_step(environment, 0.015f);
    unsigned failures = events->failures + (events->calls != 2);
    failures += host_control_word != 0x087f;
    events->throw_on_next = true;
    bool caught = false;
    try
    {
        time->event_loop(environment, IVP_Time(0));
    }
    catch (int)
    {
        caught = true;
    }
    failures += !caught || host_control_word != 0x087f;
#if !IVP_ENABLE_X87_EMULATION
    failures += callbacks != 0;
#endif
    ivp_set_fpu_control_word_callbacks(0, 0);
    delete environment;
    failures += invalid_parameters;
#if defined(_MSC_VER)
    _set_thread_local_invalid_parameter_handler(previous_handler);
#endif
    printf("time precision failures: %u (invalid parameters: %u)\n", failures, invalid_parameters);
    return failures ? 1 : 0;
}
