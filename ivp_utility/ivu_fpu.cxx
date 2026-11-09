#include <ivu_fpu.hxx>
#include <ivu_x87_config.hxx>

#include <assert.h>

#if (IVP_ENABLE_X87_EMULATION || IVP_X87_ARITHMETIC_TRANSCENDENTAL != IVP_X87_OFF)
#include <ivu_x87.hxx>

static IVP_Control_Word_Reader ivp_read_host_control = 0;
static IVP_Control_Word_Writer ivp_write_host_control = 0;
#if !defined(__i386__) && !(defined(_MSC_VER) && defined(_M_IX86))
static thread_local unsigned short ivp_control_word = 0x037f;
#endif
#endif

void ivp_set_fpu_control_word_callbacks(IVP_Control_Word_Reader reader, IVP_Control_Word_Writer writer)
{
#if (IVP_ENABLE_X87_EMULATION || IVP_X87_ARITHMETIC_TRANSCENDENTAL != IVP_X87_OFF)
    const IVP_Control_Word_Reader selected_reader = reader && writer ? reader : 0;
    const IVP_Control_Word_Writer selected_writer = reader && writer ? writer : 0;
    if (selected_reader == ivp_read_host_control && selected_writer == ivp_write_host_control)
        return;
    IVP_X87_Control::reset();
    ivp_read_host_control = selected_reader;
    ivp_write_host_control = selected_writer;
#else
    (void)reader;
    (void)writer;
#endif
}

unsigned short ivp_get_fpu_control_word()
{
#if (IVP_ENABLE_X87_EMULATION || IVP_X87_ARITHMETIC_TRANSCENDENTAL != IVP_X87_OFF)
    if (ivp_read_host_control)
        return ivp_read_host_control();
#endif
#if defined(__i386__)
    unsigned short control_word;
    __asm__ __volatile__("fnstcw %0" : "=m"(control_word));
    return control_word;
#elif defined(_MSC_VER) && defined(_M_IX86)
    unsigned short control_word;
    __asm fnstcw control_word
    return control_word;
#elif (IVP_ENABLE_X87_EMULATION || IVP_X87_ARITHMETIC_TRANSCENDENTAL != IVP_X87_OFF)
    return ivp_control_word;
#else
    return 0x037f;
#endif
}

void ivp_set_fpu_control_word(unsigned short control_word)
{
#if (IVP_ENABLE_X87_EMULATION || IVP_X87_ARITHMETIC_TRANSCENDENTAL != IVP_X87_OFF)
    if (ivp_write_host_control)
    {
        ivp_write_host_control(control_word);
        return;
    }
#endif
#if defined(__i386__)
    __asm__ __volatile__("fldcw %0" : : "m"(control_word));
#elif defined(_MSC_VER) && defined(_M_IX86)
    __asm fldcw control_word
#elif (IVP_ENABLE_X87_EMULATION || IVP_X87_ARITHMETIC_TRANSCENDENTAL != IVP_X87_OFF)
    ivp_control_word = control_word;
#else
    (void)control_word;
#endif
}

IVP_FPU_Precision_Scope::IVP_FPU_Precision_Scope(unsigned precision)
{
    assert(precision == 24 || precision == 53 || precision == 64);
    const unsigned short previous = ivp_get_fpu_control_word();
    saved_precision = previous & 0x0300;
    const unsigned short selected = precision == 24 ? 0 : precision == 53 ? 0x0200 : 0x0300;
    if (selected != saved_precision)
        ivp_set_fpu_control_word(static_cast<unsigned short>((previous & ~0x0300) | selected));
}

IVP_FPU_Precision_Scope::~IVP_FPU_Precision_Scope()
{
    const unsigned short current = ivp_get_fpu_control_word();
    if ((current & 0x0300) != saved_precision)
        ivp_set_fpu_control_word(static_cast<unsigned short>((current & ~0x0300) | saved_precision));
}
