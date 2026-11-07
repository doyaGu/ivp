#include <ivu_fpu.hxx>

#if !IVP_ENABLE_X87_EMULATION
void ivp_set_fpu_control_word_callbacks(IVP_Control_Word_Reader reader, IVP_Control_Word_Writer writer)
{
    (void)reader;
    (void)writer;
}
#endif
