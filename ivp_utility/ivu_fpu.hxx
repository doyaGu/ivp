#ifndef IVP_U_FPU_INCLUDED
#define IVP_U_FPU_INCLUDED

// Bind the host's floating-point environment before starting simulation
// threads. A missing pair restores standalone handling. The selected numerical
// implementation decides whether it needs these callbacks.
typedef unsigned short (*IVP_Control_Word_Reader)();
typedef void (*IVP_Control_Word_Writer)(unsigned short word);
void ivp_set_fpu_control_word_callbacks(IVP_Control_Word_Reader reader, IVP_Control_Word_Writer writer);

#endif
