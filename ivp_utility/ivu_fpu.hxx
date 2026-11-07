#ifndef IVP_U_FPU_INCLUDED
#define IVP_U_FPU_INCLUDED

// Bind the host's floating-point environment before starting simulation
// threads. A missing pair restores standalone handling. The selected numerical
// implementation decides whether it needs these callbacks.
typedef unsigned short (*IVP_Control_Word_Reader)();
typedef void (*IVP_Control_Word_Writer)(unsigned short word);
void ivp_set_fpu_control_word_callbacks(IVP_Control_Word_Reader reader, IVP_Control_Word_Writer writer);

// Native 32-bit x86 uses its control word. Extended arithmetic on other
// processors uses thread-local control; native arithmetic needs no override.
unsigned short ivp_get_fpu_control_word();
void ivp_set_fpu_control_word(unsigned short word);

// Temporarily select 24, 53 or 64 significand bits. Rounding, exception masks
// and changes to unrelated fields survive restoration, including nested calls.
#pragma pack(push, 8)
class IVP_FPU_Precision_Scope
{
    unsigned short saved_precision;

    IVP_FPU_Precision_Scope(const IVP_FPU_Precision_Scope &);
    IVP_FPU_Precision_Scope &operator=(const IVP_FPU_Precision_Scope &);

public:
    explicit IVP_FPU_Precision_Scope(unsigned precision);
    ~IVP_FPU_Precision_Scope();
};
#pragma pack(pop)

#endif
