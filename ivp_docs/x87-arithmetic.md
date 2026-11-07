# Extended arithmetic

Enable `IVP_ENABLE_X87_EMULATION` for the original evaluation and write-back
sequences. The option defaults to `ON`; exact original replay checks use it.

```sh
cmake -S . -B build -DIVP_ENABLE_X87_EMULATION=ON
cmake --build build --config Release
```

## Source ownership

- `ivp_utility/ivu_x87.hxx` and `.cxx` own extended values, arithmetic precision,
  rounding, exception status, conversions and transcendental operations.
- `ivu_float.hxx`, `ivu_linear.cxx` and `ivu_quat.cxx` own reciprocal square-root
  refinement, vector/matrix operations and quaternion algorithms respectively.
- Contact, collision, mass integration, controller and solver algorithms remain
  in their existing subsystem files. Local helpers share repeated calculations
  within those subsystems.
- `third_party/softfloat` embeds the pinned integer numerical sources and their
  licenses. The source manifest records the unmodified upstream files.

## Values, rounding and control

`IVP_U_Float80` represents the significand and signed exponent independently of
the host ABI. Its explicit byte conversion uses the ten-byte little-endian
extended format; object padding is never serialized.

The numerical and precision-scope headers preserve their normal library ABI
when included under a host packing directive, and restore the caller's packing
setting afterwards. The packed-consumer regression compares layouts from
separate translation units before exercising arithmetic through the library.

An `IVP_X87_Arithmetic` object owns fixed-size precision, rounding and exception
status. Operations allocate no memory. `load` converts an input exactly;
`store_float` and `store_double` round at the original memory write boundaries.
Keep a register-width intermediate as `IVP_U_Float80` when a later comparison or
operation uses it. Return it explicitly across internal numerical calls.

`IVP_X87_Control` supplies thread-local precision and rounding. Explicit control
works on every supported target. On 32-bit x86, the native control word remains
authoritative, including changes made by surrounding callers; other targets
default to 64-bit significands and nearest-even rounding. Callers that require
the same arithmetic on multiple targets should
set the control explicitly. Hull construction selects 64-bit precision and
leaves that selection active for subsequent operations.
On 32-bit x86, setting explicit control also updates the native precision and
rounding used by ordinary surrounding expressions. Reset restores the precision
and rounding saved before the first override, preserving unrelated fields.

`ivu_fpu.hxx` owns control-word access and temporary precision scopes. Event
stepping selects 64-bit precision through this interface and restores the prior
precision on normal return and stack unwinding. A bound host environment is
used by both stepping and extended arithmetic. Without a host, non-x86-32
emulation uses a thread-local control word; native builds on those processors
need no precision override. This avoids unsupported native precision requests
on x64 and ARM. Precision scopes preserve rounding and unrelated fields, and
do not change the explicit override/reset state.

The built-in anomaly manager supplies an extended gravity-based penetration
speed through `get_push_speed_penetration_extended`. Custom managers inherit an
adapter to the existing float-valued virtual function. A custom manager can
override the extended query when it needs to supply a wider result. The default
environment creates the built-in gravity manager through `create_default`.

## Compilation

Scalar API returns and memory writes are separate operations. A 32-bit MSVC ABI
adapter returns a computed value through ST0 without narrowing it prematurely.
Internal portable numerical calls pass `IVP_U_Float80` explicitly; other scalar
ABIs receive the declared float or double representation.

The arithmetic implementation and embedded numerical sources use C++11
privately; the public arithmetic header remains usable from C++98. Supported
Release builds enable interprocedural optimization across the numerical backend
and solver libraries. The numerical backend uses portable integer arithmetic
and does not depend on host `long double`, native x87 arithmetic or CPU emulation.

Hosts bind their floating-point environment through `ivu_fpu.hxx` and
`ivp_set_fpu_control_word_callbacks`, without selecting or inspecting IVP's
numerical backend. The interface is available in both build modes; native
arithmetic does not need the callbacks. With emulation enabled, this keeps
precision and rounding consistent with VxMath on all processors.
Bind before starting simulation threads, with no active precision scopes.
Changing the binding restores an outstanding explicit override on the calling
thread before replacing the callbacks. Hosts remain unaware of the backend.

The option covers the explicitly implemented kernels; it does not transform
every native floating-point expression in IVP or any third-party implementation.
The Win32 `/arch:IA32` consumer requirement remains part of this partial
coverage's evaluation/return contract. Removing it requires a separate numerical
migration and is not equivalent to disabling the software backend. The native
backend is built without `ivu_x87.cxx` or the integer numerical dependency.
