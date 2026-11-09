# Extended arithmetic

Two settings select the arithmetic, each `OFF`, `SOFT` or `HARD`:

| Setting | `OFF` | `SOFT` | `HARD` |
|---|---|---|---|
| `IVP_X87_ARITHMETIC`: the restored x87 evaluation and write-back sequences | ordinary floating-point expressions | portable integer arithmetic | the host x87 |
| `IVP_X87_ARITHMETIC_TRANSCENDENTAL`: `IVP_Inline_Math`'s sine, cosine, arcsine, arccosine, arctangent and exponential | the host C library | the x87 instructions and C runtime sequences in software | the host x87 instructions |

`SOFT` gives the same results on every target. `HARD` needs a 32-bit x86
target and gives the same results as `SOFT` for the arithmetic; the
transcendental instructions' last bit differs between processor vendors, so a
`HARD` transcendental selection follows the executing processor, as the
original build does.

An empty `IVP_X87_ARITHMETIC` follows `IVP_ENABLE_X87_EMULATION` (`SOFT` when
it is `ON`, otherwise `OFF`); an empty `IVP_X87_ARITHMETIC_TRANSCENDENTAL`
follows `IVP_X87_ARITHMETIC`. Both default to `SOFT` when building IVP directly, preserving the default
`IVP_ENABLE_X87_EMULATION=ON`.
A parent project can select them before adding IVP.

```sh
cmake -S . -B build -DIVP_X87_ARITHMETIC=SOFT
cmake --build build --config Release
```

## Source ownership

- The `third_party/libx87` submodule owns extended values, arithmetic precision,
  rounding, exception status, conversions, fast/native paths and transcendentals.
- `ivp_utility/ivu_x87.hxx` and `.cxx` adapt IVP constructors and thread control
  to the shared implementation.
- `ivu_float.hxx`, `ivu_linear.cxx` and `ivu_quat.cxx` own reciprocal square-root
  refinement, vector/matrix operations and quaternion algorithms respectively.
- Contact, collision, mass integration, controller and solver algorithms remain
  in their existing subsystem files. Local helpers share repeated calculations
  within those subsystems.
- `third_party/libx87/third_party/softfloat` contains the pinned numerical
  sources, licenses and the manifest of unmodified upstream files.

Initialize dependencies with `git submodule update --init --recursive`.
Installed IVP packages resolve `libx87::x87` with `find_dependency(libx87 CONFIG)`.

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

With 64-bit precision and nearest-even rounding, `load`, `store_float`,
`store_double`, `add`, `subtract`, `multiply` and `divide` evaluate normal and
zero operands inline with integer arithmetic (`libx87/detail/fast.hxx`) when the
result is normal or zero. Every other case, precision and rounding uses the
SoftFloat implementation. Both paths produce the same values and exception
flags; `tests/numerics/x87_fast_path_test.cxx` compares them.

With `IVP_X87_ARITHMETIC=HARD`, the same operations, `square_root`,
`round_integer` and `scale` run on the host x87 while the executing FPU also
uses 64-bit precision and nearest rounding (checked before every operation);
otherwise they take the software paths. Host operations raise the
host's exception flags, as the original build's do, not the object's.

With `IVP_X87_ARITHMETIC_TRANSCENDENTAL` other than `OFF`, `IVP_Inline_Math`
evaluates sine, cosine, arctangent and the exponential with the operations the
original build inlines (`fsin`, `fcos`, `fpatan` and the `f2xm1`/`fscale`
sequence), and arcsine and arccosine through `arcsine` and `arccosine`, which
follow the C runtime's x87 implementation the original build calls. No host
math library takes part, so these results do not depend on the platform's
libm. The runtime returns asin and acos rounded to binary64 unless the control
selects 53-bit precision with nearest rounding. Software runtime sequences
merge their exception status into the calling arithmetic object, including
inexact results and invalid arguments. `HARD` executes the
transcendental instructions on the host under the same condition as above.

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
and does not depend on host `long double`, native x87 arithmetic or CPU emulation,
except for the `HARD` selections.

Hosts bind their floating-point environment through `ivu_fpu.hxx` and
`ivp_set_fpu_control_word_callbacks`, without selecting or inspecting IVP's
numerical backend. The interface is available in both build modes; native
arithmetic does not need the callbacks. With either software arithmetic or
software transcendentals enabled, this keeps
precision and rounding consistent with VxMath on all processors.
Bind before starting simulation threads, with no active precision scopes.
Changing the binding restores an outstanding explicit override on the calling
thread before replacing the callbacks. Hosts remain unaware of the backend.

The option covers the explicitly implemented kernels; it does not transform
every native floating-point expression in IVP or any third-party implementation.
The Win32 `/arch:IA32` consumer requirement remains part of this partial
coverage's x87 intermediate precision and floating-point return values. Removing it requires a separate numerical
migration and is not equivalent to disabling the software backend. The native
backend is built without `ivu_x87.cxx` or the integer numerical dependency.
