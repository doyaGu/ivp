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
follows `IVP_X87_ARITHMETIC`. Both default to `OFF` when building IVP directly.
A parent project can select them before adding IVP.

```sh
cmake -S . -B build -DIVP_X87_ARITHMETIC=SOFT
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

An `IVP_X87_Arithmetic` object owns fixed-size precision, rounding and exception
status. Operations allocate no memory. `load` converts an input exactly;
`store_float` and `store_double` round at the original memory write boundaries.
Keep a register-width intermediate as `IVP_U_Float80` when a later comparison or
operation uses it. Return it explicitly across internal numerical calls.

With 64-bit precision and nearest-even rounding, `load`, `store_float`,
`store_double`, `add`, `subtract`, `multiply` and `divide` evaluate normal and
zero operands inline with integer arithmetic (`ivu_x87_fast.hxx`) when the
result is normal or zero. Every other case, precision and rounding uses the
SoftFloat implementation. Both paths produce the same values and exception
flags; `tests/numerics/x87_fast_path_test.cxx` compares them.

With `IVP_X87_ARITHMETIC=HARD`, the same operations, `square_root`,
`round_integer` and `scale` run on the host x87 while the executing FPU also
uses 64-bit precision and nearest rounding (checked when the selection is
made); otherwise they take the software paths. Host operations raise the
host's exception flags, as the original build's do, not the object's.

With `IVP_X87_ARITHMETIC_TRANSCENDENTAL` other than `OFF`, `IVP_Inline_Math`
evaluates sine, cosine, arctangent and the exponential with the operations the
original build inlines (`fsin`, `fcos`, `fpatan` and the `f2xm1`/`fscale`
sequence), and arcsine and arccosine through `arcsine` and `arccosine`, which
follow the C runtime's x87 implementation the original build calls. No host
math library takes part, so these results do not depend on the platform's
libm. The runtime returns asin and acos rounded to binary64 unless the control
selects 53-bit precision with nearest rounding. `HARD` executes the
transcendental instructions on the host under the same condition as above.

`IVP_X87_Control` supplies thread-local precision and rounding. Explicit control
works on every supported target. On 32-bit MSVC, the native control word remains
authoritative, including changes made by surrounding callers; other targets
default to 64-bit significands and nearest-even rounding. Callers that require
the same arithmetic on multiple targets should
set the control explicitly. Hull construction selects 64-bit precision and
leaves that selection active for subsequent operations.
On 32-bit MSVC, setting explicit control also updates the native precision and
rounding used by ordinary surrounding expressions. Reset restores the native
control saved before the first override.

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
and does not depend on host `long double`, native x87 arithmetic or CPU emulation, except for the `HARD` selections.

With the option disabled, the embedded dependency and arithmetic library are
excluded. The original subsystem algorithms use their ordinary floating-point
expressions. Behavioral decisions are implemented directly, independently of
the arithmetic option.
