# Extended arithmetic

`IVP_ENABLE_X87_EMULATION` selects software arithmetic for the restored x87
evaluation and write-back sequences. It defaults to `OFF` when building IVP
directly. A parent project can select the option before adding IVP.

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

An `IVP_X87_Arithmetic` object owns fixed-size precision, rounding and exception
status. Operations allocate no memory. `load` converts an input exactly;
`store_float` and `store_double` round at the original memory write boundaries.
Keep a register-width intermediate as `IVP_U_Float80` when a later comparison or
operation uses it. Return it explicitly across internal numerical calls.

`IVP_X87_Control` supplies thread-local precision and rounding. Explicit control
works on every supported target. Without an override, 32-bit MSVC imports the
native x87 control word; other targets use 64-bit significands and nearest-even
rounding. Callers that require the same arithmetic on multiple targets should
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

The arithmetic implementation and embedded numerical sources use C++11
privately; the public arithmetic header remains usable from C++98. Supported
Release builds enable interprocedural optimization across the numerical backend
and solver libraries. The implementation uses portable integer arithmetic and
does not depend on host `long double`, native x87 instructions or CPU emulation.

With the option disabled, the embedded dependency and arithmetic library are
excluded. The original subsystem algorithms use their ordinary floating-point
expressions. Behavioral decisions are implemented directly, independently of
the arithmetic option.
