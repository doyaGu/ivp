# IVP graphical examples

Build the examples from the repository root:

```sh
cmake -S . -B build -DIVP_BUILD_EXAMPLES=ON -DNUKLEAR_DIR=/path/to/Nuklear
cmake --build build --parallel
./build/ivp_examples/qs_ball
```

`IVP_BUILD_EXAMPLES` defaults to `OFF`. The examples use the IVP engine
targets defined by the root project; they are not a standalone CMake project.

Dependencies:

- CMake 3.20 or newer, a C99 compiler, and a C++98 compiler.
- SDL3: an installed CMake package is preferred. If unavailable, CMake fetches
  SDL `release-3.2.0` and builds its shared library.
- Nuklear: set `NUKLEAR_DIR` to a checkout containing `nuklear.h`. The default
  is `Nuklear/` under the repository root.
- An OpenGL 3.3 core context for running the graphical examples.

The renderer lives in `render/` and the OpenGL loader is bundled in
`deps/glad/`. No external engine checkout is needed for either.

All shared build targets use the `ivp_sample_` prefix. Compiler and platform
settings for project-owned code are configured in one CMake helper; engine
headers are supplied by the linked IVP targets.

The renderer was migrated from this repository's historical commit
`5aa92bd3098f252ebe678624ee0ab4e00a536b04`, with the configurable sphere
tessellation API retained for the current examples.

The loader was generated with glad 0.1.36, using its packaged specifications:

```sh
python -m glad --profile core --api gl=3.3 --generator c --extensions '' \
  --reproducible --out-path ivp_examples/deps/glad
```

Generating the loader is only needed when updating the bundled sources.
Its license notices are in `deps/glad/LICENSE` and the generated headers.

For a headless startup check:

```sh
python3 tools/smoke_test_examples.py --build-dir build/ivp_examples --timeout 1
```

The harness uses SDL's dummy video driver. Samples that cannot create an
OpenGL window with that driver are reported as skipped; this check does not
verify rendering or interactive controls.

To run the regression tests without an OpenGL window:

```sh
cmake -S . -B build -DIVP_BUILD_EXAMPLES=ON -DIVP_BUILD_EXAMPLE_TESTS=ON \
  -DNUKLEAR_DIR=/path/to/Nuklear
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

These tests require Python 3.10 or newer. They cover object deletion followed
by scene reset using real engine objects, renderer cleanup with injected shader
failures, and smoke-result classification. A reset restores surviving objects
and the camera; it does not recreate deleted objects.
