cmake_minimum_required(VERSION 3.12)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(ivp_numerics_tests LANGUAGES CXX)
    enable_testing()
    get_filename_component(IVP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
    include("${IVP_DIR}/cmake/IVPX87.cmake")
    ivp_configure_x87_arithmetic(ivp_x87_arithmetic)
endif()

if(IVP_NUMERICS_WASM)
    add_executable(ivp_softfloat_wasm softfloat_wasm.cxx)
    target_link_libraries(ivp_softfloat_wasm PRIVATE ivp_softfloat ivp_x87_arithmetic)
    target_compile_features(ivp_softfloat_wasm PRIVATE cxx_std_11)
    set_target_properties(ivp_softfloat_wasm PROPERTIES
        SUFFIX ".wasm"
        LINK_FLAGS "-nostdlib -Wl,--no-entry -Wl,--export=ivp_softfloat_arithmetic -Wl,--export=__wasm_call_ctors")
    ivp_enable_numerical_ipo(ivp_softfloat_wasm)
    find_program(IVP_NODE_EXECUTABLE node)
    if(IVP_NODE_EXECUTABLE)
        add_test(NAME ivp_x87_wasm_precision
            COMMAND "${IVP_NODE_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/run_wasm.cjs"
                $<TARGET_FILE:ivp_softfloat_wasm>)
    endif()
    return()
endif()

add_executable(ivp_softfloat_test softfloat_arithmetic_test.cxx)
target_link_libraries(ivp_softfloat_test PRIVATE ivp_softfloat)
target_compile_features(ivp_softfloat_test PRIVATE cxx_std_11)
add_test(NAME ivp_softfloat_arithmetic COMMAND ivp_softfloat_test)

add_executable(ivp_arithmetic_test x87_precision_test.cxx)
target_link_libraries(ivp_arithmetic_test PRIVATE ivp_x87_arithmetic)
set_target_properties(ivp_arithmetic_test PROPERTIES
    CXX_STANDARD 98 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
add_test(NAME ivp_x87_precision COMMAND ivp_arithmetic_test)

add_executable(ivp_runtime_state_test x87_runtime_state_test.cxx)
target_link_libraries(ivp_runtime_state_test PRIVATE ivp_x87_arithmetic)
set_target_properties(ivp_runtime_state_test PROPERTIES
    CXX_STANDARD 98 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
add_test(NAME ivp_x87_runtime_state COMMAND ivp_runtime_state_test)

add_executable(ivp_fast_path_test x87_fast_path_test.cxx)
target_link_libraries(ivp_fast_path_test PRIVATE ivp_x87_arithmetic ivp_softfloat)
target_compile_features(ivp_fast_path_test PRIVATE cxx_std_11)
add_test(NAME ivp_x87_fast_path COMMAND ivp_fast_path_test)

add_executable(ivp_packed_arithmetic_test packed_arithmetic_test.cxx packed_layout_reference.cxx)
target_link_libraries(ivp_packed_arithmetic_test PRIVATE ivp_x87_arithmetic)
set_target_properties(ivp_packed_arithmetic_test PROPERTIES
    CXX_STANDARD 98 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
add_test(NAME ivp_packed_arithmetic COMMAND ivp_packed_arithmetic_test)

add_executable(ivp_softfloat_benchmark softfloat_benchmark.cxx)
target_link_libraries(ivp_softfloat_benchmark PRIVATE ivp_softfloat)
target_compile_features(ivp_softfloat_benchmark PRIVATE cxx_std_11)

add_executable(ivp_arithmetic_benchmark softfloat_benchmark.cxx)
target_compile_definitions(ivp_arithmetic_benchmark PRIVATE IVP_BENCHMARK_ARITHMETIC)
target_link_libraries(ivp_arithmetic_benchmark PRIVATE ivp_x87_arithmetic)
target_compile_features(ivp_arithmetic_benchmark PRIVATE cxx_std_11)

foreach(target ivp_softfloat_test ivp_arithmetic_test ivp_fast_path_test ivp_runtime_state_test
    ivp_softfloat_benchmark ivp_arithmetic_benchmark)
    ivp_enable_numerical_ipo(${target})
endforeach()
