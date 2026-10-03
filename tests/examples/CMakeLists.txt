add_executable(ivp_sample_lifecycle_test app_lifecycle.cxx)
target_link_libraries(ivp_sample_lifecycle_test PRIVATE ivp_sample_common)
configure_ivp_sample_target(ivp_sample_lifecycle_test)
add_test(NAME ivp_sample_lifecycle COMMAND ivp_sample_lifecycle_test)

add_executable(ivp_sample_renderer_failure_test renderer_failure.c)
target_link_libraries(ivp_sample_renderer_failure_test PRIVATE SDL3::SDL3 ivp_sample_glad)
if(UNIX)
    target_link_libraries(ivp_sample_renderer_failure_test PRIVATE m ${CMAKE_DL_LIBS})
endif()
configure_ivp_sample_target(ivp_sample_renderer_failure_test)
add_test(NAME ivp_sample_renderer_failure COMMAND ivp_sample_renderer_failure_test)

find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
add_test(NAME ivp_sample_smoke_results
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_smoke_test_examples.py"
)
