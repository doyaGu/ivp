add_executable(ivp_deferred_deletion_test deferred_deletion_reentrancy.cxx)
target_link_libraries(ivp_deferred_deletion_test PRIVATE
    ivp_physics
    ivp_controller
    ivp_compact_builder
)
set_target_properties(ivp_deferred_deletion_test PROPERTIES
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
add_test(NAME ivp_deferred_deletion COMMAND ivp_deferred_deletion_test)

add_executable(ivp_matrix_copy_test matrix_copy_contract.cxx)
target_link_libraries(ivp_matrix_copy_test PRIVATE ivp_physics)
set_target_properties(ivp_matrix_copy_test PROPERTIES
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
add_test(NAME ivp_matrix_copy_contract COMMAND ivp_matrix_copy_test)

add_executable(ivp_legacy_defects_test legacy_defects.cxx)
target_link_libraries(ivp_legacy_defects_test PRIVATE
    ivp_physics
    ivp_controller
    ivp_compact_builder
)
set_target_properties(ivp_legacy_defects_test PROPERTIES
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

foreach(defect
    time_event
    car_builder
    merge_duplicate
    attach_self
    matrix_cache
    sphere_grid
    merged_core
    zero_inertia
)
    add_test(
        NAME ivp_legacy_${defect}
        COMMAND ivp_legacy_defects_test ${defect}
    )
endforeach()

set_tests_properties(ivp_legacy_merged_core PROPERTIES TIMEOUT 5)

add_executable(ivp_pointsoup_test pointsoup_backend.cxx)
target_link_libraries(ivp_pointsoup_test PRIVATE ivp_compact_builder)
set_target_properties(ivp_pointsoup_test PROPERTIES
    CXX_STANDARD 98 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
)
if(IVP_INCLUDE_QHULL)
    target_compile_definitions(ivp_pointsoup_test PRIVATE IVP_TEST_QHULL_ENABLED)
endif()
add_test(NAME ivp_pointsoup_backend COMMAND ivp_pointsoup_test)
set_tests_properties(ivp_pointsoup_backend PROPERTIES TIMEOUT 30)

if(IVP_INCLUDE_QHULL AND IVP_REENTRANT_QHULL)
    find_package(Threads REQUIRED)
    add_executable(ivp_pointsoup_parallel_test pointsoup_backend.cxx)
    target_compile_definitions(ivp_pointsoup_parallel_test PRIVATE
        IVP_TEST_QHULL_ENABLED IVP_TEST_QHULL_PARALLEL
    )
    target_link_libraries(ivp_pointsoup_parallel_test PRIVATE
        ivp_compact_builder Threads::Threads
    )
    set_target_properties(ivp_pointsoup_parallel_test PROPERTIES
        CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
    )
    add_test(NAME ivp_pointsoup_parallel COMMAND ivp_pointsoup_parallel_test)
    set_tests_properties(ivp_pointsoup_parallel PROPERTIES TIMEOUT 30)
endif()
