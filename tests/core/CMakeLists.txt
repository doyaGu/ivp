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
