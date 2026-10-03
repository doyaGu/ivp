cmake_minimum_required(VERSION 3.20)
enable_language(C)

# These examples consume the engine targets from the parent build.
set(IVP_SAMPLE_ENGINE_LIBS
    ivp_physics
    ivp_collision
    ivp_compact_builder
    ivp_surface_manager
    ivp_controller
    ivp_utility
)
foreach(ivp_target IN LISTS IVP_SAMPLE_ENGINE_LIBS)
    if(NOT TARGET ${ivp_target})
        message(FATAL_ERROR
            "ivp_examples requires parent engine target ${ivp_target}. "
            "Configure the repository root with -DIVP_BUILD_EXAMPLES=ON.")
    endif()
endforeach()

# External dependencies
set(NUKLEAR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Nuklear" CACHE PATH
    "Path to a Nuklear checkout containing nuklear.h")
if(NOT EXISTS "${NUKLEAR_DIR}/nuklear.h")
    message(FATAL_ERROR
        "Nuklear header missing: ${NUKLEAR_DIR}/nuklear.h. "
        "Set -DNUKLEAR_DIR=<path to Nuklear checkout>.")
endif()

find_package(SDL3 QUIET CONFIG)
if(NOT TARGET SDL3::SDL3)
    include(FetchContent)
    FetchContent_Declare(SDL3
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-3.2.0
        GIT_SHALLOW TRUE
    )
    set(SDL_SHARED ON CACHE BOOL "" FORCE)
    set(SDL_STATIC OFF CACHE BOOL "" FORCE)
    set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
    set(SDL_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(SDL3)
endif()

# Shared compiler settings for project-owned sample code.
function(configure_ivp_sample_target target)
    set_target_properties(${target} PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF
        CXX_STANDARD 98
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
    if(MSVC)
        target_compile_options(${target} PRIVATE /W3 /wd4996 /wd4244 /wd4267)
    else()
        target_compile_options(${target} PRIVATE -w
            "$<$<COMPILE_LANGUAGE:CXX>:-fpermissive>"
        )
    endif()
    if(WIN32)
        target_compile_definitions(${target} PRIVATE
            WIN32 _WINDOWS _CRT_SECURE_NO_WARNINGS
        )
    endif()
endfunction()

# Bundled OpenGL 3.3 core loader
add_library(ivp_sample_glad STATIC deps/glad/src/glad.c)
target_include_directories(ivp_sample_glad PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/glad/include"
)

# Renderer
add_library(ivp_sample_render STATIC render/render.c render/render.h)
target_include_directories(ivp_sample_render PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/render"
)
target_link_libraries(ivp_sample_render PUBLIC SDL3::SDL3 ivp_sample_glad)
if(UNIX)
    target_link_libraries(ivp_sample_render PUBLIC m ${CMAKE_DL_LIBS})
endif()
configure_ivp_sample_target(ivp_sample_render)

# Common application, GUI, and interaction code
add_library(ivp_sample_common STATIC
    ive_sample_common.cxx
    ive_sample_interaction.cxx
    ive_sample_app.cxx
    ive_gizmo.cxx
    ive_nuklear_impl.c
)
target_include_directories(ivp_sample_common PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${NUKLEAR_DIR}"
)
target_link_libraries(ivp_sample_common PUBLIC
    ivp_sample_render ${IVP_SAMPLE_ENGINE_LIBS}
)
configure_ivp_sample_target(ivp_sample_common)

function(add_ivp_sample name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ivp_sample_common)
    configure_ivp_sample_target(${name})
    if(TARGET SDL3::SDL3-shared)
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:SDL3::SDL3-shared> $<TARGET_FILE_DIR:${name}>
            VERBATIM
        )
    endif()
endfunction()

# Quickstart samples
add_ivp_sample(qs_ball               quickstart/ive_quickstart_ball.cxx)
add_ivp_sample(qs_cube               quickstart/ive_quickstart_cube.cxx)
add_ivp_sample(qs_ring               quickstart/ive_quickstart_ring.cxx)
add_ivp_sample(qs_collision          quickstart/ive_quickstart_collision.cxx)
add_ivp_sample(qs_force              quickstart/ive_quickstart_force.cxx)
add_ivp_sample(qs_actuator_spring    quickstart/ive_quickstart_actuator_spring.cxx)
add_ivp_sample(qs_stiff_spring       quickstart/ive_quickstart_stiff_spring.cxx)
add_ivp_sample(qs_actuator_force     quickstart/ive_quickstart_actuator_force.cxx)
add_ivp_sample(qs_rotmot             quickstart/ive_quickstart_rotmot.cxx)
add_ivp_sample(qs_rope               quickstart/ive_quickstart_rope.cxx)
add_ivp_sample(qs_hinge              quickstart/ive_quickstart_hinge.cxx)
add_ivp_sample(qs_active_value       quickstart/ive_quickstart_active_value.cxx)
add_ivp_sample(qs_callback           quickstart/ive_quickstart_callback.cxx)
add_ivp_sample(qs_check_distance     quickstart/ive_quickstart_check_distance.cxx)
add_ivp_sample(qs_collision_filter   quickstart/ive_quickstart_collision_filter.cxx)
add_ivp_sample(qs_cut                quickstart/ive_quickstart_cut.cxx)
add_ivp_sample(qs_motion_controller  quickstart/ive_quickstart_motion_controller.cxx)
add_ivp_sample(qs_buoyancy           quickstart/ive_quickstart_buoyancy.cxx)
add_ivp_sample(qs_maxforce           quickstart/ive_quickstart_maxforce.cxx)
add_ivp_sample(qs_phantom            quickstart/ive_quickstart_phantom.cxx)

# Advanced examples
add_ivp_sample(ex_black_hole         examples/ive_example_black_hole.cxx)
add_ivp_sample(ex_domino             examples/ive_example_domino.cxx)
add_ivp_sample(ex_snowflake          examples/ive_example_snowflake.cxx)
add_ivp_sample(ex_onthefly           examples/ive_example_onthefly.cxx)
add_ivp_sample(ex_vehicle            examples/ive_example_vehicle.cxx)
add_ivp_sample(ex_bridge             examples/ive_example_bridge.cxx)
add_ivp_sample(ex_universe           examples/ive_example_universe.cxx)
add_ivp_sample(ex_raycasting         examples/ive_example_raycasting.cxx)

if(IVP_BUILD_EXAMPLE_TESTS)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../tests/examples"
        "${CMAKE_CURRENT_BINARY_DIR}/tests")
endif()
