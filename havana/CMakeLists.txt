cmake_minimum_required(VERSION 3.20)

set(HAVANA_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(HAVOK_ROOT ${HAVANA_DIR}/havok)

file(GLOB_RECURSE HK_BASE_SOURCES CONFIGURE_DEPENDS
    ${HAVOK_ROOT}/hk_base/*.cpp
)

file(GLOB_RECURSE HK_MATH_SOURCES CONFIGURE_DEPENDS
    ${HAVOK_ROOT}/hk_math/*.cpp
)

file(GLOB_RECURSE HK_PHYSICS_SOURCES CONFIGURE_DEPENDS
    ${HAVOK_ROOT}/hk_physics/*.cpp
)

add_library(hk_base STATIC ${HK_BASE_SOURCES})
add_library(hk_math STATIC ${HK_MATH_SOURCES})
add_library(hk_physics STATIC ${HK_PHYSICS_SOURCES})

function(hk_setup_target TARGET_NAME)
    target_include_directories(${TARGET_NAME} PUBLIC
        $<BUILD_INTERFACE:${HAVOK_ROOT}>
        $<BUILD_INTERFACE:${HAVANA_DIR}>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_utility>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_surface_manager>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_collision>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_intern>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_physics>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_controller>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_compact_builder>
        $<INSTALL_INTERFACE:include/havana/havok>
    )

    target_compile_definitions(${TARGET_NAME} PRIVATE IVP_VERSION_SDK)

    set_target_properties(${TARGET_NAME} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        LINK_INTERFACE_MULTIPLICITY 4
    )

    if(MSVC)
        target_compile_options(${TARGET_NAME} PRIVATE
            /W3
            /wd4244
            /wd4267
            /wd4996
        )
    else()
        target_compile_options(${TARGET_NAME} PRIVATE
            -w
            -fpermissive
        )
    endif()

    if(WIN32)
        target_compile_definitions(${TARGET_NAME} PRIVATE
            WIN32
            _WINDOWS
            _CRT_SECURE_NO_WARNINGS
        )
    endif()

    if(UNIX)
        target_compile_definitions(${TARGET_NAME} PRIVATE UNIX)
        if(CMAKE_SYSTEM_NAME MATCHES "Linux")
            target_compile_definitions(${TARGET_NAME} PRIVATE LINUX)
        endif()
    endif()

    if(APPLE)
        target_compile_definitions(${TARGET_NAME} PRIVATE APPLE)
        target_compile_options(${TARGET_NAME} PRIVATE -Wno-deprecated-declarations)
    endif()
endfunction()

hk_setup_target(hk_base)
hk_setup_target(hk_math)
hk_setup_target(hk_physics)

target_link_libraries(hk_math PUBLIC hk_base)

target_include_directories(hk_physics PUBLIC $<INSTALL_INTERFACE:include/ivp>)
target_compile_definitions(hk_physics PRIVATE HAVANA_CONSTRAINTS HAVOK_MOPP)
target_compile_definitions(hk_physics PUBLIC HAVANA_CONSTRAINTS HAVOK_MOPP)

target_link_libraries(hk_physics PUBLIC
    hk_base
    hk_math
    ivp_physics
)

if(APPLE)
    target_link_options(hk_physics INTERFACE "LINKER:-no_warn_duplicate_libraries")
endif()
