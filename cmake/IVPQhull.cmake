# Build only libqhull_r, keeping upstream tools and targets out of IVP's package.
include(FetchContent)

function(ivp_configure_reentrant_qhull target)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    FetchContent_Declare(ivp_qhull_modern
        URL https://github.com/qhull/qhull/archive/refs/tags/v8.0.2.tar.gz
        URL_HASH SHA256=8774e9a12c70b0180b95d6b0b563c5aa4bea8d5960c15e18ae3b6d2521d64f8b
        TIMEOUT 60
        SOURCE_SUBDIR src/libqhull_r
    )
    if(CMAKE_VERSION VERSION_LESS 3.18)
        # SOURCE_SUBDIR in MakeAvailable requires CMake 3.18. Preserve support
        # for the project's 3.12 minimum without configuring upstream's root.
        FetchContent_GetProperties(ivp_qhull_modern)
        if(NOT ivp_qhull_modern_POPULATED)
            FetchContent_Populate(ivp_qhull_modern)
        endif()
    else()
        FetchContent_MakeAvailable(ivp_qhull_modern)
    endif()

    set(qhull_src "${ivp_qhull_modern_SOURCE_DIR}/src/libqhull_r")
    add_library(ivp_qhull_vendor OBJECT
        ${qhull_src}/global_r.c
        ${qhull_src}/stat_r.c
        ${qhull_src}/geom2_r.c
        ${qhull_src}/poly2_r.c
        ${qhull_src}/merge_r.c
        ${qhull_src}/libqhull_r.c
        ${qhull_src}/geom_r.c
        ${qhull_src}/poly_r.c
        ${qhull_src}/qset_r.c
        ${qhull_src}/mem_r.c
        ${qhull_src}/random_r.c
        ${qhull_src}/usermem_r.c
        ${qhull_src}/userprintf_r.c
        ${qhull_src}/io_r.c
        ${qhull_src}/user_r.c
        ${qhull_src}/rboxlib_r.c
        ${qhull_src}/userprintf_rbox_r.c
    )
    set_target_properties(ivp_qhull_vendor PROPERTIES
        C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF
        POSITION_INDEPENDENT_CODE ON
    )
    if(MSVC)
        target_compile_options(ivp_qhull_vendor PRIVATE /W0 /fp:strict)
        target_compile_definitions(ivp_qhull_vendor PRIVATE _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(ivp_qhull_vendor PRIVATE -w -fno-fast-math -ffp-contract=off)
    endif()

    # Embed vendor objects in the existing archive so exported CMake targets
    # and pkg-config consumers need no separate Qhull package or archive.
    target_sources(${target} PRIVATE $<TARGET_OBJECTS:ivp_qhull_vendor>)
    target_include_directories(${target} SYSTEM PRIVATE "${ivp_qhull_modern_SOURCE_DIR}/src")
    target_compile_definitions(${target} PRIVATE IVP_QHULL_REENTRANT)
    if(UNIX)
        target_link_libraries(${target} PRIVATE m)
    endif()
    set(IVP_QHULL_LICENSE_FILE "${ivp_qhull_modern_SOURCE_DIR}/COPYING.txt" PARENT_SCOPE)
endfunction()
