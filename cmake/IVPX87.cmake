include("${CMAKE_CURRENT_LIST_DIR}/IVPX87Modes.cmake")

function(ivp_enable_numerical_ipo target)
    if(IVP_X87_IPO_SUPPORTED)
        set_target_properties(${target} PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    endif()
endfunction()

function(ivp_configure_x87_arithmetic target)
    if(NOT TARGET ivp_softfloat)
        add_subdirectory("${IVP_DIR}/third_party/softfloat"
            "${CMAKE_CURRENT_BINARY_DIR}/softfloat")
    endif()
    add_library(${target} STATIC "${IVP_DIR}/ivp_utility/ivu_x87.cxx")
    target_link_libraries(${target} PRIVATE ivp_softfloat)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_utility>
        $<INSTALL_INTERFACE:include/ivp>)
    target_compile_features(${target} PRIVATE cxx_std_11)
    if(NOT DEFINED IVP_X87_ARITHMETIC_VALUE)
        # Standalone use (tests/numerics): resolve the selections here.
        if(NOT DEFINED IVP_X87_ARITHMETIC OR IVP_X87_ARITHMETIC STREQUAL "")
            set(IVP_X87_ARITHMETIC SOFT)
        endif()
        ivp_resolve_x87_modes()
    endif()
    target_compile_definitions(${target} PUBLIC
        IVP_ENABLE_X87_EMULATION=$<BOOL:${IVP_ENABLE_X87_EMULATION}>
        IVP_X87_ARITHMETIC=${IVP_X87_ARITHMETIC_VALUE}
        IVP_X87_ARITHMETIC_TRANSCENDENTAL=${IVP_X87_ARITHMETIC_TRANSCENDENTAL_VALUE})
    set_target_properties(${target} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON)

    include(CheckIPOSupported)
    check_ipo_supported(RESULT supported LANGUAGES CXX)
    set(IVP_X87_IPO_SUPPORTED "${supported}" CACHE INTERNAL
        "Compiler support for numerical interprocedural optimization")
    ivp_enable_numerical_ipo(ivp_softfloat)
    ivp_enable_numerical_ipo(${target})
endfunction()
