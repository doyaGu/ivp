function(ivp_enable_numerical_ipo target)
    if(IVP_X87_IPO_SUPPORTED AND (NOT DEFINED IVP_ENABLE_X87_EMULATION OR IVP_ENABLE_X87_EMULATION))
        set_target_properties(${target} PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    endif()
endfunction()

function(ivp_configure_x87_arithmetic target)
    if(NOT TARGET ivp_softfloat)
        add_subdirectory("${IVP_DIR}/third_party/softfloat"
            "${CMAKE_CURRENT_BINARY_DIR}/softfloat")
    endif()
    add_library(${target} STATIC
        "${IVP_DIR}/ivp_utility/ivu_x87.cxx"
        "${IVP_DIR}/ivp_utility/ivu_fpu.cxx")
    target_link_libraries(${target} PRIVATE ivp_softfloat)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_utility>
        $<INSTALL_INTERFACE:include/ivp>)
    target_compile_features(${target} PRIVATE cxx_std_11)
    target_compile_definitions(${target} PUBLIC IVP_ENABLE_X87_EMULATION=1)
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
