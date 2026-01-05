# Ipion Virtual Physics
cmake_minimum_required(VERSION 3.12)
project(ivp VERSION 1.0.0 LANGUAGES CXX)

# Determine if IVP is built as a subproject (using add_subdirectory) or standalone
if(NOT DEFINED IVP_IS_SUBPROJECT)
    get_directory_property(IVP_PARENT_DIR PARENT_DIRECTORY)
    if(IVP_PARENT_DIR)
        set(IVP_IS_SUBPROJECT ON)
    else()
        set(IVP_IS_SUBPROJECT OFF)
    endif()
    set(IVP_IS_SUBPROJECT ${IVP_IS_SUBPROJECT} CACHE BOOL "Whether IVP is a subproject")
endif()

# Generate compile_commands.json for IDE integration (only for standalone builds)
if(NOT IVP_IS_SUBPROJECT)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# Set C++98 standard (only if not already set by parent project)
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 98)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# Set IVP root directory
set(IVP_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# ============================================================================
# External Dependencies Configuration
# ============================================================================

# Options for including external dependencies in compact builder
# Use CACHE INTERNAL to avoid polluting parent project's cache when used as submodule
if(IVP_IS_SUBPROJECT)
    option(IVP_INCLUDE_GEOMPACK "Include geompack convex decomposition" ON)
    option(IVP_INCLUDE_3DSIMPORT "Include 3D Studio import library" ON)
    option(IVP_INCLUDE_QHULL "Include Qhull computational geometry library" ON)
    mark_as_advanced(IVP_INCLUDE_GEOMPACK IVP_INCLUDE_3DSIMPORT IVP_INCLUDE_QHULL)
else()
    option(IVP_INCLUDE_GEOMPACK "Include geompack convex decomposition" ON)
    option(IVP_INCLUDE_3DSIMPORT "Include 3D Studio import library" ON)
    option(IVP_INCLUDE_QHULL "Include Qhull computational geometry library" ON)
endif()

# Compiler options for C++98 compatibility
if(MSVC)
    # Microsoft Visual C++ options
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Zi")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
else()
    # GCC and Clang options
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fpermissive")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif()

# ============================================================================
# Common Library Function
# ============================================================================

function(add_ivp_library LIBRARY_NAME SOURCE_DIR DEPENDENCIES SOURCES)
    # Create the library
    add_library(${LIBRARY_NAME} STATIC ${SOURCES})
    
    # Set include directories - include all IVP modules for cross-module includes
    target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${SOURCE_DIR}>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_utility>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_surface_manager>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_collision>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_intern>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_physics>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_controller>
        $<BUILD_INTERFACE:${IVP_DIR}/ivp_compact_builder>
        $<INSTALL_INTERFACE:include>
    )
    
    # Set compile definitions
    target_compile_definitions(${LIBRARY_NAME} PRIVATE IVP_VERSION_SDK)
    
    # Link with dependencies
    if(DEPENDENCIES)
        target_link_libraries(${LIBRARY_NAME} PUBLIC ${DEPENDENCIES})
    endif()
    
    # Set target properties
    set_target_properties(${LIBRARY_NAME} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
    
    # Platform-specific configurations
    if(WIN32)
        target_compile_definitions(${LIBRARY_NAME} PRIVATE
            WIN32
            _WINDOWS
            _CRT_SECURE_NO_WARNINGS
        )
        if(MSVC)
            target_compile_options(${LIBRARY_NAME} PRIVATE /W3)
            target_compile_options(${LIBRARY_NAME} PRIVATE
                /wd4996  # deprecated function warnings
                /wd4244  # conversion warnings
                /wd4267  # size_t to int warnings
            )
        endif()
    endif()
    
    if(UNIX)
        target_compile_definitions(${LIBRARY_NAME} PRIVATE UNIX)
        if(CMAKE_SYSTEM_NAME MATCHES "Linux")
            target_compile_definitions(${LIBRARY_NAME} PRIVATE LINUX)
        endif()
    endif()
    
    if(APPLE)
        target_compile_definitions(${LIBRARY_NAME} PRIVATE APPLE)
        target_compile_options(${LIBRARY_NAME} PRIVATE -Wno-deprecated-declarations)
    endif()
endfunction()

# ============================================================================
# Library Definitions - One Library Per Directory
# ============================================================================

# ivp_utility - Foundation layer (no dependencies)
set(IVP_UTILITY_SOURCES
    ${IVP_DIR}/ivp_utility/ivu_active_value.cxx
    ${IVP_DIR}/ivp_utility/ivu_active_value.hxx
    ${IVP_DIR}/ivp_utility/ivu_active_value_hash.hxx
    ${IVP_DIR}/ivp_utility/ivu_bigvector.cxx
    ${IVP_DIR}/ivp_utility/ivu_bigvector.hxx
    ${IVP_DIR}/ivp_utility/ivu_diff_hash.hxx
    ${IVP_DIR}/ivp_utility/ivu_float.hxx
    ${IVP_DIR}/ivp_utility/ivu_fvector.hxx
    ${IVP_DIR}/ivp_utility/ivu_geometry.cxx
    ${IVP_DIR}/ivp_utility/ivu_geometry.hxx
    ${IVP_DIR}/ivp_utility/ivu_hash.cxx
    ${IVP_DIR}/ivp_utility/ivu_hash.hxx
    ${IVP_DIR}/ivp_utility/ivu_linear.cxx
    ${IVP_DIR}/ivp_utility/ivu_linear.hxx
    ${IVP_DIR}/ivp_utility/ivu_linear_double.hxx
    ${IVP_DIR}/ivp_utility/ivu_linear_macros.hxx
    ${IVP_DIR}/ivp_utility/ivu_linear_PIII.hxx
    ${IVP_DIR}/ivp_utility/ivu_linear_ps2.hxx
    ${IVP_DIR}/ivp_utility/ivu_linear_software.hxx
    ${IVP_DIR}/ivp_utility/ivu_linear_willamette.hxx
    ${IVP_DIR}/ivp_utility/ivu_list.hxx
    ${IVP_DIR}/ivp_utility/ivu_mapping.cxx
    ${IVP_DIR}/ivp_utility/ivu_mapping.hxx
    ${IVP_DIR}/ivp_utility/ivu_matrix_macros.hxx
    ${IVP_DIR}/ivp_utility/ivu_memory.cxx
    ${IVP_DIR}/ivp_utility/ivu_memory.hxx
    ${IVP_DIR}/ivp_utility/ivu_min_hash.cxx
    ${IVP_DIR}/ivp_utility/ivu_min_hash.hxx
    ${IVP_DIR}/ivp_utility/ivu_min_list.cxx
    ${IVP_DIR}/ivp_utility/ivu_min_list.hxx
    ${IVP_DIR}/ivp_utility/ivu_os_dep.cxx
    ${IVP_DIR}/ivp_utility/ivu_quat.cxx
    ${IVP_DIR}/ivp_utility/ivu_quat.hxx
    ${IVP_DIR}/ivp_utility/ivu_set.hxx
    ${IVP_DIR}/ivp_utility/ivu_string.cxx
    ${IVP_DIR}/ivp_utility/ivu_string.hxx
    ${IVP_DIR}/ivp_utility/ivu_string_hash.hxx
    ${IVP_DIR}/ivp_utility/ivu_types.hxx
    ${IVP_DIR}/ivp_utility/ivu_vector.cxx
    ${IVP_DIR}/ivp_utility/ivu_vector.hxx
    ${IVP_DIR}/ivp_utility/ivu_vhash.cxx
    ${IVP_DIR}/ivp_utility/ivu_vhash.hxx
)
add_ivp_library(ivp_utility "${IVP_DIR}/ivp_utility" "" "${IVP_UTILITY_SOURCES}")
# Add global include directories for all modules
target_include_directories(ivp_utility PRIVATE
    ${IVP_DIR}/ivp_surface_manager
    ${IVP_DIR}/ivp_collision
    ${IVP_DIR}/ivp_intern
    ${IVP_DIR}/ivp_physics
    ${IVP_DIR}/ivp_controller
    ${IVP_DIR}/ivp_compact_builder
)

# ivp_surface_manager - Depends on utility
set(IVP_SURFACEMANAGER_SOURCES
    ${IVP_DIR}/ivp_surface_manager/ivp_compact_grid.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_compact_surface.cxx
    ${IVP_DIR}/ivp_surface_manager/ivp_compact_surface.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_gridbuild_array.cxx
    ${IVP_DIR}/ivp_surface_manager/ivp_gridbuild_array.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surman_grid.cxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surman_grid.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surman_polygon.cxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surman_polygon.hxx
)
add_ivp_library(ivp_surface_manager "${IVP_DIR}/ivp_surface_manager" "ivp_utility" "${IVP_SURFACEMANAGER_SOURCES}")
# Add global include directories for all modules
target_include_directories(ivp_surface_manager PRIVATE
    ${IVP_DIR}/ivp_utility
    ${IVP_DIR}/ivp_surface_manager
    ${IVP_DIR}/ivp_collision
    ${IVP_DIR}/ivp_intern
    ${IVP_DIR}/ivp_physics
    ${IVP_DIR}/ivp_controller
    ${IVP_DIR}/ivp_compact_builder
)

# ivp_collision - Depends on utility and surface manager
set(IVP_COLLISION_SOURCES
    ${IVP_DIR}/ivp_collision/ivp_3d_solver.cxx
    ${IVP_DIR}/ivp_collision/ivp_3d_solver.hxx
    ${IVP_DIR}/ivp_collision/ivp_cache_ledge_point.hxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_longrange.cxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_longrange.hxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_lrange_hash.cxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_lrange_hash.hxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_visual_hash.cxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_visual_hash.hxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_visualizer.cxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_visualizer.hxx
    ${IVP_DIR}/ivp_collision/ivp_coll_del_root_mindist.cxx
    ${IVP_DIR}/ivp_collision/ivp_collision.hxx
    ${IVP_DIR}/ivp_collision/ivp_collision_filter.cxx
    ${IVP_DIR}/ivp_collision/ivp_collision_filter.hxx
    ${IVP_DIR}/ivp_collision/ivp_compact_ledge.cxx
    ${IVP_DIR}/ivp_collision/ivp_compact_ledge.hxx
    ${IVP_DIR}/ivp_collision/ivp_compact_ledge_solver.cxx
    ${IVP_DIR}/ivp_collision/ivp_compact_ledge_solver.hxx
    ${IVP_DIR}/ivp_collision/ivp_i_collision_vhash.cxx
    ${IVP_DIR}/ivp_collision/ivp_i_collision_vhash.hxx
    ${IVP_DIR}/ivp_collision/ivp_listener_collision.hxx
    ${IVP_DIR}/ivp_collision/ivp_mindist.cxx
    ${IVP_DIR}/ivp_collision/ivp_mindist.hxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_debug.cxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_event.cxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_event.hxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_intern.hxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_macros.hxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_mcases.cxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_minimize.cxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_minimize.hxx
    ${IVP_DIR}/ivp_collision/ivp_mindist_recursive.cxx
    ${IVP_DIR}/ivp_collision/ivp_oo_watcher.cxx
    ${IVP_DIR}/ivp_collision/ivp_range_manager.cxx
    ${IVP_DIR}/ivp_collision/ivp_range_manager.hxx
    ${IVP_DIR}/ivp_collision/ivp_ray_solver.cxx
    ${IVP_DIR}/ivp_collision/ivp_ray_solver.hxx
    ${IVP_DIR}/ivp_collision/ivp_sphere_solver.cxx
    ${IVP_DIR}/ivp_collision/ivp_universe_manager.hxx
)
add_ivp_library(ivp_collision "${IVP_DIR}/ivp_collision" "ivp_utility;ivp_surface_manager" "${IVP_COLLISION_SOURCES}")
# Add global include directories for all modules
target_include_directories(ivp_collision PRIVATE
    ${IVP_DIR}/ivp_utility
    ${IVP_DIR}/ivp_surface_manager
    ${IVP_DIR}/ivp_collision
    ${IVP_DIR}/ivp_intern
    ${IVP_DIR}/ivp_physics
    ${IVP_DIR}/ivp_controller
    ${IVP_DIR}/ivp_compact_builder
)

# ivp_intern - Depends on utility, surface manager, and collision
set(IVP_INTERN_SOURCES
    ${IVP_DIR}/ivp_intern/ivp_ball.cxx
    ${IVP_DIR}/ivp_intern/ivp_calc_next_psi_solver.cxx
    ${IVP_DIR}/ivp_intern/ivp_calc_next_psi_solver.hxx
    ${IVP_DIR}/ivp_intern/ivp_controller_phantom.cxx
    ${IVP_DIR}/ivp_intern/ivp_core.cxx
    ${IVP_DIR}/ivp_intern/ivp_environment.cxx
    ${IVP_DIR}/ivp_intern/ivp_friction.cxx
    ${IVP_DIR}/ivp_intern/ivp_friction.hxx
    ${IVP_DIR}/ivp_intern/ivp_friction_gaps.cxx
    ${IVP_DIR}/ivp_intern/ivp_friction_solver.hxx
    ${IVP_DIR}/ivp_intern/ivp_great_matrix.cxx
    ${IVP_DIR}/ivp_intern/ivp_hull_manager.cxx
    ${IVP_DIR}/ivp_intern/ivp_hull_manager_macros.hxx
    ${IVP_DIR}/ivp_intern/ivp_i_controller_vhash.hxx
    ${IVP_DIR}/ivp_intern/ivp_i_friction_hash.cxx
    ${IVP_DIR}/ivp_intern/ivp_i_friction_hash.hxx
    ${IVP_DIR}/ivp_intern/ivp_i_object_vhash.cxx
    ${IVP_DIR}/ivp_intern/ivp_i_object_vhash.hxx
    ${IVP_DIR}/ivp_intern/ivp_impact.cxx
    ${IVP_DIR}/ivp_intern/ivp_impact.hxx
    ${IVP_DIR}/ivp_intern/ivp_merge_core.cxx
    ${IVP_DIR}/ivp_intern/ivp_merge_core.hxx
    ${IVP_DIR}/ivp_intern/ivp_mindist_friction.cxx
    ${IVP_DIR}/ivp_intern/ivp_object.cxx
    ${IVP_DIR}/ivp_intern/ivp_object_attach.cxx
    ${IVP_DIR}/ivp_intern/ivp_physic.cxx
    ${IVP_DIR}/ivp_intern/ivp_physic_private.cxx
    ${IVP_DIR}/ivp_intern/ivp_physic_private.hxx
    ${IVP_DIR}/ivp_intern/ivp_polygon.cxx
    ${IVP_DIR}/ivp_intern/ivp_sim_unit.cxx
    ${IVP_DIR}/ivp_intern/ivp_sim_unit.hxx
    ${IVP_DIR}/ivp_intern/ivp_solver_core_reaction.cxx
    ${IVP_DIR}/ivp_intern/ivp_time.cxx
)
add_ivp_library(ivp_intern "${IVP_DIR}/ivp_intern" "ivp_utility;ivp_surface_manager;ivp_collision" "${IVP_INTERN_SOURCES}")
# Add global include directories for all modules
target_include_directories(ivp_intern PRIVATE
    ${IVP_DIR}/ivp_utility
    ${IVP_DIR}/ivp_surface_manager
    ${IVP_DIR}/ivp_collision
    ${IVP_DIR}/ivp_intern
    ${IVP_DIR}/ivp_physics
    ${IVP_DIR}/ivp_controller
    ${IVP_DIR}/ivp_compact_builder
)

# ivp_physics - Main physics API, depends on core modules
set(IVP_PHYSICS_SOURCES
    ${IVP_DIR}/ivp_physics/ivp_anomaly_manager.cxx
    ${IVP_DIR}/ivp_physics/ivp_anomaly_manager.hxx
    ${IVP_DIR}/ivp_physics/ivp_authenticity.hxx
    ${IVP_DIR}/ivp_physics/ivp_ball.hxx
    ${IVP_DIR}/ivp_physics/ivp_betterdebugmanager.cxx
    ${IVP_DIR}/ivp_physics/ivp_betterdebugmanager.hxx
    ${IVP_DIR}/ivp_physics/ivp_betterstatisticsmanager.cxx
    ${IVP_DIR}/ivp_physics/ivp_betterstatisticsmanager.hxx
    ${IVP_DIR}/ivp_physics/ivp_cache_object.cxx
    ${IVP_DIR}/ivp_physics/ivp_cache_object.hxx
    ${IVP_DIR}/ivp_physics/ivp_contact_situation.hxx
    ${IVP_DIR}/ivp_physics/ivp_core.hxx
    ${IVP_DIR}/ivp_physics/ivp_core_macros.hxx
    ${IVP_DIR}/ivp_physics/ivp_debug.hxx
    ${IVP_DIR}/ivp_physics/ivp_debug_manager.hxx
    ${IVP_DIR}/ivp_physics/ivp_environment.hxx
    ${IVP_DIR}/ivp_physics/ivp_great_matrix.hxx
    ${IVP_DIR}/ivp_physics/ivp_hull_manager.hxx
    ${IVP_DIR}/ivp_physics/ivp_liquid_surface_descript.cxx
    ${IVP_DIR}/ivp_physics/ivp_liquid_surface_descript.hxx
    ${IVP_DIR}/ivp_physics/ivp_listener_hull.hxx
    ${IVP_DIR}/ivp_physics/ivp_listener_object.hxx
    ${IVP_DIR}/ivp_physics/ivp_listener_psi.hxx
    ${IVP_DIR}/ivp_physics/ivp_material.cxx
    ${IVP_DIR}/ivp_physics/ivp_material.hxx
    ${IVP_DIR}/ivp_physics/ivp_object.hxx
    ${IVP_DIR}/ivp_physics/ivp_object_attach.hxx
    ${IVP_DIR}/ivp_physics/ivp_performancecounter.cxx
    ${IVP_DIR}/ivp_physics/ivp_performancecounter.hxx
    ${IVP_DIR}/ivp_physics/ivp_phantom.hxx
    ${IVP_DIR}/ivp_physics/ivp_physics.hxx
    ${IVP_DIR}/ivp_physics/ivp_polygon.hxx
    ${IVP_DIR}/ivp_physics/ivp_radar.hxx
    ${IVP_DIR}/ivp_physics/ivp_real_object.hxx
    ${IVP_DIR}/ivp_physics/ivp_solver_core_reaction.hxx
    ${IVP_DIR}/ivp_physics/ivp_stat_manager_cback_con.cxx
    ${IVP_DIR}/ivp_physics/ivp_stat_manager_cback_con.hxx
    ${IVP_DIR}/ivp_physics/ivp_surface_manager.cxx
    ${IVP_DIR}/ivp_physics/ivp_surface_manager.hxx
    ${IVP_DIR}/ivp_physics/ivp_templates.cxx
    ${IVP_DIR}/ivp_physics/ivp_templates.hxx
    ${IVP_DIR}/ivp_physics/ivp_time.hxx
    ${IVP_DIR}/ivp_physics/ivp_time_event.hxx
)
add_ivp_library(ivp_physics "${IVP_DIR}/ivp_physics" "ivp_utility;ivp_surface_manager;ivp_collision;ivp_intern;ivp_controller" "${IVP_PHYSICS_SOURCES}")
# Add global include directories for all modules
target_include_directories(ivp_physics PRIVATE
    ${IVP_DIR}/ivp_utility
    ${IVP_DIR}/ivp_surface_manager
    ${IVP_DIR}/ivp_collision
    ${IVP_DIR}/ivp_intern
    ${IVP_DIR}/ivp_physics
    ${IVP_DIR}/ivp_controller
    ${IVP_DIR}/ivp_compact_builder
)

# ivp_controller - Physics controllers, depends on physics core
set(IVP_CONTROLLER_SOURCES
    ${IVP_DIR}/ivp_controller/ivp_actuator.cxx
    ${IVP_DIR}/ivp_controller/ivp_actuator.hxx
    ${IVP_DIR}/ivp_controller/ivp_actuator_info.hxx
    ${IVP_DIR}/ivp_controller/ivp_actuator_spring.cxx
    ${IVP_DIR}/ivp_controller/ivp_actuator_spring.hxx
    ${IVP_DIR}/ivp_controller/ivp_attacher_to_cores.hxx
    ${IVP_DIR}/ivp_controller/ivp_buoyancy_solver.cxx
    ${IVP_DIR}/ivp_controller/ivp_buoyancy_solver.hxx
    ${IVP_DIR}/ivp_controller/ivp_car_system.cxx
    ${IVP_DIR}/ivp_controller/ivp_car_system.hxx
    ${IVP_DIR}/ivp_controller/ivp_constraint.cxx
    ${IVP_DIR}/ivp_controller/ivp_constraint.hxx
    ${IVP_DIR}/ivp_controller/ivp_constraint_car.cxx
    ${IVP_DIR}/ivp_controller/ivp_constraint_car.hxx
    ${IVP_DIR}/ivp_controller/ivp_constraint_fixed_keyed.cxx
    ${IVP_DIR}/ivp_controller/ivp_constraint_fixed_keyed.hxx
    ${IVP_DIR}/ivp_controller/ivp_constraint_local.cxx
    ${IVP_DIR}/ivp_controller/ivp_constraint_local.hxx
    ${IVP_DIR}/ivp_controller/ivp_constraint_types.hxx
    ${IVP_DIR}/ivp_controller/ivp_controller.cxx
    ${IVP_DIR}/ivp_controller/ivp_controller.hxx
    ${IVP_DIR}/ivp_controller/ivp_controller_airboat.cpp
    ${IVP_DIR}/ivp_controller/ivp_controller_airboat.h
    ${IVP_DIR}/ivp_controller/ivp_controller_fake_jetski.cpp
    ${IVP_DIR}/ivp_controller/ivp_controller_fake_jetski.h
    ${IVP_DIR}/ivp_controller/ivp_controller_buoyancy.cxx
    ${IVP_DIR}/ivp_controller/ivp_controller_buoyancy.hxx
    ${IVP_DIR}/ivp_controller/ivp_controller_floating.cxx
    ${IVP_DIR}/ivp_controller/ivp_controller_floating.hxx
    ${IVP_DIR}/ivp_controller/ivp_controller_golem.hxx
    ${IVP_DIR}/ivp_controller/ivp_controller_motion.cxx
    ${IVP_DIR}/ivp_controller/ivp_controller_motion.hxx
    ${IVP_DIR}/ivp_controller/ivp_controller_raycast_car.cxx
    ${IVP_DIR}/ivp_controller/ivp_controller_raycast_car.hxx
    ${IVP_DIR}/ivp_controller/ivp_controller_stiff_spring.cxx
    ${IVP_DIR}/ivp_controller/ivp_controller_stiff_spring.hxx
    ${IVP_DIR}/ivp_controller/ivp_controller_world_frict.cxx
    ${IVP_DIR}/ivp_controller/ivp_controller_world_frict.hxx
    ${IVP_DIR}/ivp_controller/ivp_forcefield.cxx
    ${IVP_DIR}/ivp_controller/ivp_forcefield.hxx
    ${IVP_DIR}/ivp_controller/ivp_multidimensional_interp.cxx
    ${IVP_DIR}/ivp_controller/ivp_multidimensional_interp.hxx
    ${IVP_DIR}/ivp_controller/ivp_template_constraint.cxx
    ${IVP_DIR}/ivp_controller/ivp_template_constraint.hxx
)
add_ivp_library(ivp_controller "${IVP_DIR}/ivp_controller" "ivp_utility;ivp_physics" "${IVP_CONTROLLER_SOURCES}")
# Add global include directories for all modules
target_include_directories(ivp_controller PRIVATE
    ${IVP_DIR}/ivp_utility
    ${IVP_DIR}/ivp_surface_manager
    ${IVP_DIR}/ivp_collision
    ${IVP_DIR}/ivp_intern
    ${IVP_DIR}/ivp_physics
    ${IVP_DIR}/ivp_controller
    ${IVP_DIR}/ivp_compact_builder
)

# ivp_compact_builder - Geometry building tools, depends on core modules
set(IVP_COMPACT_BUILDER_DIR ${IVP_DIR}/ivp_compact_builder)

# Collect external library sources
set(EXTERNAL_SOURCES)

if(IVP_INCLUDE_GEOMPACK)
    list(APPEND EXTERNAL_SOURCES
        ${IVP_COMPACT_BUILDER_DIR}/geompack.hxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_cutfac.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_cvdec3.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_drdec3.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_dsphdc.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_edght.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_initcb.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_insed3.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_insfac.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_insvr3.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_prime.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_ptpolg.cxx
        ${IVP_COMPACT_BUILDER_DIR}/geompack_resedg.cxx
    )
endif()

if(IVP_INCLUDE_3DSIMPORT)
    list(APPEND EXTERNAL_SOURCES
        ${IVP_COMPACT_BUILDER_DIR}/3dsimport_co.cxx
        ${IVP_COMPACT_BUILDER_DIR}/3dsimport_load.cxx
        ${IVP_COMPACT_BUILDER_DIR}/3dsimport_load.hxx
        ${IVP_COMPACT_BUILDER_DIR}/3dsimport_out.cxx
    )
endif()

if(IVP_INCLUDE_QHULL)
    list(APPEND EXTERNAL_SOURCES
        ${IVP_COMPACT_BUILDER_DIR}/qhull.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_geom.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_geom2.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_global.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_io.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_mem.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_merge.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_poly.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_poly2.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_qset.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_stat.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_user.cxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_a.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_geom.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_io.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_mem.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_merge.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_poly.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_qset.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_stat.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull_user.hxx
        ${IVP_COMPACT_BUILDER_DIR}/qhull.hxx
    )
endif()

# Main compact builder sources
set(COMPACT_BUILDER_MAIN_SOURCES
    ${IVP_COMPACT_BUILDER_DIR}/ivp_compact_ledge_gen.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_compact_ledge_gen.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_compact_modify.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_compact_modify.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_compact_recursive.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_compact_recursive.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_convex_decompositor.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_convex_decompositor.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_halfspacesoup.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_halfspacesoup.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_i_fpoint_vhash.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_i_fpoint_vhash.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_i_point_vhash.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_i_point_vhash.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_object_polygon_tetra.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_object_polygon_tetra.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_rot_inertia_solver.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_rot_inertia_solver.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_3ds.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_halfspacesoup.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_halfspacesoup.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_ledge_soup.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_ledge_soup.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_pointsoup.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_pointsoup.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_polygon_convex.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_polygon_convex.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_polyhdrn_cncv.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_polyhdrn_cncv.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_q12.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_surbuild_q12.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_templates_intern.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_templates_intern.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_tetra_intrude.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_tetra_intrude.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivp_triangle_gen.hxx
    ${IVP_COMPACT_BUILDER_DIR}/ivv_cluster_min_hash.cxx
    ${IVP_COMPACT_BUILDER_DIR}/ivv_cluster_min_hash.hxx
)

# Combine all sources
set(ALL_COMPACT_BUILDER_SOURCES ${COMPACT_BUILDER_MAIN_SOURCES} ${EXTERNAL_SOURCES})

# Create the compact builder library
add_library(ivp_compact_builder STATIC ${ALL_COMPACT_BUILDER_SOURCES})

# Set include directories
target_include_directories(ivp_compact_builder PUBLIC
    $<BUILD_INTERFACE:${IVP_COMPACT_BUILDER_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Set compile definitions
target_compile_definitions(ivp_compact_builder PRIVATE IVP_VERSION_SDK)

# Link with dependencies
target_link_libraries(ivp_compact_builder PUBLIC 
    ivp_utility
    ivp_surface_manager
    ivp_collision
    ivp_intern
    ivp_physics
)

# Set target properties
set_target_properties(ivp_compact_builder PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Platform-specific configurations for compact builder
if(WIN32)
    target_compile_definitions(ivp_compact_builder PRIVATE
        WIN32
        _WINDOWS
        _CRT_SECURE_NO_WARNINGS
    )
    if(MSVC)
        target_compile_options(ivp_compact_builder PRIVATE /W3)
        target_compile_options(ivp_compact_builder PRIVATE
            /wd4996  # deprecated function warnings
            /wd4244  # conversion warnings
            /wd4267  # size_t to int warnings
        )
    endif()
endif()

if(UNIX)
    target_compile_definitions(ivp_compact_builder PRIVATE UNIX)
    if(CMAKE_SYSTEM_NAME MATCHES "Linux")
        target_compile_definitions(ivp_compact_builder PRIVATE LINUX)
    endif()
endif()

if(APPLE)
    target_compile_definitions(ivp_compact_builder PRIVATE APPLE)
    target_compile_options(ivp_compact_builder PRIVATE -Wno-deprecated-declarations)
endif()

# ============================================================================
# Havana Library Integration
# ============================================================================

# Check if Havana sources should be built
option(IVP_BUILD_HAVANA "Build Havana libraries alongside IVP" OFF)
if(IVP_IS_SUBPROJECT)
    mark_as_advanced(IVP_BUILD_HAVANA)
endif()

# Check if Havana directory exists
set(HAVANA_SOURCES_FOUND FALSE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/havana")
    set(HAVANA_SOURCES_FOUND TRUE)
endif()

if(IVP_BUILD_HAVANA)
    # Add Havana subdirectory if it contains CMakeLists.txt
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/havana/CMakeLists.txt")
        add_subdirectory(havana)
        # Link with Havana libraries
        target_link_libraries(ivp_physics PUBLIC hk_base hk_math hk_physics)
        target_include_directories(ivp_physics PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/havana
            ${CMAKE_CURRENT_SOURCE_DIR}/havana/havok
        )
    else()
        # Create interface libraries for Havana if not built
        add_library(hk_base INTERFACE)
        add_library(hk_math INTERFACE)
        add_library(hk_physics INTERFACE)

        target_include_directories(hk_base INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/havana/havok/hk_base
        )
        target_include_directories(hk_math INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/havana/havok/hk_math
        )
        target_include_directories(hk_physics INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/havana/havok/hk_physics
        )

        target_link_libraries(ivp_physics INTERFACE hk_base hk_math hk_physics)
    endif()
endif()

# ============================================================================
# Installation and Export Configuration
# ============================================================================

# Option to control installation (disabled by default when used as subproject)
if(IVP_IS_SUBPROJECT)
    option(IVP_INSTALL "Generate installation target" OFF)
else()
    option(IVP_INSTALL "Generate installation target" ON)
endif()

if(IVP_INSTALL)
    # Include GNUInstallDirs to use standard directories
    include(GNUInstallDirs)

    # Install libraries
    install(TARGETS ivp_utility ivp_surface_manager ivp_collision ivp_intern ivp_physics ivp_controller ivp_compact_builder
        EXPORT ivp_targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # Install header files
    install(DIRECTORY
        ${IVP_DIR}/ivp_collision/
        ${IVP_DIR}/ivp_controller/
        ${IVP_DIR}/ivp_intern/
        ${IVP_DIR}/ivp_physics/
        ${IVP_DIR}/ivp_surface_manager/
        ${IVP_DIR}/ivp_utility/
        ${IVP_DIR}/ivp_compact_builder/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ivp
        FILES_MATCHING PATTERN "*.hxx" PATTERN "*.h"
    )

    # Install Havana headers if available
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/havana")
        install(DIRECTORY
            ${CMAKE_CURRENT_SOURCE_DIR}/havana/havok/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/havana/havok
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.inl"
        )
    endif()

    # Create and install export targets
    install(EXPORT ivp_targets
        FILE ivp_targets.cmake
        NAMESPACE ivp::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ivp
    )

    # Create package configuration files
    include(CMakePackageConfigHelpers)

    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ivp-config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/ivp-config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ivp
    )

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/ivp-config-version.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/ivp-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/ivp-config-version.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ivp
    )

    # Generate pkg-config file for easier integration
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ivp.pc.in"
        "${CMAKE_CURRENT_BINARY_DIR}/ivp.pc"
        @ONLY
    )

    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ivp.pc"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    )
endif()

# ============================================================================
# Configuration Summary
# ============================================================================

if(NOT IVP_IS_SUBPROJECT)
    message(STATUS "")
    message(STATUS "IVP Configuration Summary:")
    message(STATUS "  Build Mode: Standalone")
    message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
    message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
    message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "  Install Enabled: ${IVP_INSTALL}")
    message(STATUS "  Build Havana: ${IVP_BUILD_HAVANA}")
    if(IVP_BUILD_HAVANA)
        message(STATUS "    Havana sources found: ${HAVANA_SOURCES_FOUND}")
    endif()
    message(STATUS "")
else()
    message(STATUS "IVP: Configured as subproject (installation disabled by default)")
endif()