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
option(IVP_INCLUDE_GEOMPACK "Include geompack convex decomposition" ON)
option(IVP_INCLUDE_3DSIMPORT "Include 3D Studio import library" ON)
option(IVP_INCLUDE_QHULL "Include Qhull computational geometry library" ON)
if(IVP_IS_SUBPROJECT)
    mark_as_advanced(IVP_INCLUDE_GEOMPACK IVP_INCLUDE_3DSIMPORT IVP_INCLUDE_QHULL)
endif()

# Note: compiler options are set per-target inside add_ivp_library(), not globally,
# so that subproject consumers are not affected.  CMake's built-in defaults for
# Debug (-g -O0 / /Od /Zi) and Release (-O2 / /O2) are sufficient.

# ============================================================================
# Common Library Function
# ============================================================================

function(add_ivp_library LIBRARY_NAME SOURCE_DIR DEPENDENCIES SOURCES)
    # Create the library
    add_library(${LIBRARY_NAME} STATIC ${SOURCES})
    
    # All public headers are staged flat under include/ivp/.  Consumers use
    # bare includes (#include "ivp_physics.hxx") which is the original convention.
    target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/ivp>
        $<INSTALL_INTERFACE:include/ivp>
    )
    target_include_directories(${LIBRARY_NAME} PRIVATE
        ${SOURCE_DIR}
    )
    
    # Set compile definitions
    target_compile_definitions(${LIBRARY_NAME} PRIVATE IVP_VERSION_SDK)
    
    # Link with dependencies
    if(DEPENDENCIES)
        target_link_libraries(${LIBRARY_NAME} PUBLIC ${DEPENDENCIES})
    endif()
    
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
    
    if(NOT MSVC)
        # C++98 codebase — suppress warnings and disable strict-aliasing
        # optimisations that break the original pointer-casting patterns.
        target_compile_options(${LIBRARY_NAME} PRIVATE -w -fno-strict-aliasing)
    endif()

    if(UNIX)
        target_compile_definitions(${LIBRARY_NAME} PRIVATE UNIX)
        if(CMAKE_SYSTEM_NAME MATCHES "Linux")
            target_compile_definitions(${LIBRARY_NAME} PRIVATE LINUX)
        endif()
    endif()

    if(APPLE)
        target_compile_definitions(${LIBRARY_NAME} PRIVATE APPLE)
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
    ${IVP_DIR}/ivp_utility/ivu_linear_piii.hxx
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

# ivp_surface_manager - Depends on utility
set(IVP_SURFACEMANAGER_SOURCES
    ${IVP_DIR}/ivp_surface_manager/ivp_compact_grid.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_compact_ledge.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_compact_ledge_solver.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_compact_surface.cxx
    ${IVP_DIR}/ivp_surface_manager/ivp_compact_surface.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_gridbuild_array.cxx
    ${IVP_DIR}/ivp_surface_manager/ivp_gridbuild_array.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_ray_solver.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surface_manager.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surman_grid.cxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surman_grid.hxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surman_polygon.cxx
    ${IVP_DIR}/ivp_surface_manager/ivp_surman_polygon.hxx
)
add_ivp_library(ivp_surface_manager "${IVP_DIR}/ivp_surface_manager" "ivp_utility" "${IVP_SURFACEMANAGER_SOURCES}")

# ivp_intern - Runtime implementation compiled into the public physics API
set(IVP_INTERN_SOURCES
    ${IVP_DIR}/ivp_intern/ivp_cache_ledge_point.hxx
    ${IVP_DIR}/ivp_intern/ivp_cache_object.cxx
    ${IVP_DIR}/ivp_intern/ivp_cache_object.hxx
    ${IVP_DIR}/ivp_intern/ivp_calc_next_psi_solver.cxx
    ${IVP_DIR}/ivp_intern/ivp_calc_next_psi_solver.hxx
    ${IVP_DIR}/ivp_intern/ivp_clustering_longrange.hxx
    ${IVP_DIR}/ivp_intern/ivp_friction.cxx
    ${IVP_DIR}/ivp_intern/ivp_friction.hxx
    ${IVP_DIR}/ivp_intern/ivp_friction_gaps.cxx
    ${IVP_DIR}/ivp_intern/ivp_friction_solver.hxx
    ${IVP_DIR}/ivp_intern/ivp_hull_manager.cxx
    ${IVP_DIR}/ivp_intern/ivp_hull_manager.hxx
    ${IVP_DIR}/ivp_intern/ivp_hull_manager_macros.hxx
    ${IVP_DIR}/ivp_intern/ivp_i_collision_vhash.hxx
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
    ${IVP_DIR}/ivp_intern/ivp_physic_private.cxx
    ${IVP_DIR}/ivp_intern/ivp_physic_private.hxx
    ${IVP_DIR}/ivp_intern/ivp_sim_unit.cxx
    ${IVP_DIR}/ivp_intern/ivp_sim_unit.hxx
)

# ivp_physics - Main physics API
set(IVP_PHYSICS_SOURCES
    ${IVP_DIR}/ivp_physics/ivp_anomaly_manager.cxx
    ${IVP_DIR}/ivp_physics/ivp_anomaly_manager.hxx
    ${IVP_DIR}/ivp_physics/ivp_3d_solver.cxx
    ${IVP_DIR}/ivp_physics/ivp_3d_solver.hxx
    ${IVP_DIR}/ivp_physics/ivp_authenticity.hxx
    ${IVP_DIR}/ivp_physics/ivp_ball.cxx
    ${IVP_DIR}/ivp_physics/ivp_ball.hxx
    ${IVP_DIR}/ivp_physics/ivp_betterdebugmanager.cxx
    ${IVP_DIR}/ivp_physics/ivp_betterdebugmanager.hxx
    ${IVP_DIR}/ivp_physics/ivp_betterstatisticsmanager.cxx
    ${IVP_DIR}/ivp_physics/ivp_betterstatisticsmanager.hxx
    ${IVP_DIR}/ivp_physics/ivp_collision.hxx
    ${IVP_DIR}/ivp_physics/ivp_collision_filter.cxx
    ${IVP_DIR}/ivp_physics/ivp_collision_filter.hxx
    ${IVP_DIR}/ivp_physics/ivp_contact_situation.hxx
    ${IVP_DIR}/ivp_physics/ivp_core.cxx
    ${IVP_DIR}/ivp_physics/ivp_core.hxx
    ${IVP_DIR}/ivp_physics/ivp_core_macros.hxx
    ${IVP_DIR}/ivp_physics/ivp_debug.hxx
    ${IVP_DIR}/ivp_physics/ivp_debug_manager.hxx
    ${IVP_DIR}/ivp_physics/ivp_environment.cxx
    ${IVP_DIR}/ivp_physics/ivp_environment.hxx
    ${IVP_DIR}/ivp_physics/ivp_great_matrix.cxx
    ${IVP_DIR}/ivp_physics/ivp_great_matrix.hxx
    ${IVP_DIR}/ivp_physics/ivp_liquid_surface_descript.cxx
    ${IVP_DIR}/ivp_physics/ivp_liquid_surface_descript.hxx
    ${IVP_DIR}/ivp_physics/ivp_listener_collision.hxx
    ${IVP_DIR}/ivp_physics/ivp_listener_hull.hxx
    ${IVP_DIR}/ivp_physics/ivp_listener_object.hxx
    ${IVP_DIR}/ivp_physics/ivp_listener_psi.hxx
    ${IVP_DIR}/ivp_physics/ivp_material.cxx
    ${IVP_DIR}/ivp_physics/ivp_material.hxx
    ${IVP_DIR}/ivp_physics/ivp_mindist.cxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_debug.cxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_event.cxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_event.hxx
    ${IVP_DIR}/ivp_physics/ivp_mindist.hxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_impact.cxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_macros.hxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_mcases.cxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_minimize.cxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_minimize.hxx
    ${IVP_DIR}/ivp_physics/ivp_mindist_recursive.cxx
    ${IVP_DIR}/ivp_physics/ivp_object.cxx
    ${IVP_DIR}/ivp_physics/ivp_object.hxx
    ${IVP_DIR}/ivp_physics/ivp_object_attach.cxx
    ${IVP_DIR}/ivp_physics/ivp_object_attach.hxx
    ${IVP_DIR}/ivp_physics/ivp_performancecounter.cxx
    ${IVP_DIR}/ivp_physics/ivp_performancecounter.hxx
    ${IVP_DIR}/ivp_physics/ivp_phantom.hxx
    ${IVP_DIR}/ivp_physics/ivp_phantom.cxx
    ${IVP_DIR}/ivp_physics/ivp_physic.cxx
    ${IVP_DIR}/ivp_physics/ivp_physics.hxx
    ${IVP_DIR}/ivp_physics/ivp_polygon.cxx
    ${IVP_DIR}/ivp_physics/ivp_polygon.hxx
    ${IVP_DIR}/ivp_physics/ivp_radar.hxx
    ${IVP_DIR}/ivp_physics/ivp_range_manager.cxx
    ${IVP_DIR}/ivp_physics/ivp_range_manager.hxx
    ${IVP_DIR}/ivp_physics/ivp_real_object.hxx
    ${IVP_DIR}/ivp_physics/ivp_solver_core_reaction.cxx
    ${IVP_DIR}/ivp_physics/ivp_solver_core_reaction.hxx
    ${IVP_DIR}/ivp_physics/ivp_stat_manager_cback_con.cxx
    ${IVP_DIR}/ivp_physics/ivp_stat_manager_cback_con.hxx
    ${IVP_DIR}/ivp_physics/ivp_surface_manager.cxx
    ${IVP_DIR}/ivp_physics/ivp_templates.cxx
    ${IVP_DIR}/ivp_physics/ivp_templates.hxx
    ${IVP_DIR}/ivp_physics/ivp_time.cxx
    ${IVP_DIR}/ivp_physics/ivp_time.hxx
    ${IVP_DIR}/ivp_physics/ivp_time_event.hxx
    ${IVP_DIR}/ivp_physics/ivp_universe_manager.hxx
    ${IVP_DIR}/ivp_physics/ivp_coll_del_root_mindist.cxx
    ${IVP_DIR}/ivp_physics/ivp_clustering_visualizer.cxx
    ${IVP_DIR}/ivp_physics/ivp_clustering_visual_hash.cxx
    ${IVP_DIR}/ivp_physics/ivp_clustering_visual_hash.hxx
    ${IVP_DIR}/ivp_physics/ivp_clustering_visualizer.hxx
    ${IVP_DIR}/ivp_physics/ivp_controller.cxx
    ${IVP_DIR}/ivp_physics/ivp_controller.hxx
    ${IVP_DIR}/ivp_physics/ivp_oo_watcher.cxx
)
list(APPEND IVP_PHYSICS_SOURCES ${IVP_INTERN_SOURCES})
add_ivp_library(ivp_physics "${IVP_DIR}/ivp_physics" "ivp_utility;ivp_surface_manager" "${IVP_PHYSICS_SOURCES}")
target_include_directories(ivp_physics PRIVATE
    ${IVP_DIR}/ivp_intern
)

# ivp_collision - collision implementation, depends on physics API/runtime
set(IVP_COLLISION_SOURCES
    ${IVP_DIR}/ivp_collision/ivp_clustering_longrange.cxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_lrange_hash.cxx
    ${IVP_DIR}/ivp_collision/ivp_clustering_lrange_hash.hxx
    ${IVP_DIR}/ivp_collision/ivp_compact_ledge.cxx
    ${IVP_DIR}/ivp_collision/ivp_compact_ledge_solver.cxx
    ${IVP_DIR}/ivp_collision/ivp_i_collision_vhash.cxx
    ${IVP_DIR}/ivp_collision/ivp_ray_solver.cxx
    ${IVP_DIR}/ivp_collision/ivp_sphere_solver.cxx
)
# IMPORTANT: collision headers DO expose physics types (ivp_clustering_longrange.hxx
# -> ivp_real_object.hxx).  This PRIVATE link is only correct because the staged-headers
# architecture (ivp_public_headers target) provides all headers through each library's own
# PUBLIC include/ivp dir.  If staged headers are ever removed, this must become PUBLIC.
add_ivp_library(ivp_collision "${IVP_DIR}/ivp_collision" "" "${IVP_COLLISION_SOURCES}")
target_link_libraries(ivp_collision PRIVATE ivp_physics)

# ivp_controller - Physics controllers, depends on physics core
set(IVP_CONTROLLER_SOURCES
    ${IVP_DIR}/ivp_controller/ivp_actuator.cxx
    ${IVP_DIR}/ivp_controller/ivp_actuator.hxx
    ${IVP_DIR}/ivp_controller/ivp_actuator_info.hxx
    ${IVP_DIR}/ivp_controller/ivp_actuator_spring.cxx
    ${IVP_DIR}/ivp_controller/ivp_actuator_spring.hxx
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
    ${IVP_DIR}/ivp_controller/ivp_controller_factory.hxx
    ${IVP_DIR}/ivp_controller/ivp_environment_factories.cxx
    ${IVP_DIR}/ivp_controller/ivp_car_wheel_physics.cxx
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
add_ivp_library(ivp_controller "${IVP_DIR}/ivp_controller" "ivp_physics" "${IVP_CONTROLLER_SOURCES}")

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

# compact_builder's public headers only use ivu_* (utility) and ivp_compact_* (surface_manager)
# types.  ivp_physics and ivp_collision are only needed at link time — their types appear in
# compact_builder headers only as forward-declared pointers / friend classes.
add_ivp_library(ivp_compact_builder "${IVP_COMPACT_BUILDER_DIR}" "ivp_surface_manager" "${ALL_COMPACT_BUILDER_SOURCES}")
target_link_libraries(ivp_compact_builder PRIVATE ivp_physics ivp_collision)
if(IVP_INCLUDE_GEOMPACK)
    target_compile_definitions(ivp_compact_builder PRIVATE IVP_ENABLE_GEOMPACK)
endif()
if(IVP_INCLUDE_QHULL)
    target_compile_definitions(ivp_compact_builder PRIVATE IVP_ENABLE_QHULL)
endif()

# ============================================================================
# Havana Library Integration
# ============================================================================

# Check if Havana sources should be built
option(IVP_BUILD_HAVANA "Build Havana libraries alongside IVP" OFF)
if(IVP_IS_SUBPROJECT)
    mark_as_advanced(IVP_BUILD_HAVANA)
endif()

set(IVP_HAVANA_FUNCTIONAL FALSE)
set(IVP_HAVANA_HEADER_TARGETS FALSE)

# Check if Havana directory exists
set(HAVANA_SOURCES_FOUND FALSE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/havana")
    set(HAVANA_SOURCES_FOUND TRUE)
endif()

if(IVP_BUILD_HAVANA)
    # Add Havana subdirectory if it contains CMakeLists.txt
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/havana/CMakeLists.txt")
        add_subdirectory(havana)
        # Keep IVP self-contained; Havana builds against IVP instead of the reverse.
        target_compile_definitions(ivp_physics PRIVATE HAVANA_CONSTRAINTS)
        set(IVP_HAVANA_FUNCTIONAL TRUE)
    elseif(HAVANA_SOURCES_FOUND)
        message(WARNING
            "IVP_BUILD_HAVANA=ON was requested, but this checkout does not provide "
            "havana/CMakeLists.txt. Building IVP without Havana functionality and "
            "exporting header-only compatibility targets only."
        )

        if(NOT TARGET hk_base)
            add_library(hk_base INTERFACE)
            target_include_directories(hk_base INTERFACE
                ${CMAKE_CURRENT_SOURCE_DIR}/havana/havok
            )
        endif()
        if(NOT TARGET hk_math)
            add_library(hk_math INTERFACE)
            target_include_directories(hk_math INTERFACE
                ${CMAKE_CURRENT_SOURCE_DIR}/havana/havok
            )
        endif()
        if(NOT TARGET hk_physics)
            add_library(hk_physics INTERFACE)
            target_include_directories(hk_physics INTERFACE
                ${CMAKE_CURRENT_SOURCE_DIR}/havana/havok
            )
        endif()

        set(IVP_HAVANA_HEADER_TARGETS TRUE)
    else()
        message(WARNING
            "IVP_BUILD_HAVANA=ON was requested, but the havana sources are missing. "
            "Building IVP without Havana functionality."
        )
    endif()
endif()

# ============================================================================
# Installation and Export Configuration
# ============================================================================

set(IVP_PUBLIC_PACKAGE_TARGETS
    ivp_utility
    ivp_surface_manager
    ivp_physics
    ivp_collision
    ivp_controller
    ivp_compact_builder
)

set(IVP_PUBLIC_HEADER_DIRS
    ivp_collision
    ivp_controller
    ivp_intern
    ivp_physics
    ivp_surface_manager
    ivp_utility
    ivp_compact_builder
)

include(CMakePackageConfigHelpers)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/cmake")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include/ivp")

set(IVP_PUBLIC_HEADER_SOURCES)
foreach(_ivp_header_dir IN LISTS IVP_PUBLIC_HEADER_DIRS)
    file(GLOB _ivp_header_matches CONFIGURE_DEPENDS
        "${IVP_DIR}/${_ivp_header_dir}/*.h"
        "${IVP_DIR}/${_ivp_header_dir}/*.hxx"
    )
    list(APPEND IVP_PUBLIC_HEADER_SOURCES ${_ivp_header_matches})
endforeach()

set(IVP_PUBLIC_STAGED_HEADERS)
foreach(_ivp_header_src IN LISTS IVP_PUBLIC_HEADER_SOURCES)
    get_filename_component(_ivp_header_name "${_ivp_header_src}" NAME)
    set(_ivp_header_dst "${CMAKE_CURRENT_BINARY_DIR}/include/ivp/${_ivp_header_name}")
    add_custom_command(
        OUTPUT "${_ivp_header_dst}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/include/ivp"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_ivp_header_src}" "${_ivp_header_dst}"
        DEPENDS "${_ivp_header_src}"
        VERBATIM
    )
    list(APPEND IVP_PUBLIC_STAGED_HEADERS "${_ivp_header_dst}")
endforeach()

add_custom_target(ivp_public_headers ALL
    DEPENDS ${IVP_PUBLIC_STAGED_HEADERS}
)

foreach(_ivp_public_target IN LISTS IVP_PUBLIC_PACKAGE_TARGETS)
    add_dependencies(${_ivp_public_target} ivp_public_headers)
endforeach()

export(TARGETS ${IVP_PUBLIC_PACKAGE_TARGETS}
    FILE "${CMAKE_CURRENT_BINARY_DIR}/ivp_targets.cmake"
    NAMESPACE ivp::
)

# Build-tree config uses configure_file (absolute paths, not relocatable).
# Install-tree config uses configure_package_config_file (relocatable).
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ivp-config-build-tree.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/ivp-config.cmake"
    @ONLY
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/ivp-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Phase 1 package surface intentionally exports only the current engine
# libraries. Example-side renderer ownership is classified in the repository
# split work and should not be implicitly exported here.

# Option to control installation (disabled by default when used as subproject)
if(IVP_IS_SUBPROJECT)
    option(IVP_INSTALL "Generate installation target" OFF)
else()
    option(IVP_INSTALL "Generate installation target" ON)
endif()

if(IVP_INSTALL)
    # Include GNUInstallDirs to use standard directories
    include(GNUInstallDirs)

    # Install libraries.
    # Note: INCLUDES DESTINATION is omitted — include paths are already set
    # via $<INSTALL_INTERFACE:...> in target_include_directories().
    install(TARGETS ${IVP_PUBLIC_PACKAGE_TARGETS}
        EXPORT ivp_targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    if(IVP_HAVANA_FUNCTIONAL)
        install(TARGETS hk_base hk_math hk_physics
            EXPORT ivp_targets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    endif()

    # Install header files (same directories used for build-tree staging)
    set(_ivp_install_header_dirs)
    foreach(_ivp_hdir IN LISTS IVP_PUBLIC_HEADER_DIRS)
        list(APPEND _ivp_install_header_dirs "${IVP_DIR}/${_ivp_hdir}/")
    endforeach()
    install(DIRECTORY ${_ivp_install_header_dirs}
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

    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ivp-config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/ivp-config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ivp
        NO_SET_AND_CHECK_MACRO
    )

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/ivp-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake/ivp-config-version.cmake"
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
        message(STATUS "    Havana functionality enabled: ${IVP_HAVANA_FUNCTIONAL}")
        message(STATUS "    Havana header-only compatibility targets: ${IVP_HAVANA_HEADER_TARGETS}")
    endif()
    message(STATUS "")
else()
    message(STATUS "IVP: Configured as subproject (installation disabled by default)")
endif()
