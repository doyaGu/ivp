// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivu_memory.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>

#include <ivp_physic_private.hxx>
#include <ivp_sim_unit.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_material.hxx>
#include <ivp_impact.hxx>
#include <ivp_mindist.hxx>
#include <ivp_friction.hxx>
#include <ivp_hull_manager.hxx>
#include <ivp_mindist_macros.hxx>
#include <ivp_listener_collision.hxx>
#include <ivp_calc_next_psi_solver.hxx>
#include <ivp_debug_manager.hxx>
#include <ivp_anomaly_manager.hxx>

extern IVP_Mindist *g_pCurrentMindist;
extern bool g_fDeferDeleteMindist;

// generates friction mindist from mindist when no friction mindist exists and
// makes sure that friction mindist (generated or not) is up to date.
// sim_unit_not_destroy is the one that has to remain after fusion.
IVP_Contact_Point *IVP_Mindist::try_to_generate_managed_friction(
    IVP_Friction_System **associated_fs,
    IVP_BOOL *having_new,
    IVP_Simulation_Unit *sim_unit_not_destroy,
    IVP_BOOL call_recalc_svals)
{
    IVP_Mindist *my_dist = this;
    IVP_Real_Object *obj0 = get_sorted_synapse(0)->l_obj;
    IVP_Real_Object *obj1 = get_sorted_synapse(1)->l_obj;

    IVP_IF(obj0->get_environment()->debug_information->debug_friction)
    {
        printf("new_fri_mindist\n");
    }

    IVP_Friction_System *fr_sys0, *fr_sys1;
    IVP_Friction_Info_For_Core *fr_info0, *fr_info1;

    IVP_Core *core0 = obj0->friction_core;
    IVP_Core *core1 = obj1->friction_core;

    if (core0->physical_unmoveable)
    {
        IVP_Core *temp_core;
        temp_core = core0;
        core0 = core1;
        core1 = temp_core;
    }

    fr_info0 = core0->moveable_core_has_friction_info();

    IVP_Environment *my_env = core0->environment;
    IVP_Contact_Point *friction_dist;
    IVP_BOOL gen_success;
    *associated_fs = NULL;
    *having_new = IVP_FALSE;

    friction_dist = IVP_Friction_Manager::generate_contact_point(my_dist, &gen_success);
    if (friction_dist == NULL)
    {
        if (fr_info0)
        {
            *associated_fs = fr_info0->l_friction_system;
        }
        return NULL;
    }
    {
        if (gen_success != IVP_TRUE)
        {
            *associated_fs = friction_dist->l_friction_system;
            if ((*associated_fs == NULL) && fr_info0)
            {
                *associated_fs = fr_info0->l_friction_system;
            }
            *having_new = IVP_FALSE;
            if (call_recalc_svals)
            {
                friction_dist->recalc_friction_s_vals();
                friction_dist->read_materials_for_contact_situation(friction_dist->tmp_contact_info);
            }
            return friction_dist;
        }
    }
    if (call_recalc_svals)
    {
        friction_dist->recalc_friction_s_vals();
        friction_dist->read_materials_for_contact_situation(friction_dist->tmp_contact_info);
    }

    {
        IVP_Event_Friction event_friction;
        IVP_Environment *env = obj0->get_environment();
        event_friction.environment = env;
        event_friction.contact_situation = friction_dist->tmp_contact_info;
        event_friction.friction_handle = friction_dist;
        env->fire_event_friction_created(&event_friction);
        if (obj0->flags.collision_listener_exists)
        {
            IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
            clus_man->fire_event_friction_created(obj0, &event_friction);
        }
        if (obj1->flags.collision_listener_exists)
        {
            IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
            clus_man->fire_event_friction_created(obj1, &event_friction);
        }
    }

    *having_new = IVP_TRUE;

    IVP_Friction_System *affected_fs;

    if (fr_info0)
    {
        fr_sys0 = fr_info0->l_friction_system;
        fr_info1 = core1->get_friction_info(fr_sys0);
        if (fr_info1)
        {
            fr_sys0->add_dist_to_system(friction_dist);
            fr_sys0->dist_added_update_pair_info(friction_dist);
            affected_fs = fr_sys0;
        }
        else
        {
            if (core1->physical_unmoveable)
            {
                goto add_second_to_first;
            }
            fr_info1 = core1->moveable_core_has_friction_info();
            if (fr_info1)
            {
                fr_sys0->fusion_friction_systems(fr_info1->l_friction_system);
                goto add_second_to_first_dist_only;
            }
        add_second_to_first:
            fr_info1 = new IVP_Friction_Info_For_Core();
            fr_info1->l_friction_system = fr_sys0;
            core1->add_friction_info(fr_info1);
            fr_sys0->add_core_to_system(core1);

        add_second_to_first_dist_only:
            fr_sys0->add_dist_to_system(friction_dist);
            fr_sys0->dist_added_update_pair_info(friction_dist);
            affected_fs = fr_sys0;
        }
    }
    else
    {
        fr_info0 = new IVP_Friction_Info_For_Core();

        if (core1->physical_unmoveable)
        {
            goto new_system_for_both_objects;
        }
        fr_info1 = core1->moveable_core_has_friction_info();
        if (fr_info1)
        {
            fr_sys1 = fr_info1->l_friction_system;
            fr_info0->l_friction_system = fr_sys1;
            core0->add_friction_info(fr_info0);
            fr_sys1->add_core_to_system(core0);
            fr_sys1->add_dist_to_system(friction_dist);
            fr_sys1->dist_added_update_pair_info(friction_dist);
            affected_fs = fr_sys1;
        }
        else
        {
        new_system_for_both_objects:
            IVP_Friction_System *new_system;
            new_system = new IVP_Friction_System(my_env);
            fr_info1 = new IVP_Friction_Info_For_Core();
            fr_info0->l_friction_system = new_system;
            fr_info1->l_friction_system = new_system;
            core1->add_friction_info(fr_info1);
            core0->add_friction_info(fr_info0);
            new_system->add_dist_to_system(friction_dist);
            new_system->dist_added_update_pair_info(friction_dist);
            new_system->add_core_to_system(core0);
            new_system->add_core_to_system(core1);
            affected_fs = new_system;
        }
    }

    fr_info0->friction_info_insert_friction_dist(friction_dist);
    fr_info1->friction_info_insert_friction_dist(friction_dist);
#ifdef EASEONIMPACT
    fr_info0->set_all_dists_of_obj_neutral();
    fr_info1->set_all_dists_of_obj_neutral();
#endif

    *associated_fs = affected_fs;
#ifdef DEBUG
    IVP_IF(0)
    {
        IVP_Friction_System *fs0 = NULL;
        IVP_Friction_System *fs1 = NULL;
        if (!core0->physical_unmoveable)
        {
            IVP_Friction_Info_For_Core *info_friction = core0->moveable_core_has_friction_info();
            if (info_friction)
            {
                fs0 = info_friction->l_friction_system;
            }
        }
        if (!core1->physical_unmoveable)
        {
            IVP_Friction_Info_For_Core *info_for_core = core1->moveable_core_has_friction_info();
            if (info_for_core)
            {
                fs1 = info_for_core->l_friction_system;
            }
        }
        printf("\nmaking_new_frdist for cores %p %d  %p %d  fss %p %p  with sim_units: %p %p\n",
               core0, core0->physical_unmoveable,
               core1, core1->physical_unmoveable,
               fs0, fs1, core0->sim_unit_of_core, core1->sim_unit_of_core);
        core0->sim_unit_of_core->sim_unit_debug_out();
        core1->sim_unit_of_core->sim_unit_debug_out();
    }
#endif
    friction_dist->calc_virtual_mass_of_mindist();

    IVP_IF(obj0->get_environment()->get_debug_manager()->check_fs)
    {
        affected_fs->test_hole_fr_system_data();
    }

    IVP_Simulation_Unit *sim0 = core0->sim_unit_of_core;
    IVP_Simulation_Unit *sim1 = core1->sim_unit_of_core;

    if (!(core1->physical_unmoveable | core0->physical_unmoveable))
    {
        if (sim0 != sim1)
        {
            if (sim1 == sim_unit_not_destroy)
            {
                sim1->fusion_simulation_unities(sim0);
                P_DELETE(sim0);
            }
            else
            {
                sim0->fusion_simulation_unities(sim1);
                P_DELETE(sim1);
            }
        }
    }
#ifdef DEBUG
    IVP_IF(1)
    {
        if (!core0->physical_unmoveable)
        {
            core0->sim_unit_of_core->sim_unit_debug_consistency();
        }
        if (!core1->physical_unmoveable)
        {
            core1->sim_unit_of_core->sim_unit_debug_consistency();
        }
    }
#endif
    return friction_dist;
}

// assert: recalc_mindist called
// assert: mindist of EXACT
// actions perform the impact of two real (!) objects with all side effects
void IVP_Mindist::do_impact()
{
    IVP_Mindist *previous_mindist = g_pCurrentMindist;
    bool previous_defer_delete = g_fDeferDeleteMindist;
    g_fDeferDeleteMindist = false;
    g_pCurrentMindist = this;
    IVP_Environment *env = get_environment();

    IVP_ASSERT(mindist_status == IVP_MD_EXACT);

    IVP_Real_Object *objects[2];
    for (int i = 0; i < 2; i++)
    {
        IVP_Synapse_Real *syn = this->get_synapse(i);
        IVP_Real_Object *obj = syn->l_obj;
        objects[i] = obj;
        obj->revive_object_for_simulation();
    }

    if (g_fDeferDeleteMindist)
    {
        IVP_ASSERT(0);
        g_pCurrentMindist = previous_mindist;
        g_fDeferDeleteMindist = previous_defer_delete;
        delete this;
        return;
    }

    env->sim_unit_mem->start_memory_transaction();

    for (int j = 0; j < 2; j++)
    {
        IVP_Core *core = objects[j]->get_core();
        if (IVP_MTIS_SIMULATED(core->movement_state) && !core->pinned)
        {
            core->synchronize_with_rot_z();
        }
    }

    env->mindist_event_timestamp_reference++;

    IVP_Impact_Solver_Long_Term::do_impact_of_two_objects(this, objects[0], objects[1]);
    env->sim_unit_mem->end_memory_transaction();
    bool delete_current = g_fDeferDeleteMindist;
    g_pCurrentMindist = previous_mindist;
    g_fDeferDeleteMindist = previous_defer_delete;
    if (delete_current)
    {
        delete this;
    }
}
