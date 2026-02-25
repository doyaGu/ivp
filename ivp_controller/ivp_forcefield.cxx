// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#ifndef WIN32
#pragma implementation "ivp_forcefield.hxx"
#endif

#include <ivp_forcefield.hxx>

void IVP_Forcefield::element_added(IVP_U_Set_Active<IVP_Core> *, IVP_Core *elem)
{
    if (!elem)
    {
        return;
    }
    attached_cores.install(elem);
    elem->environment->get_controller_manager()->add_controller_to_core(this, elem);
}

void IVP_Forcefield::element_removed(IVP_U_Set_Active<IVP_Core> *, IVP_Core *elem)
{
    if (!elem)
    {
        return;
    }
    elem->environment->get_controller_manager()->remove_controller_from_core(this, elem);
    int idx = attached_cores.index_of(elem);
    if (idx >= 0)
    {
        attached_cores.remove_at(idx);
    }
}

IVP_Forcefield::IVP_Forcefield(IVP_Environment *, IVP_U_Set_Active<IVP_Core> *set_of_cores_in, IVP_BOOL owner_of_set)
{
    set_of_cores = set_of_cores_in;
    this->i_am_owner_of_set_of_cores = owner_of_set;
    set_is_going_to_be_deleted = IVP_FALSE;
    IVP_U_Set_Enumerator<IVP_Core> all_objects(set_of_cores);
    while (IVP_Core *my_core = all_objects.get_next_element(set_of_cores))
    {
        this->element_added(set_of_cores, my_core);
    }
    set_of_cores->add_listener_set_active(this);
}

void IVP_Forcefield::pset_is_going_to_be_deleted(IVP_U_Set_Active<IVP_Core> *)
{ // Note: pset is not removing elements when deleted
    set_is_going_to_be_deleted = IVP_TRUE;
    set_of_cores = NULL;
    P_DELETE_THIS(this);
}

void IVP_Forcefield::core_is_going_to_be_deleted_event(IVP_Core *core)
{
    if (!core)
    {
        return;
    }
    if (!set_is_going_to_be_deleted && set_of_cores && set_of_cores->find_element(core))
    {
        set_of_cores->remove_element(core);
        return;
    }
    this->element_removed(NULL, core);
}

IVP_CONTROLLER_PRIORITY IVP_Forcefield::get_controller_priority()
{
    // default implementation
    return IVP_CP_ACTUATOR;
}

IVP_Forcefield::~IVP_Forcefield()
{
    for (int i = attached_cores.len() - 1; i >= 0; --i)
    {
        IVP_Core *my_core = attached_cores.element_at(i);
        if (my_core)
        {
            my_core->environment->get_controller_manager()->remove_controller_from_core(this, my_core);
        }
    }
    attached_cores.remove_all();
    if (!set_is_going_to_be_deleted && this->set_of_cores)
    {
        this->set_of_cores->remove_listener_set_active(this);
    }
}
