// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef IVP_COLLISION_I_COLLISION_VHASH_INCLUDED
#define IVP_COLLISION_I_COLLISION_VHASH_INCLUDED

#include "ivu_vector.hxx"
#include "ivu_vhash.hxx"
#include "ivp_listener_collision.hxx"

class IVP_Collision_Callback_Table
{
public:
    IVP_Real_Object *real_object;
    IVP_U_Vector<IVP_Listener_Collision> listeners;

    IVP_Collision_Callback_Table() : real_object(NULL) {}
    ~IVP_Collision_Callback_Table();
};

class IVP_Collision_Callback_Table_Hash : protected IVP_VHash
{
protected:
    IVP_BOOL compare(void *elem0, void *elem1) const;
    int object_to_index(IVP_Real_Object *real_object);

public:
    void add_table(IVP_Collision_Callback_Table *table)
    {
        add_elem(table, object_to_index(table->real_object));
    };

    IVP_Collision_Callback_Table *remove_table(IVP_Real_Object *real_object)
    {
        IVP_Collision_Callback_Table table;
        table.real_object = real_object;
        return (IVP_Collision_Callback_Table *)remove_elem(&table, object_to_index(real_object));
    };

    IVP_Collision_Callback_Table *find_table(IVP_Real_Object *real_object)
    {
        IVP_Collision_Callback_Table table;
        table.real_object = real_object;
        return (IVP_Collision_Callback_Table *)find_elem(&table, object_to_index(real_object));
    };

    ~IVP_Collision_Callback_Table_Hash();
    IVP_Collision_Callback_Table_Hash(int init_size) : IVP_VHash(init_size) {}
};

#endif // IVP_COLLISION_I_COLLISION_VHASH_INCLUDED
