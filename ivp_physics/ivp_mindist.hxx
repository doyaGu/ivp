// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

#ifndef IVP_COLLISION_MINDIST_INCLUDED
#define IVP_COLLISION_MINDIST_INCLUDED

class IVP_Mindist_Base;
class IVP_Compact_Edge;

#include <ivu_fvector.hxx>
#include <ivp_time_event.hxx>
#include <ivp_listener_hull.hxx>
#include <ivp_collision.hxx>

// #define IVP_MINDIST_BEHAVIOUR_DEBUG	// do core as soon as possible

/* if IVP_HALFSPACE_OPTIMIZATION_ENABLED is defined, the mindist remembers a virtual plane
 * seperating both ledges. It tries to use that halfspace to optimize the hull */
#define IVP_HALFSPACE_OPTIMIZATION_ENABLED
// #define IVP_PHANTOM_FULL_COLLISION //@@CB - not default behaviour

/********************************************************************************
 *	Name:	       	IVP_SYNAPSE_POLYGON_STATUS
 *	Description:	How the synapse is connected to a geometry
 ********************************************************************************/
enum IVP_SYNAPSE_POLYGON_STATUS
{
    IVP_ST_POINT = 0,
    IVP_ST_EDGE = 1,
    IVP_ST_TRIANGLE = 2,
    IVP_ST_BALL = 3,
    IVP_ST_MAX_LEGAL = 4, // max legal status, should be 2**x
    IVP_ST_BACKSIDE = 5   // unknown, intrusion
};

/********************************************************************************
 *	Name:	       	IVP_Synapse
 *	Description:	Synapses are attachments to objects
 ********************************************************************************/
class IVP_Synapse : public IVP_Listener_Hull
{ // sizeof() == 32
public:
    IVP_Synapse *next, *prev;     // per object, only for exact/invalid synapses
    IVP_Real_Object *l_obj;       // back link to object
    const IVP_Compact_Edge *edge; // Note: all balls share one dummy edge

protected:
    short mindist_offset; // back link to my controlling mindist
    short status;         // IVP_SYNAPSE_POLYGON_STATUS point, edge, tri, ball ....
public:
protected:
    // hull manager
    IVP_HULL_ELEM_TYPE get_type()
    {
        return IVP_HULL_ELEM_POLYGON;
    }
    virtual void hull_limit_exceeded_event(IVP_Hull_Manager *hull_manager, IVP_HTIME hull_intrusion_value);
    virtual void hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *hull_manager);
    virtual void hull_manager_is_reset(IVP_FLOAT dt, IVP_FLOAT center_dt);

public:
    IVP_Real_Object *get_object()
    {
        return l_obj;
    }
    IVP_SYNAPSE_POLYGON_STATUS get_status() const
    {
        return (IVP_SYNAPSE_POLYGON_STATUS)status;
    }

    IVP_Synapse() : next(NULL), prev(NULL), l_obj(NULL), edge(NULL), mindist_offset(SHRT_MAX), status(SHRT_MAX) {}
    virtual ~IVP_Synapse() {} // dummy, do not call
    const IVP_Compact_Ledge *get_ledge() const;
    const IVP_Compact_Edge *get_edge() const
    {
        return edge;
    }

    IVP_Mindist_Base *get_synapse_mindist() const
    {
        return (IVP_Mindist_Base *)(mindist_offset + (char *)this);
    }
    void set_synapse_mindist(IVP_Mindist_Base *md)
    {
        mindist_offset = ((char *)md) - (char *)this;
    }

    void init_synapse_real(IVP_Mindist_Base *min, IVP_Real_Object *object_to_link)
    {
        set_synapse_mindist(min);
        l_obj = object_to_link;
        IVP_IF(1)
        {
            next = prev = this;
        }
    }
};

#if defined(DEBUG) && defined(IVP_MINDIST_BEHAVIOUR_DEBUG)
    #define DEBUG_CHECK_LEN
#endif

#define IVP_MAX_MINIMIZE_BEFORE_HASH_CHECK 20
#define IVP_MAX_PIERCINGS 2
#define IVP_LOOP_HASH_SIZE 20
#define IVP_MAX_PIERCE_CNT 0xFFFE

#define IVP_MIN_TERMINATION_QDLEN 1E-3f     // generates core
#define IVP_MIN_TERMINATION_QDLEN_EPS 1E-6f // generates warning

class IVP_Environment;
class IVP_Collision;

class IVP_Synapse_Real;
class IVP_Contact_Point;
class IVP_Actuator;
class IVP_Compact_Ledge;
class IVP_Mindist_OO_Watcher;
class IVP_Mindist;
class IVP_Friction_System;
class IVP_Mindist_Manager;
class IVP_Simulation_Unit;

#define IVP_MAX_STEPS_FOR_COLLDIST_DECREASE 64

extern class IVP_Mindist_Settings
{
  public:
    // mindist
    IVP_FLOAT real_coll_dist;                                  // very absolute bound of distance
    IVP_FLOAT min_coll_dists;                                  // the minimum of all call dists
    IVP_FLOAT coll_dists[IVP_MAX_STEPS_FOR_COLLDIST_DECREASE]; // Minimal distance between objects before collision occurs [meter]

    // friction
    IVP_FLOAT minimum_friction_dist;          // for friction checks for impact
    IVP_FLOAT friction_dist;                  // distance for friction
    IVP_FLOAT keeper_dist;                    // try to hold objects with a simple spring
    IVP_FLOAT speed_after_keeper_dist;        // speed when falling from keeper_dist to coll_dist
    IVP_FLOAT distance_keepers_safety;        // safety gap, when surpassed mindist doesnt appear in complex
    IVP_FLOAT max_distance_for_friction;      // when to through away friction mindist
    IVP_FLOAT max_distance_for_impact_system; // check friction mindinst for impact when in this range

    // collision
    IVP_FLOAT mindist_change_force_dist;       // in [meter]
    IVP_FLOAT min_vertical_speed_at_collision; // collision is assumed when distance is < coll_dist + mindist_change_force_dist

    // spawned mindists for recursive hulls @@CB
    int max_spawned_mindist_count;

    // base size for event queue delta time stepping @@CB
    IVP_DOUBLE event_queue_min_delta_time_base;

    void set_collision_tolerance(IVP_DOUBLE);
    void set_event_queue_min_delta_time_base(IVP_DOUBLE); //@@CB
    IVP_Mindist_Settings();
} ivp_mindist_settings;

/********************************************************************************
 *	Name:	  	IVP_Collision_Delegator_Root_Sphere
 *	Description:	Simple root collision delegator which generates functions
 *			useable for convex subparts
 ********************************************************************************/
class IVP_Collision_Delegator_Root_Mindist : public IVP_Collision_Delegator_Root
{
  public:
    virtual void object_is_removed_from_collision_detection(IVP_Real_Object *);
    virtual IVP_Collision *delegate_collisions_for_object(IVP_Real_Object *base_object, IVP_Real_Object *colliding_element);

    virtual void environment_is_going_to_be_deleted_event(IVP_Environment *env);
    virtual void collision_is_going_to_be_deleted_event(class IVP_Collision *t);
    ~IVP_Collision_Delegator_Root_Mindist();
    IVP_Collision_Delegator_Root_Mindist();
};

class IVP_OO_Watcher;
/********************************************************************************
 *	Name:	  	IVP_Synapse_OO
 *	Description:	Synapses for the IVP_OO_Watcher
 ********************************************************************************/
class IVP_Synapse_OO : public IVP_Listener_Hull
{
    friend class IVP_OO_Watcher;
    IVP_Real_Object *object;
    IVP_OO_Watcher *watcher;
    virtual ~IVP_Synapse_OO();
    IVP_Synapse_OO() : object(NULL), watcher(NULL) {}
    void init_synapse_oo(IVP_OO_Watcher *, IVP_Real_Object *);

  public:
    IVP_HULL_ELEM_TYPE get_type()
    {
        return IVP_HULL_ELEM_OO_WATCHER;
    }
    void hull_limit_exceeded_event(IVP_Hull_Manager *hull_manager, IVP_HTIME);
    void hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *); // function have to remove itself from hull manager
};

/********************************************************************************
 *	Name:	  	IVP_OO_Watcher
 *	Description:	Manages a hash of IVP_Mindist_OO_Watcher
 ********************************************************************************/
class IVP_OO_Watcher : public IVP_Collision, public IVP_Collision_Delegator
{
    IVP_Synapse_OO synapses[2];
    IVP_U_FVector<IVP_Collision> mindists;

  protected:
    void get_objects(IVP_Real_Object *objects_out[2]);
    void get_ledges(const IVP_Compact_Ledge *ledges_out[2]);

    IVP_Synapse_OO *get_synapse(int i)
    {
        return &synapses[i];
    }

  public:
    // IVP_Collision_Delegator
    void collision_is_going_to_be_deleted_event(class IVP_Collision *t);

    void hull_limit_exceeded_event();
    void hull_manager_is_going_to_be_deleted_event();
    virtual ~IVP_OO_Watcher();
    IVP_OO_Watcher(IVP_Collision_Delegator *del, IVP_Real_Object *obj0, IVP_Real_Object *obj1);
    virtual void simulate_time_event(IVP_Environment *)
    {
        CORE;
    }
};

/********************************************************************************
 *	Name:	  	IVP_Synapse_Real
 *	Description:	Adds additional functionality to IVP_Synapse,
 *	Attention:	No additional storage elements allowed in IVP_Synapse_Real
 ********************************************************************************/
class IVP_Synapse_Real : public IVP_Synapse
{
  protected:
    friend class IVP_Mindist;
    friend class IVP_Mindist_Base;
    friend class IVP_Mindist_Manager;
    friend class IVP_Contact_Point;

  protected:
    void check_consistency_of_ledge(const IVP_Compact_Edge *edge) const;

    inline void remove_exact_synapse_from_object();
    inline void insert_exact_synapse_in_object();

    inline void remove_invalid_synapse_from_object();
    inline void insert_invalid_synapse_in_object();

    virtual ~IVP_Synapse_Real() {}

  public:
    inline void update_synapse(const IVP_Compact_Edge *e, IVP_SYNAPSE_POLYGON_STATUS s)
    {
        IVP_IF(s != IVP_ST_BALL)
        {
            check_consistency_of_ledge(e);
        }
        edge = e;
        status = s;
    }

  protected:
    IVP_Synapse_Real() {}
    void init_synapse(IVP_Mindist *min, IVP_Real_Object *object_to_link, const IVP_Compact_Edge *e, IVP_SYNAPSE_POLYGON_STATUS s)
    {
        IVP_Synapse::init_synapse_real((IVP_Mindist_Base *)min, object_to_link);
        update_synapse(e, s);
    }

  public:
    inline IVP_Core *get_core() const
    {
        return l_obj->physical_core;
    }

    virtual void print();

    IVP_Hull_Manager *get_hull_manager()
    {
        return get_object()->get_hull_manager();
    }
    inline IVP_DOUBLE insert_in_hull_manager(IVP_DOUBLE rel_hull_time); // returns current center pos
    inline IVP_DOUBLE insert_lazy_in_hull_manager(IVP_DOUBLE rel_hull_time);

    IVP_Synapse_Real *get_next()
    {
        return (IVP_Synapse_Real *)next;
    }
    IVP_Synapse_Real *get_prev()
    {
        return (IVP_Synapse_Real *)prev;
    }
    IVP_Mindist *get_mindist()
    {
        return (IVP_Mindist *)get_synapse_mindist();
    }
};

enum IVP_MINIMAL_DIST_STATUS
{
    IVP_MD_UNINITIALIZED = 0,
    IVP_MD_INVALID = 2,
    IVP_MD_EXACT = 3,
    IVP_MD_HULL_RECURSIVE = 4,
    IVP_MD_HULL = 5
};

enum IVP_MINIMAL_DIST_RECALC_RESULT
{
    IVP_MDRR_OK = 0,
    IVP_MDRR_INTRUSION = 1
};

enum IVP_COLL_TYPE
{
    IVP_COLL_NONE = 0x00,
    IVP_COLL_PP_COLL = 0x10,
    IVP_COLL_PP_PK = 0x11,

    IVP_COLL_PF_COLL = 0x20,
    IVP_COLL_PF_NPF = 0x21,

    IVP_COLL_PK_COLL = 0x30,
    IVP_COLL_PK_PF = 0x31,
    IVP_COLL_PK_KK = 0x32,
    IVP_COLL_PK_NOT_MORE_PARALLEL = 0x33,

    IVP_COLL_KK_COLL = 0x40,
    IVP_COLL_KK_PARALLEL = 0x41,
    IVP_COLL_KK_PF = 0x42
};

enum IVP_MINDIST_FUNCTION
{
    IVP_MF_COLLISION = 0,
    IVP_MF_PHANTOM = 1
};

class IVP_Mindist_Base : public IVP_Collision
{
  protected:
    IVP_COLL_TYPE coll_type : 8;
    unsigned int synapse_sort_flag : 2;

  public:
    IVP_BOOL is_in_phantom_set : 2;
    IVP_MINDIST_FUNCTION mindist_function : 2;
    IVP_MINIMAL_DIST_RECALC_RESULT recalc_result : 2;
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED
    IVP_BOOL disable_halfspace_optimization : 2;
#endif
    IVP_MINIMAL_DIST_STATUS mindist_status : 4;
    unsigned int coll_dist_selector : 8;
    unsigned int coll_dist_decrease_counter : 8;

    IVP_Synapse_Real synapse[2];
    IVP_FLOAT sum_extra_radius;

    IVP_FLOAT len_numerator;
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED
    IVP_FLOAT contact_dot_diff_center;
    IVP_DOUBLE sum_angular_hull_time;
#endif

    IVP_U_Float_Point contact_plane;

    IVP_Synapse *get_mindist_synapse(int i)
    {
        return &synapse[i];
    }
    const IVP_Synapse *get_mindist_synapse(int i) const
    {
        return &synapse[i];
    }

    IVP_FLOAT get_length() const
    {
        return len_numerator;
    }

    virtual void get_objects(IVP_Real_Object *objects_out[2]);
    virtual void get_ledges(const IVP_Compact_Ledge *ledges_out[2]);

    IVP_Mindist_Base(IVP_Collision_Delegator *del);
    virtual ~IVP_Mindist_Base() {}
};

enum IVP_MRC_TYPE
{
    IVP_MRC_UNINITIALIZED = 0,
    IVP_MRC_OK = 1,
    IVP_MRC_ENDLESS_LOOP = 2,
    IVP_MRC_BACKSIDE = 3,
    IVP_MRC_ALREADY_CALCULATED = 4,
    IVP_MRC_ILLEGAL = 5
};

enum IVP_MINDIST_EVENT_HINT
{
    IVP_EH_NOW,
    IVP_EH_SMALL_DELAY,
    IVP_EH_BIG_DELAY
};

class IVP_Mindist : public IVP_Mindist_Base
{
    friend class IVP_Mindist_Manager;
    friend class IVP_Contact_Point;
    friend class IVP_Mindist_Event_Solver;
    friend class IVP_Mindist_Minimize_Solver;
    friend class IVP_Synapse;
    friend class IVP_Synapse_Real;

  protected:
    IVP_Time_CODE recalc_time_stamp;

  public:
    IVP_Mindist *next;
    IVP_Mindist *prev;
    const IVP_Compact_Edge *last_visited_triangle;

  protected:
    virtual void mindist_rescue_push();

    void mindist_hull_limit_exceeded_event(IVP_HTIME hull_intrusion_value);
    void hull_manager_is_reset(IVP_FLOAT dt, IVP_FLOAT center_dt);

  public:
    IVP_Synapse_Real *get_synapse(int i) const
    {
        return (IVP_Synapse_Real *)&synapse[i];
    }
    IVP_Synapse_Real *get_sorted_synapse(int i) const
    {
        return (IVP_Synapse_Real *)&synapse[synapse_sort_flag ^ i];
    }
    IVP_DOUBLE get_coll_dist()
    {
        return ivp_mindist_settings.coll_dists[coll_dist_selector];
    }

    IVP_Mindist(IVP_Environment *env, IVP_Collision_Delegator *del);
    virtual ~IVP_Mindist();

    virtual IVP_BOOL is_recursive()
    {
        return IVP_FALSE;
    }

    IVP_Environment *get_environment()
    {
        return get_synapse(0)->get_object()->get_environment();
    }

    void init_mindist(IVP_Real_Object *pop0, IVP_Real_Object *pop1, const IVP_Compact_Edge *e0, const IVP_Compact_Edge *e1);

    void print(const char *text);

    void simulate_time_event(IVP_Environment *env);

    IVP_MRC_TYPE recalc_mindist();
    IVP_MRC_TYPE recalc_invalid_mindist();

    virtual void exact_mindist_went_invalid(IVP_Mindist_Manager *mm);

    void update_exact_mindist_events(IVP_BOOL allow_hull_conversion, IVP_MINDIST_EVENT_HINT allow_events_at_now);

    virtual void do_impact();

    IVP_Contact_Point *try_to_generate_managed_friction(IVP_Friction_System **associated_fs, IVP_BOOL *having_new, IVP_Simulation_Unit *sim_unit_not_destroy, IVP_BOOL call_recalc_svals);

    void create_cp_in_advance_pretension(IVP_Real_Object *robject, float gap_len);
};

enum IVP_MINDIST_RECURSIVE_TYPES
{
    IVP_MR_NORMAL = -1,
    IVP_MR_FIRST_SYNAPSE_RECURSIVE = 0,
    IVP_MR_SECOND_SYNAPSE_RECURSIVE = 1
};

class IVP_Mindist_Recursive : public IVP_Mindist, public IVP_Collision_Delegator
{
    void delete_all_children();
    virtual void collision_is_going_to_be_deleted_event(class IVP_Collision *t);
    void recheck_recursive_childs(IVP_DOUBLE hull_dist_intra_object);
    void invalid_mindist_went_exact();

  public:
    virtual void mindist_rescue_push();
    void rec_hull_limit_exceeded_event();
    virtual void exact_mindist_went_invalid(IVP_Mindist_Manager *mm);

    IVP_MINDIST_RECURSIVE_TYPES recursive_status;
    IVP_U_FVector<IVP_Collision> mindists;
    int spawned_mindist_count;

    void change_spawned_mindist_count(int change);
    int get_spawned_mindist_count();

    virtual IVP_BOOL is_recursive()
    {
        return IVP_TRUE;
    }

    virtual void do_impact();

    IVP_Mindist_Recursive(IVP_Environment *env, IVP_Collision_Delegator *del);
    ~IVP_Mindist_Recursive();
};

/********************************************************************************
 *	Name:	    	IVP_Mindist_Manager
 *	Description:	manages all exact mindists, hull_mindists are handled by
 *			the IVP_Hull _Manager
 ********************************************************************************/
class IVP_Mindist_Manager
{
    friend class IVP_Real_Object;
    IVP_BOOL scanning_universe;

  public:
    IVP_Environment *environment;

    IVP_Mindist *exact_mindists;
    IVP_U_Vector<IVP_Mindist> wheel_look_ahead_mindists;

    IVP_Mindist *invalid_mindists;

    static void create_exact_mindists(IVP_Real_Object *obj0, IVP_Real_Object *obj1, IVP_DOUBLE scan_radius, IVP_U_FVector<IVP_Collision> *existing_collisions, const IVP_Compact_Ledge *single_ledge0, const IVP_Compact_Ledge *single_ledge1, const IVP_Compact_Ledge *root_ledge0, const IVP_Compact_Ledge *root_ledge1, IVP_Collision_Delegator *del);

    void insert_exact_mindist(IVP_Mindist *new_mindist);
    void insert_and_recalc_exact_mindist(IVP_Mindist *new_mindist);
    void insert_and_recalc_phantom_mindist(IVP_Mindist *new_mindist);
    void remove_exact_mindist(IVP_Mindist *del_mindist);

    void insert_invalid_mindist(IVP_Mindist *new_mindist);
    void remove_invalid_mindist(IVP_Mindist *del_mindist);

    static void mindist_entered_phantom(IVP_Mindist *mdist);
    static void mindist_left_phantom(IVP_Mindist *mdist);

    void remove_hull_mindist(IVP_Mindist *del_mindist);
    static void insert_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time0, IVP_HTIME hull_time1);
    static void insert_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time);

    static void insert_lazy_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time0, IVP_HTIME hull_time1);
    static void insert_lazy_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time);

    void recheck_ov_element(IVP_Real_Object *object);

    void recalc_exact_mindist(IVP_Mindist *mdist);
    void recalc_all_exact_mindists();
    void recalc_all_exact_wheel_mindist();

    void recalc_all_exact_mindists_events();

    void enable_collision_detection_for_object(IVP_Real_Object *);

    void print_mindists();

    IVP_Mindist_Manager(IVP_Environment *i_env);
    ~IVP_Mindist_Manager();
};

#endif // IVP_COLLISION_MINDIST_INCLUDED
