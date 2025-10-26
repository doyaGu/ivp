// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

// Implementation of graphics and demo helper functions

#include "ive_graphics.hxx"
#include "ivp_physics.hxx"
#include "ivu_active_value.hxx"

// Global variables for demo support
char debugstring[1024] = {0};
static IVP_U_Active_Value_Manager *g_active_value_manager = NULL;
static int g_last_key_pressed = 0;
static IVP_DOUBLE g_last_frame_time = 0.016; // Default 60fps

/********************************************************************************
 * Name:        p_demo_get_active_float_manager
 * Description: Returns the global Active Value Manager for keyboard-driven
 *              active floats and demo control
 ********************************************************************************/
IVP_U_Active_Value_Manager *p_demo_get_active_float_manager()
{
    if (!g_active_value_manager)
    {
        g_active_value_manager = new IVP_U_Active_Value_Manager(IVP_TRUE);
    }
    return g_active_value_manager;
}

/********************************************************************************
 * Name:        p_demo_eval_key_input
 * Description: Returns the key pressed by the user
 *              This is a simplified stub implementation
 ********************************************************************************/
int p_demo_eval_key_input(P_Hardware *hw)
{
    // Stub implementation - in a real application, this would query the hardware
    // for actual keyboard input. For demo purposes, this returns the last key set.
    int key = g_last_key_pressed;
    g_last_key_pressed = 0; // Clear after reading
    return key;
}

/********************************************************************************
 * Name:        p_demo_set_key_input
 * Description: Helper function to set key input (for testing/simulation)
 ********************************************************************************/
void p_demo_set_key_input(int key)
{
    g_last_key_pressed = key;
}

/********************************************************************************
 * Name:        p_demo_get_delta_time
 * Description: Returns the time needed to draw a frame (clipped [0 .. 0.1] sec)
 ********************************************************************************/
IVP_DOUBLE p_demo_get_delta_time(IVP_Environment *environment)
{
    // Stub implementation - returns a reasonable default frame time
    IVP_DOUBLE dt = g_last_frame_time;

    // Clip to valid range
    if (dt < 0.0)
        dt = 0.0;
    if (dt > 0.1)
        dt = 0.1;

    return dt;
}

/********************************************************************************
 * Name:        p_demo_set_delta_time
 * Description: Helper to set frame delta time (for testing)
 ********************************************************************************/
void p_demo_set_delta_time(IVP_DOUBLE dt)
{
    g_last_frame_time = dt;
}

/********************************************************************************
 * Name:        p_demo_get_shift_key_state
 * Description: Returns whether shift key is pressed
 ********************************************************************************/
IVP_BOOL p_demo_get_shift_key_state(P_Hardware *hw)
{
    // Stub implementation
    return IVP_FALSE;
}

/********************************************************************************
 * Graphics Support Functions - Stub Implementations
 * These are minimal implementations to satisfy the interface requirements
 ********************************************************************************/

void p_demo_register_texture_server(P_Hardware *hw, P_Texture_Server *server)
{
    // Stub - in full implementation, register texture callback
}

void p_init_demo_display(P_Hardware *hw, IVP_Environment *environment)
{
    // Stub - in full implementation, initialize graphics display
}

void p_set_demo_camera_matrix(IVP_U_Matrix *mat)
{
    // Stub - in full implementation, set camera transformation
}

void p_demo_put_object_in_moving_list(IVP_Real_Object *real_object)
{
    // Stub - in full implementation, add object to render list
}

void p_demo_update_graphics_for_frozen_object(IVP_Real_Object *real_object)
{
    // Stub - in full implementation, update frozen object graphics
}

void p_demo_render_scene(P_Hardware *hw, IVP_Environment *env)
{
    // Stub - in full implementation, render the scene
}

void p_demo_dont_render_next_object()
{
    // Stub - in full implementation, skip next object rendering
}

#if !defined(_XBOX) && (!defined(__MWERKS__) || !defined(__POWERPC__)) && !defined(GEKKO)
void p_demo_replace_graphical_surface(const IVP_Compact_Surface *graphical_surface)
{
    // Stub - in full implementation, replace object's graphical representation
}
#else
void p_demo_replace_graphical_surface(IVP_Real_Object *obj, const IVP_Compact_Surface *graphical_surface)
{
    // Stub - in full implementation, replace object's graphical representation
}
#endif

void p_graphlib_render_scene_splitscreen(P_Hardware *hw)
{
    // Stub - splitscreen rendering
}

void p_graphlib_init_splitscreen(P_Hardware *hw, IVP_Environment *environment)
{
    // Stub - initialize splitscreen
}

void p_graphlib_set_camera_matrix_split_screen(IVP_U_Matrix *mat, int frame_nr)
{
    // Stub - set splitscreen camera
}

void p_demo_create_ambientlight(IVP_U_Point light_color)
{
    // Stub - create ambient light
}

void p_demo_create_pointlight(IVP_U_Point light_color, IVP_U_Point position)
{
    // Stub - create point light
}

void p_demo_move_pointlight(IVP_U_Point position)
{
    // Stub - move point light
}

void p_demo_enable_mousepointer(P_Hardware *hardware)
{
    // Stub - enable mouse pointer
}

void p_demo_disable_mousepointer(P_Hardware *hardware)
{
    // Stub - disable mouse pointer
}

int p_mousepointer_raycast(P_Hardware *hardware, IVP_Environment *environment,
                           IVP_U_Matrix *camera, IVP_U_Point *ray_direction,
                           IVP_Ray_Solver_Min_Hash **raysolver_minhash,
                           IVP_U_Min_Hash **min_hash)
{
    // Stub - mouse raycast
    return 0;
}

/********************************************************************************
 * Sound Support Functions - Stub Implementations
 ********************************************************************************/

void p_init_sound(P_Hardware *hw, char *levelname)
{
    // Stub - initialize sound
}

void p_exit_sound(P_Hardware *hw)
{
    // Stub - cleanup sound
}

void p_demo_make_motor_sound_loud()
{
    // Stub - motor sound
}

void p_demo_refresh_sound()
{
    // Stub - refresh sound
}

void p_demo_refresh_vehicle_sound(IVP_FLOAT rot_speed)
{
    // Stub - vehicle sound
}

/********************************************************************************
 * Statistics Support - Stub Implementation
 ********************************************************************************/

IVP_BetterStatisticsmanager_Callback_Interface *p_demo_get_statisticsmanager_callback(P_Hardware *hardware)
{
    // Stub - statistics manager
    return NULL;
}

/********************************************************************************
 * DirectX Import Functions - Stub Implementations
 ********************************************************************************/

int p_graphlib_convert_directx_object_to_concave_data(P_Hardware *hw, const char *filename,
                                                      IVP_Concave_Polyhedron *concave_polyhedron)
{
    // Stub - DirectX import
    return -1;
}

int p_graphlib_robust_convert_directx_object_to_compact_ledges(P_Hardware *hw, const char *filename,
                                                               IVP_U_BigVector<IVP_Compact_Ledge> *ledges)
{
    // Stub - DirectX import to compact ledges
    return -1;
}

/********************************************************************************
 * IVP_Example_Base Implementation
 ********************************************************************************/

IVP_Example_Base::IVP_Example_Base(IVP_EXAMPLE_MENU_CLASS example_class)
{
    my_name = "Unnamed Example";
    my_level_name = NULL;
    my_helptext = NULL;
    my_introtext = NULL;
    my_class = example_class;
    import_external_environment = IVP_FALSE;
    environment = NULL;
    hardware = NULL;
}

IVP_Example_Base::~IVP_Example_Base()
{
    gc_cleanup();
}

void IVP_Example_Base::register_to_main(IVP_BOOL import_ext_env)
{
    import_external_environment = import_ext_env;
    // In full implementation, register with main demo launcher
}

void IVP_Example_Base::gc_cleanup()
{
    // Clean up garbage collected items
    int i;
    for (i = gc_compactsurface.len() - 1; i >= 0; i--)
    {
        IVP_Compact_Surface *cs = gc_compactsurface.element_at(i);
        if (cs)
            delete cs;
    }
    gc_compactsurface.clear();

    for (i = gc_surfacemanager.len() - 1; i >= 0; i--)
    {
        IVP_SurfaceManager *sm = gc_surfacemanager.element_at(i);
        if (sm)
            delete sm;
    }
    gc_surfacemanager.clear();

    for (i = gc_material.len() - 1; i >= 0; i--)
    {
        IVP_Material *mat = gc_material.element_at(i);
        if (mat)
            delete mat;
    }
    gc_material.clear();
}

int IVP_Example_Base::main()
{
    // Old-style main loop
    setup();

    int result = 0;
    while (result != -1)
    {
        result = step();
        if (result == -2)
        {
            cleanup();
            setup();
            result = 0;
        }
    }

    cleanup();
    return 0;
}

/********************************************************************************
 * Example_Moving_Camera Implementation - Minimal Stub
 ********************************************************************************/

Example_Moving_Camera::Example_Moving_Camera(IVP_Environment &env_in)
{
    env = &env_in;
    init();
}

Example_Moving_Camera::Example_Moving_Camera(IVP_Environment &env_in, const IVP_U_Point &position)
{
    env = &env_in;
    init();
    focus = position;
    manual_position_set = IVP_TRUE;
}

void Example_Moving_Camera::init()
{
    current_mode = IVP_EMCM_ROTATION;
    manual_position_set = IVP_FALSE;
    distance = 10.0f;
    angle_x = 0.0f;
    angle_y = 0.0f;
    focus.set(0.0f, 0.0f, 0.0f);
    offset_center.set(0.0f, 0.0f, 0.0f);
    mouse_x_speed = 1.0f;
    mouse_y_speed = 1.0f;
    initial_speed = 5.0f;
    acceleration = 10.0f;
    m_world_f_camera.init();
    camera_is_pointlight = IVP_FALSE;
    offset_modifier = NULL;
}

void Example_Moving_Camera::set_mode(IVP_EXAMPLE_MOVING_CAMERA_MODE mode)
{
    current_mode = mode;
}

void Example_Moving_Camera::update(IVP_DOUBLE d_time)
{
    // Stub - in full implementation, update camera position
}

void Example_Moving_Camera::show_picture()
{
    // Stub - display current camera view
}

void Example_Moving_Camera::set_offset_modifier(Camera_Offset_Modifier *mod)
{
    offset_modifier = mod;
}

void Example_Moving_Camera::show_picture(const IVP_U_Point &cameraposition, const IVP_U_Point &direction)
{
    // Stub - show picture with camera position and direction
}

void Example_Moving_Camera::show_picture(const IVP_U_Point &cameraposition, const IVP_U_Point &direction, const IVP_U_Point &up)
{
    // Stub - show picture with up vector
}

void Example_Moving_Camera::show_worldpoint(const IVP_U_Point &cameraposition, const IVP_U_Point &worldpoint, const IVP_U_Point *up_direction)
{
    // Stub - show world point
}

void Example_Moving_Camera::rotation(const IVP_U_Point &center, IVP_DOUBLE d_time, IVP_BOOL recalc_values)
{
    // Stub - rotating camera
}

void Example_Moving_Camera::space_movement(IVP_DOUBLE d_time)
{
    // Stub - space movement
}

void Example_Moving_Camera::space_flight(IVP_DOUBLE d_time, IVP_DOUBLE flight_acceleration)
{
    // Stub - space flight
}

void Example_Moving_Camera::space_flight_original(IVP_DOUBLE d_time, IVP_BOOL recalc_values)
{
    // Stub - original space flight
}

void Example_Moving_Camera::space_flight_old(IVP_DOUBLE d_time)
{
    // Stub - old space flight
}

IVP_U_Point Example_Moving_Camera::get_position()
{
    return m_world_f_camera.vv;
}

IVP_U_Point Example_Moving_Camera::get_direction()
{
    IVP_U_Point dir, col;
    m_world_f_camera.get_col(IVP_COORDINATE_INDEX(2), &col);
    dir.set_multiple(&col, -1.0f);
    return dir;
}

IVP_U_Point Example_Moving_Camera::get_upvec()
{
    IVP_U_Point col;
    m_world_f_camera.get_col(IVP_COORDINATE_INDEX(1), &col);
    return col;
}

void Example_Moving_Camera::cameramatrix_workaround()
{
    // Stub
}

void Example_Moving_Camera::convert_position_and_focus_to_angles_and_distance(
    const IVP_U_Point &center)
{
    // Stub
}
