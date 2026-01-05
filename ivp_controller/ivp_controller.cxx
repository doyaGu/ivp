#include "ivp_controller.hxx"

IVP_Event_Sim::IVP_Event_Sim(IVP_Environment *env, IVP_DOUBLE dtime)
    : sim_unit(NULL)
{
    environment = env;
    delta_time = dtime;
    if (dtime > P_FLOAT_EPS)
    {
        i_delta_time = 1.0f / dtime;
    }
    else
    {
        i_delta_time = 1.0f / P_FLOAT_EPS;
    }
}

IVP_Event_Sim::IVP_Event_Sim(IVP_Environment *env)
    : sim_unit(NULL)
{
    environment = env;
    delta_time = env->get_delta_PSI_time();
    i_delta_time = env->get_inv_delta_PSI_time();
}