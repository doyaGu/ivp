/* Exercise deletion and reset with real engine objects, without a GL window. */
#include "ive_sample_app.hxx"
#include <ivp_cache_object.hxx>
#include <SDL3/SDL.h>
#include <cmath>
#include <cstdio>

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
    return 1; } } while (0)

int main()
{
    CHECK(SDL_Init(SDL_INIT_EVENTS));
    ive::SampleAppConfig cfg;
    ive::app_config_defaults(&cfg);
    ive::SampleApp app(cfg);
    CHECK(app.saved_count == 0 && app.pick_list.count == 0);
    CHECK(app.saved[0].rot.w == 0.0);
    app.env = ive::create_environment();
    app.env->client_data = &app.pick_list;
    IVP_U_Quat rot; rot.init();
    IVP_U_Point first_pos(0.0, -4.0, 0.0), second_pos(3.0, -4.0, 0.0);
    IVP_Ball *first = ive::create_ball(app.env, &app.mat, 0.3, 1.0, &rot, &first_pos);
    IVP_Ball *second = ive::create_ball(app.env, &app.mat, 0.3, 1.0, &rot, &second_pos);
    ive::app_add_pick(&app, first);
    ive::app_add_pick(&app, second);
    ive::app_save_initial_state(&app);
    app.focus.focused = first;
    app.dragger.target = first;
    app.dragger.active = app.dragger.has_target_point = true;
    app.ground_obj = first;
    ive::app_delete_object(&app, first);
    CHECK(app.saved_count == 1 && app.saved[0].obj == second);
    CHECK(app.pick_list.count == 1 && app.pick_list.objs[0] == second);
    CHECK(!app.focus.focused && !app.dragger.target && !app.ground_obj);
    CHECK(!app.dragger.active && !app.dragger.has_target_point);
    IVP_U_Point moved(10.0, -8.0, 0.0);
    second->beam_object_to_new_position(&rot, &moved, IVP_FALSE);
    app.reset_requested = true;
    CHECK(ive::app_check_reset(&app));
    IVP_Cache_Object *cache = second->get_cache_object();
    CHECK(std::fabs(cache->m_world_f_object.get_position()->k[0] - second_pos.k[0]) < 1e-5);
    cache->remove_reference();
    ive::app_delete_object(&app, second);
    CHECK(app.saved_count == 0 && app.pick_list.count == 0);
    app.cam.orbit_distance = 999.0f;
    app.reset_requested = true;
    CHECK(ive::app_check_reset(&app));
    CHECK(app.cam.orbit_distance == app.initial_cam.orbit_distance);
    delete app.env;
    app.env = NULL;
    SDL_Quit();
    return 0;
}
