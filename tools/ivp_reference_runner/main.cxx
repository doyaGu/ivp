#include <clocale>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#if defined(_WIN32)
#include <process.h>
#endif

#include "ref_runner.hxx"

namespace {

const char *scenario_name(ref_runner::Scenario scenario) {
    switch (scenario) {
        case ref_runner::SCENARIO_FREEFALL: return "freefall";
        case ref_runner::SCENARIO_TWO_BALLS: return "two_balls";
        case ref_runner::SCENARIO_SLOPE_FRICTION: return "slope_friction";
        case ref_runner::SCENARIO_SPRINGS: return "springs";
        case ref_runner::SCENARIO_ROPE: return "rope";
        case ref_runner::SCENARIO_MOTOR: return "motor";
        case ref_runner::SCENARIO_BUOYANCY: return "buoyancy";
        case ref_runner::SCENARIO_CUBES: return "cubes";
        case ref_runner::SCENARIO_FORCE_ACTUATOR: return "force_actuator";
        case ref_runner::SCENARIO_FORCEFIELD: return "forcefield";
        case ref_runner::SCENARIO_STIFF_SPRING: return "stiff_spring";
        case ref_runner::SCENARIO_CHECK_DISTANCE: return "check_distance";
        case ref_runner::SCENARIO_COLLISION_FILTER: return "collision_filter";
        case ref_runner::SCENARIO_MOTION_CONTROLLER: return "motion_controller";
        case ref_runner::SCENARIO_PHANTOM: return "phantom";
        case ref_runner::SCENARIO_VEHICLE: return "vehicle";
        case ref_runner::SCENARIO_CAR_REAL_WHEELS: return "car_real_wheels";
        case ref_runner::SCENARIO_CONCAVE_STATIC: return "concave_static";
        case ref_runner::SCENARIO_COMPOUND_DYNAMIC: return "compound_dynamic";
        case ref_runner::SCENARIO_CONVEX_HULLS: return "convex_hulls";
        case ref_runner::SCENARIO_GRID_TERRAIN: return "grid_terrain";
        case ref_runner::SCENARIO_HULL_PILE: return "hull_pile";
        case ref_runner::SCENARIO_GALTON: return "galton";
        case ref_runner::SCENARIO_FAST_IMPACTS: return "fast_impacts";
        case ref_runner::SCENARIO_SPAWN_REMOVE: return "spawn_remove";
        case ref_runner::SCENARIO_LONG_RANGE: return "long_range";
        case ref_runner::SCENARIO_TORQUE: return "torque";
        case ref_runner::SCENARIO_STABILIZER: return "stabilizer";
        case ref_runner::SCENARIO_FLOATING: return "floating";
        case ref_runner::SCENARIO_WORLD_FRICTION: return "world_friction";
        case ref_runner::SCENARIO_GOLEM: return "golem";
        case ref_runner::SCENARIO_FIXED_KEYED: return "fixed_keyed";
        case ref_runner::SCENARIO_HINGE_LIMITS: return "hinge_limits";
        case ref_runner::SCENARIO_CARDAN_TENSE: return "cardan_tense";
        case ref_runner::SCENARIO_SLIDER_LIMITS: return "slider_limits";
        case ref_runner::SCENARIO_CONSTRAINT_BREAK: return "constraint_break";
        case ref_runner::SCENARIO_ATTACHER: return "attacher";
        case ref_runner::SCENARIO_ACTUATOR_EXTRA: return "actuator_extra";
        case ref_runner::SCENARIO_AIRBOAT: return "airboat";
        case ref_runner::SCENARIO_FAKE_JETSKI: return "fake_jetski";
        case ref_runner::SCENARIO_RAYCAST_CAR_DRIVE: return "raycast_car_drive";
        case ref_runner::SCENARIO_CHECK_DIST_EVENTS: return "check_dist_events";
        case ref_runner::SCENARIO_GOLEM_BEAM: return "golem_beam";
        case ref_runner::SCENARIO_UNIVERSE_EVICT: return "universe_evict";
        case ref_runner::SCENARIO_MERGE_OBJECTS: return "merge_objects";
        case ref_runner::SCENARIO_OBJECT_ATTACH: return "object_attach";
        case ref_runner::SCENARIO_MERGE_BUOYANCY: return "merge_buoyancy";
        default: return 0;
    }
}

std::string executable_dir(const char *argv0) {
    if (!argv0 || !*argv0) {
        return std::string(".");
    }
    std::string path(argv0);
    std::string::size_type slash = path.find_last_of("/\\");
    if (slash == std::string::npos) {
        return std::string(".");
    }
    return path.substr(0, slash);
}

bool file_exists(const std::string &path) {
    std::FILE *fp = std::fopen(path.c_str(), "rb");
    if (!fp) {
        return false;
    }
    std::fclose(fp);
    return true;
}

int run_c17_test_scenarios(const char *argv0, const ref_runner::RunnerConfig &cfg) {
    const char *name = scenario_name(cfg.scenario);
    if (!name) {
        return -1;
    }

    std::string dir = executable_dir(argv0);
#if defined(_WIN32)
    const char *exe_name = "test_scenarios.exe";
    const char *sep = "\\";
#else
    const char *exe_name = "test_scenarios";
    const char *sep = "/";
#endif

    std::string test_scenarios_path = dir + sep + exe_name;
    if (!file_exists(test_scenarios_path)) {
        std::string nested = dir + sep + "ivp-c17" + sep + exe_name;
        if (file_exists(nested)) {
            test_scenarios_path = nested;
        } else {
            return -1;
        }
    }

    std::string temp_output_path = dir + sep + "ivp_reference_runner_delegate.jsonl";

    std::ostringstream dt_ss;
    dt_ss << std::setprecision(17) << cfg.dt;
    std::string dt_str = dt_ss.str();
    std::ostringstream steps_ss;
    steps_ss << cfg.steps;
    std::string steps_str = steps_ss.str();

    int rc = 0;
#if defined(_WIN32)
    const char *argv_spawn[10];
    argv_spawn[0] = test_scenarios_path.c_str();
    argv_spawn[1] = "--scenario";
    argv_spawn[2] = name;
    argv_spawn[3] = "--steps";
    argv_spawn[4] = steps_str.c_str();
    argv_spawn[5] = "--dt";
    argv_spawn[6] = dt_str.c_str();
    argv_spawn[7] = "--output";
    argv_spawn[8] = temp_output_path.c_str();
    argv_spawn[9] = 0;

    rc = _spawnv(_P_WAIT, test_scenarios_path.c_str(), argv_spawn);
#else
    std::ostringstream cmd;
    cmd << '"' << test_scenarios_path << '"'
        << " --scenario " << name
        << " --steps " << cfg.steps
        << " --dt " << dt_str
        << " --output \"" << temp_output_path << "\"";
    rc = std::system(cmd.str().c_str());
#endif
    if (rc != 0) {
        std::remove(temp_output_path.c_str());
        return -1;
    }

    std::FILE *in = std::fopen(temp_output_path.c_str(), "rb");
    if (!in) {
        return -1;
    }

    char buf[8192];
    while (true) {
        std::size_t n = std::fread(buf, 1, sizeof(buf), in);
        if (n > 0) {
            std::fwrite(buf, 1, n, stdout);
        }
        if (n < sizeof(buf)) {
            if (std::feof(in)) {
                break;
            }
            std::fclose(in);
            std::remove(temp_output_path.c_str());
            return -1;
        }
    }

    std::fclose(in);
    std::remove(temp_output_path.c_str());
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    std::setlocale(LC_NUMERIC, "C");

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            ref_runner::print_usage(argv[0]);
            return 0;
        }
    }

    ref_runner::RunnerConfig cfg;
    if (!ref_runner::parse_args(argc, argv, &cfg)) {
        return 2;
    }

    {
        int delegated_rc = run_c17_test_scenarios(argv[0], cfg);
        if (delegated_rc == 0) {
            return 0;
        }
    }

    IVP_Application_Environment appl_env;
    IVP_Environment_Manager *mgr = IVP_Environment_Manager::get_environment_manager();
    IVP_Environment *env = mgr->create_environment(&appl_env, "ivp_reference_runner", 0);
    env->reset_time();
    env->set_delta_PSI_time(cfg.dt);

    ref_runner::SceneObjects scene;
    ref_runner::ScenarioResources resources;

    if (!ref_runner::setup_scenario(cfg.scenario, env, &scene, &resources)) {
        std::fprintf(stderr, "Scenario not implemented: %d\n", (int)cfg.scenario);
        resources.cleanup();
        delete env;
        return 2;
    }

    ref_runner::snapshot_frame(cfg.scenario, 0, cfg.dt, env, &scene);
    for (int step = 1; step <= cfg.steps; ++step) {
        if (resources.step_hook) {
            resources.step_hook(step, env, &resources);
        }
        IVP_Time target = env->get_current_time() + cfg.dt;
        env->simulate_until(target);
        ref_runner::snapshot_frame(cfg.scenario, step, cfg.dt, env, &scene);
    }

    resources.cleanup();
    delete env;
    return 0;
}
