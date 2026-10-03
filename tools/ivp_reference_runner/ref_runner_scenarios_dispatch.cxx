#include "ref_runner.hxx"

namespace ref_runner {

bool setup_scenario(Scenario scenario, IVP_Environment *env, SceneObjects *scene, ScenarioResources *resources) {
    if (setup_basic_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    if (setup_constraint_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    if (setup_controller_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    if (setup_car_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    if (setup_geometry_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    if (setup_controllers2_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    if (setup_controllers3_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    if (setup_misc_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    if (setup_merge_scenarios(scenario, env, scene, resources)) {
        return true;
    }
    return false;
}

} // namespace ref_runner
