#include "factory.h"
#include "../ext/json.hpp"

#include "neighborhoods/dubins_optimization.h"
#include "neighborhoods/insertions.h"
#include "neighborhoods/smoothing.h"
#include "neighborhoods/shuffling.h"

using json = nlohmann::json;

void check_field_is_present(json json_obj, std::string field) {
    if((json_obj).find(field) == (json_obj).end()) {
        std::cerr << "Missing field " << field << " in json object:\n" << (json_obj).dump() << std::endl;
    }
    ASSERT((json_obj).find(field) != (json_obj).end())
}

shared_ptr<Neighborhood> build_neighborhood(const json& conf) {
    const std::string& name = conf["name"];


    if(name == "dubins-opt") {
        return make_shared<DubinsOptimizationNeighborhood>();
    }
    if(name == "one-insert") {
        check_field_is_present(conf, "max_trials");
        const size_t max_trials = conf["max_trials"];
        check_field_is_present(conf, "select_arbitrary_trajectory");
        const bool select_arbitrary_trajectory = conf["select_arbitrary_trajectory"];
        check_field_is_present(conf, "select_arbitrary_position");
        const bool select_arbitrary_position = conf["select_arbitrary_position"];
        return make_shared<OneInsertNbhd>(
                max_trials, select_arbitrary_trajectory, select_arbitrary_position
        );
    }
    if(name == "trajectory-smoothing") {
        return make_shared<TrajectorySmoothingNeighborhood>();
    }

    std::cerr << "Unrecognized neighborhood name: " << name << std::endl;
    std::exit(1);
}

shared_ptr<VariableNeighborhoodSearch> vns::build_from_config(const std::string &json_config) {
    auto j = json::parse(json_config);

    auto neighborhoods_confs = j["neighborhoods"];
    std::vector<shared_ptr<Neighborhood>> ns;
    for(auto& it : neighborhoods_confs) {
        ns.push_back(build_neighborhood(it));
    }

    return make_shared<VariableNeighborhoodSearch>(ns, make_shared<PlanPortionRemover>(0., 1.));
}

std::shared_ptr<VariableNeighborhoodSearch> vns::build_default() {
    json conf = R"(
      { "neighborhoods": [
          {"name": "dubins-opt"}, {
           "name": "one-insert",
           "max_trials": 50,
           "select_arbitrary_trajectory": false,
           "select_arbitrary_position": false}] }
)"_json;
    return vns::build_from_config(conf.dump());
}
