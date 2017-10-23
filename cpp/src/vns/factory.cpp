#include "factory.h"
#include "../ext/json.hpp"

#include "neighborhoods/dubins_optimization.h"
#include "neighborhoods/insertions.h"
#include "neighborhoods/smoothing.h"
#include "neighborhoods/shuffling.h"

using json = nlohmann::json;


shared_ptr<Neighborhood> build_neighborhood(const json& conf) {
    const std::string& name = conf["name"];

    if(name == "dubins-opt") {
        return make_shared<DubinsOptimizationNeighborhood>();
    }
    if(name == "one-insert") {
        return make_shared<OneInsertNbhd>();
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
        std::cout << it["name"] << endl;
    }

    return make_shared<VariableNeighborhoodSearch>(ns, make_shared<PlanPortionRemover>(0., 1.));
}
