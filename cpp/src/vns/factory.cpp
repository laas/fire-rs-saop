/* Copyright (c) 2017, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "factory.hpp"
#include "../ext/json.hpp"

#include "neighborhoods/dubins_optimization.hpp"
#include "neighborhoods/insertions.hpp"
#include "neighborhoods/smoothing.hpp"
#include "neighborhoods/shuffling.hpp"

using json = nlohmann::json;

void check_field_is_present(json json_obj, std::string field) {
    if((json_obj).find(field) == (json_obj).end()) {
        std::cerr << "Missing field " << field << " in json object:\n" << (json_obj).dump() << std::endl;
    }
    ASSERT((json_obj).find(field) != (json_obj).end())
}

shared_ptr<OrientationChangeGenerator> build_dubins_optimization_generator(const json& conf) {
    check_field_is_present(conf, "name");
    const std::string& name = conf["name"];

    if(name == "MeanOrientationChangeGenerator") {
        return shared_ptr<OrientationChangeGenerator>(new MeanOrientationChangeGenerator);
    }

    if(name == "RandomOrientationChangeGenerator") {
        return shared_ptr<OrientationChangeGenerator>(new RandomOrientationChangeGenerator);
    }

    if(name == "FlipOrientationChangeGenerator") {
        return shared_ptr<OrientationChangeGenerator>(new FlipOrientationChangeGenerator);
    }

    std::cerr << "Unrecognized OrientationChangeGenerator name: " << name << std::endl;
    std::exit(1);

}

shared_ptr<Neighborhood> build_neighborhood(const json& conf) {
    const std::string& name = conf["name"];

    if(name == "dubins-opt") {
        check_field_is_present(conf, "max_trials");
        const size_t max_trials = conf["max_trials"];

        check_field_is_present(conf, "generators");
        auto generators_j = conf["generators"];
        vector<shared_ptr<OrientationChangeGenerator>> generators;
        for(auto& it : generators_j) {
            generators.push_back(build_dubins_optimization_generator(it));
        }
        
        return make_shared<DubinsOptimizationNeighborhood>(generators, max_trials);
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
        check_field_is_present(conf, "max_trials");
        const size_t max_trials = conf["max_trials"];
        return make_shared<TrajectorySmoothingNeighborhood>(max_trials);
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
    check_field_is_present(j, "shuffler");
    check_field_is_present(j["shuffler"], "min_removal_ratio");
    check_field_is_present(j["shuffler"], "max_removal_ratio");
    const double min_removal_ratio = j["shuffler"]["min_removal_ratio"];
    const double max_removal_ratio = j["shuffler"]["max_removal_ratio"];

    return make_shared<VariableNeighborhoodSearch>(
            ns,
            make_shared<PlanPortionRemover>(min_removal_ratio, max_removal_ratio));
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
