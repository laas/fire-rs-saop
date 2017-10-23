//
// Created by saop on 10/23/17.
//

#include "factory.h"
#include "../ext/json.hpp"

using json = nlohmann::json;

void neighborhood::build_from_config(std::string &json_config) {
    auto j = json::parse(json_config);

    auto neiborhoods_confs = j["neighborhoods"];
    for(auto& it : neiborhoods_confs) {
        std::cout << it["name"] << endl;
    }


}