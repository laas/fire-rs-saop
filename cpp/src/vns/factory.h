#ifndef PROJECT_VNS_FACTORY_H
#define PROJECT_VNS_FACTORY_H


#include "vns_interface.h"

namespace vns {

    std::shared_ptr<VariableNeighborhoodSearch> build_from_config(const std::string& json_config);

    std::shared_ptr<VariableNeighborhoodSearch> build_default();

}

#endif //PROJECT_VNS_FACTORY_H
