#ifndef PLANNING_CPP_PLANNING_H
#define PLANNING_CPP_PLANNING_H

#include <vector>
#include "trajectory.h"
#include "visibility.h"
#import "ext/optional.h"
#include "neighborhoods/dubins_optimization.h"
#include "vns_interface.h"
#include "neighborhoods/insertions.h"
#include "neighborhoods/contour_following.h"

using namespace std;


struct DefaultVnsSearch : public VariableNeighborhoodSearch {

    DefaultVnsSearch() : VariableNeighborhoodSearch(defaults)
    {}


private:
    static vector<shared_ptr<Neighborhood>> defaults;
};

vector<shared_ptr<Neighborhood>> DefaultVnsSearch::defaults = {
        make_shared<RandomOrientationChangeNbhd>(),
        make_shared<ProjectOnFrontNbhd>(),
        make_shared<OneInsertNbhd>()
};



#endif //PLANNING_CPP_PLANNING_H
