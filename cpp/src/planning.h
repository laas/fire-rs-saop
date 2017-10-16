#ifndef PLANNING_CPP_PLANNING_H
#define PLANNING_CPP_PLANNING_H

#include <vector>
#include "core/structures/trajectory.h"
#import "ext/optional.h"
#include "neighborhoods/dubins_optimization.h"
#include "vns_interface.h"
#include "neighborhoods/insertions.h"
#include "neighborhoods/shuffling.h"
#include "neighborhoods/smoothing.h"

using namespace std;


struct DefaultVnsSearch : public VariableNeighborhoodSearch {

    DefaultVnsSearch() : VariableNeighborhoodSearch(defaults, make_shared<PlanPortionRemover>(0., 1.))
    {}


private:
    static vector<shared_ptr<Neighborhood>> defaults;
};

vector<shared_ptr<Neighborhood>> DefaultVnsSearch::defaults = {
        make_shared<TrajectorySmoothingNeighborhood>(),
        make_shared<DubinsOptimizationNeighborhood>(),
//        make_shared<SegmentInsertNeighborhood>(),
        make_shared<OneInsertNbhd>(),
};



#endif //PLANNING_CPP_PLANNING_H
