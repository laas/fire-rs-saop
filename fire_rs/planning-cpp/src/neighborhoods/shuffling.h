#ifndef PLANNING_CPP_SHUFFLING_H
#define PLANNING_CPP_SHUFFLING_H

#include "../vns_interface.h"

struct PlanPortionRemover : public Shuffler {

    explicit PlanPortionRemover(double min_removal_portion = 0., double max_removal_portion=1.) :
            min_removal_portion(min_removal_portion), max_removal_portion(max_removal_portion) {}

    const double min_removal_portion;
    const double max_removal_portion;

    void suffle(shared_ptr<Plan> plan) override {
        for(auto& traj : plan->trajectories) {
            const size_t num_removable = traj.last_modifiable() + 1 - traj.first_modifiable();
            ASSERT(num_removable < traj.last_modifiable());
            const size_t to_remove_lb = (size_t) max(0, (int) floor(min_removal_portion * num_removable));
            const size_t to_remove_ub = (size_t) min((int) num_removable, (int) floor(max_removal_portion * num_removable));
            // number of segments to remove
            const size_t to_remove = rand(to_remove_lb, to_remove_ub+1);

            size_t next_removal = rand(traj.first_modifiable(), traj.last_modifiable()+1);
            size_t removed = 0;
            while(removed < to_remove) {
                if(next_removal > traj.last_modifiable())
                    next_removal = traj.first_modifiable();
                traj.erase_segment(next_removal);
                removed++;
            }
        }
        plan->post_process();
    }

};

#endif //PLANNING_CPP_SHUFFLING_H
