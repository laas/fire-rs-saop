#ifndef PLANNING_CPP_PLANNING_H
#define PLANNING_CPP_PLANNING_H

#include <vector>
#include "trajectory.h"
#include "visibility.h"
#import "ext/optional.h"

using namespace std;

struct Plan;
typedef shared_ptr<Plan> PPlan;

struct Plan {
    vector<Trajectory> trajectories;
    Visibility visibility;

    double cost = -1;
    double length = -1;
    double duration = -1;

    Plan(const Plan& plan) = default;

    Plan(vector<tuple<UAV&,Waypoint,Waypoint>> uav_start_end, Visibility visibility) :
            visibility(visibility)
    {
        for(auto it=uav_start_end.begin(); it!=uav_start_end.end(); it++) {
            auto traj = Trajectory(std::get<0>(*it), vector<Waypoint> { std::get<1>(*it), std::get<2>(*it) });
            trajectories.push_back(traj);
        }
        compute_duration_and_cost();
    }

    Plan(vector<UAV> uavs, Visibility visibility) :
            visibility(visibility)
    {
        for(auto it=uavs.begin(); it!=uavs.end(); it++) {
            auto traj = Trajectory(*it, vector<Waypoint> { });
            trajectories.push_back(traj);
        }
        compute_duration_and_cost();
    }

    void compute_duration_and_cost() {
        length = 0;
        duration = 0;
        for(auto it=trajectories.begin(); it!=trajectories.end(); it++) {
            length += it->length();
            duration += it->duration();
        }

        cost = visibility.cost();
    }

    void add_segment(size_t traj_id, Segment seg, size_t insert_loc) {
        ASSERT(traj_id < trajectories.size());
        ASSERT(insert_loc <= trajectories[traj_id].traj.size());
        trajectories[traj_id] = trajectories[traj_id].with_additional_segment(insert_loc, seg);
        visibility.add_segment(trajectories[traj_id].uav, seg);
        compute_duration_and_cost();
    }

};


class LocalMove {
protected:
    double _cost = -1;
    double _duration = -1;

    static constexpr double INF = 9999999999999999;

public:
    PPlan base_plan;
    LocalMove(PPlan base) : base_plan(base) {}
    inline double cost() const { ASSERT(_cost >= 0); return _cost; };
    inline double duration() const { ASSERT(_duration >= 0); return _duration; };

    /** Applies the move on the inner plan */
    void apply() {
        apply_on(base_plan);
    }

    /** Creates a new Plan on which the mode is applied */
    PPlan apply_on_new() {
        PPlan ret = make_shared<Plan>(*base_plan);
        apply_on(ret);
        return ret;
    }

protected:
    virtual void apply_on(PPlan target) = 0;
};
typedef shared_ptr<LocalMove> PLocalMove;

class IdentityMove : public LocalMove {
public:
    IdentityMove(PPlan p) : LocalMove(p) {
        this->_cost = p->cost;
        this->_duration = p->duration;
    }
    /** does nothing */
    void apply_on(PPlan p) override {
    }
};

//
//vector<Plan> plan(const Raster& ignitions, vector<tuple<UAV&,Waypoint,Waypoint>> uav_start_end, double min_time_window, double max_time_window) {
//    Visibility vis(ignitions, min_time_window, max_time_window);
//
//}



struct Neighborhood {
    bool stop_on_first_improvement = false;
    size_t max_neighbors = 50;
    virtual PLocalMove get_move(PPlan plan) = 0;
};


struct VariableNeighborhoodSearch {
    vector<Neighborhood> neighborhoods;

    vector<Plan> search(Plan p, size_t max_restarts, int save_every=0) {
        ASSERT(max_restarts == 0) // currently no shaking function
        vector<Plan> plans;
        plans.push_back(p);

        PPlan best_plan = make_shared<Plan>(p);

        size_t current_iter = 0;

        size_t num_restarts = 0;
        while(num_restarts <= max_restarts) {
            size_t current_neighborhood = 0;
            while(current_neighborhood < neighborhoods.size()) {
                Neighborhood& neighborhood = neighborhoods[current_neighborhood];

                const PLocalMove no_move = make_shared<IdentityMove>(best_plan);
                PLocalMove best_move = no_move;
                for(size_t i=0; i<neighborhood.max_neighbors; i++) {
                    const PLocalMove move = neighborhood.get_move(best_plan);
                    if(best_move->cost() > move->cost()) {
                        if(best_move == no_move && neighborhood.stop_on_first_improvement) {
                            best_move = move;
                            break;
                        } else {
                            best_move = move;
                        }

                    }
                }
                best_move->apply();

                if(best_move == no_move) {
                    printf("Improvement: %f\n", best_plan->cost);
                    current_neighborhood += 1;
                } else {
                    current_neighborhood = 0;
                }

                if(save_every != 0 && (current_iter % save_every) == 0) {
                    plans.push_back(*best_plan);
                }
                current_iter += 1;

            }



            num_restarts += 1;
        }
    }

};




struct Insert : LocalMove {
    const size_t traj_id;
    const Segment seg;
    const size_t insert_loc;

    Insert(PPlan base, size_t traj_id, Segment seg, size_t insert_loc)
            : LocalMove(base),
              traj_id(traj_id),
              seg(seg),
              insert_loc(insert_loc)
    {
        ASSERT(traj_id < base->trajectories.size());
        ASSERT(insert_loc <= base->trajectories[traj_id].traj.size());
        _cost = base->visibility.cost_given_addition(base->trajectories[traj_id].uav, seg);
        _duration = base->duration + base->trajectories[traj_id].insertion_duration_cost(insert_loc, seg);
    }

    void apply_on(PPlan p) override {
        p->add_segment(traj_id, seg, insert_loc);
        ASSERT(base_plan->trajectories[traj_id].traj.size() == p->trajectories[traj_id].traj.size() -1);
    }

    static experimental::optional<Insert> best_insert(PPlan base, size_t traj_id, Segment seg, double max_cost=INF) {
        ASSERT(traj_id < base->trajectories.size());

        if(max_cost == INF || max_cost >= base->visibility.cost_given_addition(base->trajectories[traj_id].uav, seg)) {
            long best_loc = -1;
            double best_dur = INF;
            Trajectory& traj = base->trajectories[traj_id];
            for(size_t i=0; i<=traj.traj.size(); i++) {
                const double dur = traj.duration() + traj.insertion_duration_cost(i, seg);
                if(best_dur > dur) {
                    best_dur = dur;
                    best_loc = i;
                }
            }
            if(best_loc < 0) {
                return {};
            }
            else {
                return Insert(base, traj_id, seg, (size_t) best_loc);
            }
        } else {
            return {};
        }
    }

    static PPlan smart_insert(PPlan base, size_t traj_id, vector<Segment> segments) {
        experimental::optional<PPlan> current = base;
        for(auto it=segments.begin(); it!=segments.end() && current; it++) {
            current = best_insert(*current, traj_id, *it)->apply_on_new();
        }
        return *current;
    }
};




#endif //PLANNING_CPP_PLANNING_H
