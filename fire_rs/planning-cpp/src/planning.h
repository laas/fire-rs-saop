#ifndef PLANNING_CPP_PLANNING_H
#define PLANNING_CPP_PLANNING_H

#include <vector>
#include "trajectory.h"
#include "visibility.h"
#import "ext/optional.h"

using namespace std;

struct Plan;
typedef shared_ptr<Plan> PPlan;

template<class T>
using opt = experimental::optional<T>;

struct Plan {
    vector<Trajectory> trajectories;
    Visibility visibility;

    double cost = -1;
    double length = -1;
    double duration = -1;

    Plan(const Plan& plan) = default;

    Plan(vector<TrajectoryConfig> traj_confs, Visibility visibility) :
            visibility(visibility)
    {
        for(auto conf=traj_confs.begin(); conf!=traj_confs.end(); conf++) {
            auto traj = Trajectory(*conf);
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
        visibility.add_segment(trajectories[traj_id].conf->uav, seg);
        compute_duration_and_cost();
    }

};


class LocalMove {
protected:
    /** Cost that would result in applying the move. To be filled at instantiation */
    double _cost = -1;

    /** Total duration (sum of all subplans durations) that would result in applying the move*/
    double _duration = -1;

    static constexpr double INF = 9999999999999999;

public:
    /** Reference to the plan this move is created for. */
    PPlan base_plan;
    LocalMove(PPlan base) : base_plan(base) {}

    /** Cost that would result in applying the move. */
    inline double cost() const { ASSERT(_cost >= 0); return _cost; };

    /** Total duration that would result in applying the move */
    inline double duration() const { ASSERT(_duration >= 0); return _duration; };

    /** Applies the move on the inner plan */
    void apply() {
        apply_on(base_plan);
    }

    /** Creates a new Plan on which the move is applied */
    PPlan apply_on_new() {
        PPlan ret = make_shared<Plan>(*base_plan);
        apply_on(ret);
        return ret;
    }

    /** this move is better than another if it has a better cost or if it has a similar cost but a strictly better duration */
    bool is_better_than(const LocalMove& other) const {
        return cost() < other.cost() || (abs(cost() - other.cost()) < 0.0001 && duration() < other.duration());
    }

protected:
    /** Internal method that applies the move on a given plan.
     * This should not be used directly. Instead you should the apply/apply_on_new public methods. */
    virtual void apply_on(PPlan target) = 0;

private:
    Plan* plan_raw = base_plan.get();
};
typedef shared_ptr<LocalMove> PLocalMove;


/** A simple move that does nothing. Typically used to represent a fix-point in type-safe manner. */
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


/** Interface for generating local move in local search.
 *
 * A neighborhood provides a single method get_move() that optionally generates a new move.
 * It also contains several parameters, with defaults, that can be overriden by subclasses.*/
struct Neighborhood {
    /** Should be set to true, if local search should apply should stop on the first move that */
    bool stop_on_first_improvement = false;

    /** Maximum number of neighbors to generate.
     * The contract is that local search algorithms should invoke the "get_move()" method at most
     * "max_neighbors" on a given plan.*/
    size_t max_neighbors = 50;

    /** Generate a new move for the given plan.
     *
     * The optional might be empty if this neighborhood failed to generate a valid move. */
    virtual opt<PLocalMove> get_move(PPlan plan) = 0;
};

struct SearchResult {
    shared_ptr<Plan> init_plan;
    shared_ptr<Plan> final_plan;

public:
    vector<Plan> intermediate_plans;

    SearchResult(Plan& init_plan)
            : init_plan(shared_ptr<Plan>(new Plan(init_plan))),
              final_plan(shared_ptr<Plan>())
    {}

    void set_final_plan(Plan& p) {
        ASSERT(!final_plan)
        final_plan.reset(new Plan(p));
    }

    Plan initial() const { return *init_plan; }
    Plan final() const { return *final_plan; }


};

struct VariableNeighborhoodSearch {
    vector<shared_ptr<Neighborhood>> neighborhoods;

    VariableNeighborhoodSearch(vector<shared_ptr<Neighborhood>>& neighborhoods) :
            neighborhoods(neighborhoods) {}

    SearchResult search(Plan p, size_t max_restarts, int save_every=0, bool verbose = true) {
        ASSERT(max_restarts == 0) // currently no shaking function

        SearchResult result(p);

        PPlan best_plan = make_shared<Plan>(p);

        size_t current_iter = 0;

        size_t num_restarts = 0;
        while(num_restarts <= max_restarts) {
            size_t current_neighborhood = 0;
            while(current_neighborhood < neighborhoods.size()) {
                shared_ptr<Neighborhood> neighborhood = neighborhoods[current_neighborhood];

                const PLocalMove no_move = make_shared<IdentityMove>(best_plan);
                PLocalMove best_move = no_move;
                for(size_t i=0; i<neighborhood->max_neighbors; i++) {
                    const opt<PLocalMove> move = neighborhood->get_move(best_plan);
                    if(move) {
                        if ((*move)->is_better_than(*best_move)) {
                            if (best_move == no_move && neighborhood->stop_on_first_improvement) {
                                best_move = *move;
                                break;
                            } else {
                                best_move = *move;
                            }
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
                    result.intermediate_plans.push_back(*best_plan);
                }
                current_iter += 1;
            }

            num_restarts += 1;
        }
        result.set_final_plan(*best_plan);
        return result;
    }
};



/** Local move that insert a segment at given place in the plan. */
struct Insert : public LocalMove {
    /** Index of the trajectory in which to perform the insertion. */
    size_t traj_id;

    /** Segment to insert. */
    Segment seg;

    /** Place in the trajectory where this segment should be inserted. */
    size_t insert_loc;

    Insert(PPlan base, size_t traj_id, Segment seg, size_t insert_loc)
            : LocalMove(base),
              traj_id(traj_id),
              seg(seg),
              insert_loc(insert_loc)
    {
        ASSERT(traj_id < base->trajectories.size());
        ASSERT(insert_loc <= base->trajectories[traj_id].traj.size());
        _cost = base->visibility.cost_given_addition(base->trajectories[traj_id].conf->uav, seg);
        _duration = base->duration + base->trajectories[traj_id].insertion_duration_cost(insert_loc, seg);
    }

    void apply_on(PPlan p) override {
        p->add_segment(traj_id, seg, insert_loc);
    }

    /** Generates an insert move that include the segment at the best place in the given trajectory.
     * Currently, this does not checks the trajectory constraints. */
    static opt<Insert> best_insert(PPlan base, size_t traj_id, Segment seg, double max_cost=INF) {
        ASSERT(traj_id < base->trajectories.size());

        if(max_cost == INF || max_cost >= base->visibility.cost_given_addition(base->trajectories[traj_id].conf->uav, seg)) {
            long best_loc = -1;
            double best_dur = INF;
            Trajectory& traj = base->trajectories[traj_id];
            for(size_t i=0; i<=traj.traj.size(); i++) {
                const double dur = traj.duration() + traj.insertion_duration_cost(i, seg);
                if(dur <=  traj.conf->max_flight_time) {
                    if (best_dur > dur) {
                        best_dur = dur;
                        best_loc = i;
                    }
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

    /** Tries to insert each of the segments at the best place in the given trajectory of the given plan */
    static PPlan smart_insert(PPlan base, size_t traj_id, vector<Segment> segments) {
        opt<PPlan> current = base;
        for(auto it=segments.begin(); it!=segments.end() && current; it++) {
            current = best_insert(*current, traj_id, *it)->apply_on_new();
        }
        return *current;
    }
};

/** Neighborhood for generating Insert moves.
 *
 * The neighborhood randomly picks a pending point in the visibility structures and returns
 * the best insertion for this point.
 */
struct OneInsertNbhd : public Neighborhood {

    opt<PLocalMove> get_move(PPlan p) override {
        if(p->visibility.interesting_pending.size() == 0) {
            // no candidates left
            return {};
        }
        /** Select a random point in the pending list */
        const size_t index = rand() % p->visibility.interesting_pending.size();
        const Point pt = p->visibility.interesting_pending[index];

        /** Pick an angle randomly */
        const double angle = ((double) rand() / RAND_MAX) * (M_PI * 2);

        /** Waypoint and segment resulting from the random picks */
        const Waypoint wp = Waypoint(p->visibility.ignitions.x_coords(pt.x), p->visibility.ignitions.y_coords(pt.y), angle);
        const Segment seg = Segment(wp.forward(-30), 30);

        opt<Insert> best_insert = {};

        /** Try best insert for each subtrajectory in the plan */
        for(size_t i=0; i< p->trajectories.size(); i++) {
            auto candidate = Insert::best_insert(p, i, seg);
            if(candidate) {
                if(!best_insert || candidate->is_better_than(*best_insert))
                    best_insert = candidate;
            }
        }

        /** Return the best, if any */
        if(best_insert)
            return opt<PLocalMove>(make_shared<Insert>(*best_insert));
        else
            return {};
    }
};




#endif //PLANNING_CPP_PLANNING_H
