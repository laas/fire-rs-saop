#ifndef PLANNING_CPP_VNS_INTERFACE_H
#define PLANNING_CPP_VNS_INTERFACE_H


#include "plan.h"

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
class IdentityMove final : public LocalMove {
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
    bool stop_on_first_improvement;

    /** Maximum number of neighbors to generate.
     * The contract is that local search algorithms should invoke the "get_move()" method at most
     * "max_neighbors" on a given plan.*/
    size_t max_neighbors;

    Neighborhood(bool stop_on_first_improvement = false, size_t max_neighbors = 50)
            : stop_on_first_improvement(stop_on_first_improvement), max_neighbors(max_neighbors) {}

    /** Generate a new move for the given plan.
     *
     * The optional might be empty if this neighborhood failed to generate a valid move. */
    virtual opt<PLocalMove> get_move(PPlan plan) = 0;
};

struct CombinedNeighborhood : public Neighborhood {
    vector<shared_ptr<Neighborhood>> subneighborhoods;

    CombinedNeighborhood(const vector<shared_ptr<Neighborhood>>& subneighborhoods)
            : subneighborhoods(subneighborhoods) {
        ASSERT(!subneighborhoods.empty())
    }

    opt<PLocalMove> get_move(PPlan plan) override {
        const size_t i = rand() % subneighborhoods.size();
        return subneighborhoods[i]->get_move(plan);
    }
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


#endif //PLANNING_CPP_VNS_INTERFACE_H
