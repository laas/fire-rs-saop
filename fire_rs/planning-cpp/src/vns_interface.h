#ifndef PLANNING_CPP_VNS_INTERFACE_H
#define PLANNING_CPP_VNS_INTERFACE_H


#include "plan.h"

class LocalMove {

public:
    /** Reference to the plan this move is created for. */
    PPlan base_plan;
    LocalMove(PPlan base) : base_plan(base) {}

    /** Cost that would result in applying the move. */
    virtual double cost() const = 0;

    /** Total duration that would result in applying the move */
    virtual double duration() const = 0;

    virtual size_t num_segments() const = 0;

    virtual bool is_valid() const = 0;

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
        if(num_segments() > other.num_segments()) {
            // segments inserted, is better if there is significant cost improvement or if cost is equivalent but durations improves
            return cost() < other.cost() -0.3 ||
                    (ALMOST_LESSER_EQUAL(cost(), other.cost()) &&  duration() < other.duration());
        } else {
            return cost() < other.cost() ||
                    (ALMOST_LESSER_EQUAL(cost(), other.cost()) &&  duration() < other.duration());
        }
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
    IdentityMove(PPlan p) : LocalMove(p) {}

    /** Cost that would result in applying the move. */
    virtual double cost() const override { return base_plan->cost(); };

    /** Total duration that would result in applying the move */
    virtual double duration() const override { return base_plan->duration(); };

    virtual size_t num_segments() const override { return base_plan->num_segments(); };

    virtual bool is_valid() const override { return base_plan->is_valid(); }

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
                    if(move && (*move)->is_valid()) {
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
                    current_neighborhood += 1;
                } else {
                    printf("Improvement (lvl: %d): cost: %f -- duration: %f\n", (int)current_neighborhood, best_plan->cost(), best_plan->duration());
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
