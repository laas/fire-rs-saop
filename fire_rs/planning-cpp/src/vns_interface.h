#ifndef PLANNING_CPP_VNS_INTERFACE_H
#define PLANNING_CPP_VNS_INTERFACE_H


#include "plan.h"

class LocalMove {

public:
    /** Reference to the plan this move is created for. */
    PPlan base_plan;
    LocalMove(PPlan base) : base_plan(base) {}

    /** Cost that would result in applying the move. */
    virtual double cost() = 0;

    /** Total duration that would result in applying the move */
    virtual double duration() = 0;

    virtual size_t num_segments() = 0;

    virtual bool is_valid() = 0;

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

protected:
    /** Internal method that applies the move on a given plan.
     * This should not be used directly. Instead you should the apply/apply_on_new public methods. */
    virtual void apply_on(PPlan target) = 0;

private:
    Plan* plan_raw = base_plan.get();
};
typedef shared_ptr<LocalMove> PLocalMove;


/** Interface for generating local move in local search.
 *
 * A neighborhood provides a single method get_move() that optionally generates a new move.
 * It also contains several parameters, with defaults, that can be overriden by subclasses.*/
struct Neighborhood {

    /** Generates a new move for the given plan.
     * The optional might be empty if this neighborhood failed to generate a valid move.
     *
     * If the optional return is non-empty, local search will apply this move regardless of its cost.
     * Hence subclasses should make sure the returned values contribute to the overall quality of the plan.
     * */
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

    SearchResult search(Plan p, size_t max_restarts, size_t save_every=0) {
        ASSERT(max_restarts == 0) // currently no shaking function

        SearchResult result(p);

        PPlan best_plan = make_shared<Plan>(p);

        size_t current_iter = 0;

        size_t num_restarts = 0;
        while(num_restarts <= max_restarts) {
            size_t current_neighborhood = 0;
            while(current_neighborhood < neighborhoods.size()) {
                shared_ptr<Neighborhood> neighborhood = neighborhoods[current_neighborhood];

                const opt<PLocalMove> move = neighborhood->get_move(best_plan);
                if(move) {
                    // neighborhood generate a move, apply it
                    LocalMove& m = **move;
                    ASSERT(m.is_valid());
                    m.apply();

                    printf("Improvement (lvl: %d): cost: %f -- duration: %f\n", (int)current_neighborhood, best_plan->cost(), best_plan->duration());

                    // plan change, go back to first neighborhood
                    current_neighborhood = 0;
                } else {
                    // no move
                    current_neighborhood += 1;
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
