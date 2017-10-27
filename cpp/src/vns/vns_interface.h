/* Copyright (c) 2017, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef PLANNING_CPP_VNS_INTERFACE_H
#define PLANNING_CPP_VNS_INTERFACE_H

#include <ctime>
#include "plan.h"

#include "../ext/json.hpp"
using json = nlohmann::json;

class LocalMove {

public:
    /** Reference to the plan this move is created for. */
    PPlan base_plan;
    explicit LocalMove(PPlan base) : base_plan(base) {};

    /** Cost that would result in applying the move. */
    virtual double utility() = 0;

    /** Total duration that would result in applying the move */
    virtual double duration() = 0;

    /** True if applying this move on the base plan results in a valid plan. */
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
    /** NEVER ACCESS. This is only to make the plan visible in gdb which has problems with shared points. */
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

    virtual std::string name() const = 0;
};

struct Shuffler {
public:
    virtual void suffle(shared_ptr<Plan> plan) = 0;
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
        metadata["plan"] = p.metadata();
    }

    Plan initial() const { return *init_plan; }
    Plan final() const { return *final_plan; }

    json metadata;
};

struct VariableNeighborhoodSearch {
    shared_ptr<Shuffler> shuffler;
    /** Sequence of neighborhoods to be considered by VNS. */
    vector<shared_ptr<Neighborhood>> neighborhoods;

    explicit VariableNeighborhoodSearch(vector<shared_ptr<Neighborhood>>& neighborhoods, shared_ptr<Shuffler> shuffler) :
            neighborhoods(neighborhoods) {
        ASSERT(neighborhoods.size() > 0);
    }

    /** Refines an initial plan with Variable Neighborhood Search.
     *
     * @param p: Initial plan.
     * @param max_restarts: Number of allowed restarts (currently only 0 is supported).
     * @param save_every: If >0, the Search result will contain snapshots of the search every N iterations.
     *                    Warning: this is very heavy on memory usage.
     * @param save_improvements: If set, the Search result will contain snapshots of every improvement in the plan.
     *                    Warning: this is very heavy on memory usage.
     * @return
     */
    SearchResult search(Plan p, size_t max_restarts, size_t save_every=0, bool save_improvements=false) {
        ASSERT(max_restarts == 0) // currently no shaking function

        SearchResult result(p);
        PPlan best_plan = make_shared<Plan>(p);

        size_t current_iter = 0;
        size_t num_restarts = 0;

        vector<double> runtime_per_neighborhood(neighborhoods.size(), 0.0);

        bool saved = false; /*True if an improvement was saved so save_every do not take an snapshot again*/

        while(num_restarts <= max_restarts) {
            // choose first neighborhood
            size_t current_neighborhood = 0;

            while(current_neighborhood < neighborhoods.size()) {
                // current neighborhood
                shared_ptr<Neighborhood> neighborhood = neighborhoods[current_neighborhood];

                // get move for current neighborhood
                clock_t start = clock();
                const opt<PLocalMove> move = neighborhood->get_move(best_plan);
                clock_t end = clock();
                runtime_per_neighborhood[current_neighborhood] += double(end - start) / CLOCKS_PER_SEC;

                if(move) {
                    // neighborhood generate a move, apply it
                    LocalMove& m = **move;
                    ASSERT(m.is_valid());
                    m.apply();

                    printf("Improvement (lvl: %d): utility: %f -- duration: %f\n", (int)current_neighborhood,
                           best_plan->utility(), best_plan->duration());

                    if(save_improvements) {
                        result.intermediate_plans.push_back(*best_plan);
                        saved = true;
                    }

                    // plan changed, go back to first neighborhood
                    current_neighborhood = 0;
                } else {
                    // no move
                    current_neighborhood += 1;
                }
                if(!saved && save_every != 0 && (current_iter % save_every) == 0) {
                    result.intermediate_plans.push_back(*best_plan);
                }
                saved = false;
                current_iter += 1;
            }
            // no neighborhood provides improvements, restart or exit.
            num_restarts += 1;
        }
        result.set_final_plan(*best_plan);

        // save neighborhoods metadata
        result.metadata["neighborhoods"] = json::array();
        for(size_t i=0; i<neighborhoods.size(); i++) {
            json j;
            auto& n = *neighborhoods[i];
            j["name"] = n.name();
            j["runtime"] = runtime_per_neighborhood[i];
            result.metadata["neighborhoods"].push_back(j);
        }
        return result;
    }
};


#endif //PLANNING_CPP_VNS_INTERFACE_H
