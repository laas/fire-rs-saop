#ifndef PLANNING_CPP_LOCAL_SEARCH_H
#define PLANNING_CPP_LOCAL_SEARCH_H

#include <memory>
#include "trajectory.h"





template<class T>
class Neighborhood {
public:
    virtual T random_neighbour(const T& n) = 0;
};


class OrientationChangeNeighborhood : public Neighborhood<Trajectory> {
public:
    Trajectory random_neighbour(const Trajectory& traj) override {
        auto id = rand() % traj.traj.size();
        double orientation_update = ((double) rand()) / RAND_MAX * M_PI;
        const Segment& s = traj[id];
        Segment newS = Segment(Waypoint(s.start.x, s.start.y, s.start.dir + orientation_update), s.length);
        std::vector<Segment> v = { newS };
        return traj.with_replaced_section(id, v);
    }
};

class TwoOrientationChangeNeighborhood : public Neighborhood<Trajectory> {
public:
    Trajectory random_neighbour(const Trajectory& traj) override {
            assert(traj.traj.size() >= 2);
            auto id = rand() % (traj.traj.size()-1);
            assert(id+1 < traj.traj.size());
            double orientation_update1 = ((double) rand()) / RAND_MAX * M_PI;
            double orientation_update2 = ((double) rand()) / RAND_MAX * M_PI;
            const Segment& s1 = traj[id];
            const Segment& s2 = traj[id+1];

            Segment newS1 = Segment(Waypoint(s1.start.x, s1.start.y, s1.start.dir + orientation_update1), s1.length);
            Segment newS2 = Segment(Waypoint(s2.start.x, s2.start.y, s2.start.dir + orientation_update2), s2.length);
            std::vector<Segment> v = { newS1, newS2 };
        return traj.with_replaced_section(id, v);
    }
};


Trajectory first_improvement_search(const Trajectory& traj, Neighborhood<Trajectory> &neighborhood, unsigned int max_failures) {
        printf("Initial cost: %f\n", traj.length());
        std::unique_ptr<Trajectory> b(new Trajectory(traj));
        unsigned int num_failures = 0;
        while(num_failures < max_failures) {
                Trajectory candidate = neighborhood.random_neighbour(*b);
                if(candidate.length() < b->length()) {
                        printf("Improved cost: %f (previous: %f) (num try: %d)\n", candidate.length(), b->length(), num_failures);
                        b.reset(new Trajectory(candidate));
                        num_failures = 0;
                } else {
                        num_failures++;
                }
        }
        Trajectory result(*b);
        printf("Final cost:    %f  (num try: %d)\n", result.length(), num_failures);
        return result;
}

#endif //PLANNING_CPP_LOCAL_SEARCH_H
