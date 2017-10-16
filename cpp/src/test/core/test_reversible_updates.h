#ifndef PROJECT_TEST_REVERSIBLE_UPDATES_H
#define PROJECT_TEST_REVERSIBLE_UPDATES_H


#include "../../core/structures/trajectories.h"
#include "../../core/updates/updates.h"


Segment3d seg(double xy) {
    return Segment3d(Waypoint3d(xy, xy, 0, 0));
}

Trajectories default_plan() {
    UAV uav(10., 32.*M_PI/180, 0.1);
    Waypoint3d start(5, 5, 0, 0);
    Waypoint3d end(11, 11, 0, 0);

    // circular firedata spread
    DRaster ignitions(100, 100, 0, 0, 1);
    ignitions.reset();

    auto fd = make_shared<FireData>(ignitions, ignitions);
    vector<TrajectoryConfig> confs{TrajectoryConfig(
            uav,
            start,
            end,
            10)};
    Trajectories ts(confs);
    InsertSegment(0, seg(1), 1).apply(ts);
    InsertSegment(0, seg(2), 2).apply(ts);
    InsertSegment(0, seg(3), 3).apply(ts);
    ASSERT(ts.num_segments() == 5)

    double dur = ts.duration();
    auto rev = DeleteSegment(0, 1).apply(ts);
    ASSERT(ts.num_segments() == 4);
    ASSERT(ts.duration() < dur);
    ASSERT(ts.trajectories[0][1].start.x == 2)
    rev->apply(ts);
    ASSERT(ALMOST_EQUAL(ts.duration(), dur))


    return ts;
}

void all_reversible_updates_test() {
    default_plan();
}

#endif //PROJECT_TEST_REVERSIBLE_UPDATES_H
