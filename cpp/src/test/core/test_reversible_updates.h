#ifndef PROJECT_TEST_REVERSIBLE_UPDATES_H
#define PROJECT_TEST_REVERSIBLE_UPDATES_H


#include "../../core/structures/trajectories.h"
#include "../../core/updates/updates.h"
#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test;

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
    BOOST_CHECK(ts.num_segments() == 5);

    double dur = ts.duration();
    auto rev = DeleteSegment(0, 1).apply(ts);
    BOOST_CHECK(ts.num_segments() == 4);
    BOOST_CHECK(ts.duration() < dur);
    BOOST_CHECK(ts.trajectories[0][1].start.x == 2);
    rev->apply(ts);
    BOOST_CHECK(ALMOST_EQUAL(ts.duration(), dur));


    return ts;
}

test_suite* reversible_updates_test_suite() {
    test_suite* ts3 = BOOST_TEST_SUITE( "reversible_updates_tests" );
    ts3->add(BOOST_TEST_CASE(&default_plan));

    return ts3;
}

#endif //PROJECT_TEST_REVERSIBLE_UPDATES_H
