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

#include <iostream>

#include "../ext/dubins.h"
#include "../core/structures/trajectory.hpp"
#include "../raster.hpp"
#include "../core/structures/uav.hpp"
#include "../vns/vns_interface.hpp"
#include "../vns/factory.hpp"
#include "../vns/neighborhoods/dubins_optimization.hpp"
#include "../vns/fire_data.hpp"
#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test;

UAV uav(10., 32.*M_PI/180, 0.1);

void test_single_point_to_observe() {
    // all points ignited at time 0, except ont at time 100
    DRaster ignitions(100, 100, 0, 0, 25);
    ignitions.set(10, 10, 100);

    DRaster elevation(100, 100, 0, 0, 25);

    auto fd = make_shared<FireData>(ignitions, elevation);

    // only interested in the point ignited at time 100
    vector<TrajectoryConfig> confs { TrajectoryConfig(uav, 100) };
    Plan p(confs, fd, TimeWindow{90, 110});


    auto vns = vns::build_default();

    auto res = vns->search(p, 0, 1);
    BOOST_CHECK(res.final_plan);
    Plan solution = res.final();

    cout << "SUCCESS" << endl;
}

void test_many_points_to_observe() {
    // circular firedata spread
    DRaster ignitions(100, 100, 0, 0, 1);
    for (size_t x = 0; x < 100; x++) {
        for (size_t y = 0; y < 100; y++) {
            ignitions.set(x, y, sqrt(pow((double) x-50,2) + pow((double) y-50, 2)));
        }
    }

    DRaster elevation(100, 100, 0, 0, 1);

    auto fd = make_shared<FireData>(ignitions, elevation);
    vector<TrajectoryConfig> confs { TrajectoryConfig(uav, 10) };
    Plan p(confs, fd, TimeWindow{0, 110});

    auto vns = vns::build_default();

    auto res = vns->search(p, 0, 1);
    BOOST_CHECK(res.final_plan);
    Plan solution = res.final();

    cout << "SUCCESS" << endl;
}

void test_many_points_to_observe_with_start_end_positions() {
    Waypoint3d start(5,5,0,0);
    Waypoint3d end(11,11,0,0);

    // circular firedata spread
    DRaster ignitions(100, 100, 0, 0, 1);
    for (size_t x = 0; x < 100; x++) {
        for (size_t y = 0; y < 100; y++) {
            ignitions.set(x, y, sqrt(pow((double) x-50,2) + pow((double) y-50, 2)));
        }
    }

    DRaster elevation(100, 100, 0, 0, 1);

    auto fd = make_shared<FireData>(ignitions, elevation);
    vector<TrajectoryConfig> confs { TrajectoryConfig(
            uav,
            start,
            end,
            10) };
    Plan p(confs, fd, TimeWindow{0, 110});

    auto vns = vns::build_default();

    auto res = vns->search(p, 0, 1);
    BOOST_CHECK(res.final_plan);
    Plan solution = res.final();

    auto& traj = solution.core[0];
//    ASSERT(traj[0] == start);
//    ASSERT(traj[traj.size()-1] == end);
    BOOST_CHECK(traj.first_modifiable() == 1);
    BOOST_CHECK(traj.last_modifiable() == traj.size()-2);

    cout << "SUCCESS" << endl;
}


void test_segment_rotation() {
    for(size_t i=0; i<100; i++) {
        Waypoint wp(drand(-100000, 10000), drand(-100000, 100000), drand(-10*M_PI, 10*M_PI));
        Segment seg(wp, drand(0, 1000));

        Segment seg_rotated = uav.rotate_on_visibility_center(seg, drand(-10*M_PI, 10*M_PI));
        Segment seg_back = uav.rotate_on_visibility_center(seg_rotated, wp.dir);
        BOOST_CHECK(seg == seg_back);
    }
}

void test_projection_on_firefront() {
    // uniform propagation along the y axis
    {
        DRaster ignitions(100, 100, 0, 0, 1);
        for (size_t x = 0; x < 100; x++) {
            for (size_t y = 0; y < 100; y++) {
                ignitions.set(x, y, y);
            }
        }

        DRaster elevation(100, 100, 0, 0, 1);

        FireData fd(ignitions, elevation);
        auto res = fd.project_on_fire_front(Cell{1, 1}, 50.5);
        BOOST_CHECK(res && res->y == 50);


        auto res_back = fd.project_on_fire_front(Cell{79, 1}, 50.5);
        BOOST_CHECK(res_back && res_back->y == 50);
    }

    // uniform propagation along the x axis
    {
        DRaster ignitions(10, 10, 0, 0, 1);
        for (size_t x = 0; x < 10; x++) {
            for (size_t y = 0; y < 10; y++) {
                ignitions.set(x, y, x);
            }
        }

        DRaster elevation(10, 10, 0, 0, 1);

        FireData fd(ignitions, elevation);
        auto res = fd.project_on_fire_front(Cell{1, 1}, 5.5);
        BOOST_CHECK(res && res->x == 5);


        auto res_back = fd.project_on_fire_front(Cell{7, 1}, 5.5);
        BOOST_CHECK(res_back && res_back->x == 5);
    }
    // circular propagation center on (50,50)
    {
        auto dist = [](size_t x, size_t y) { return sqrt(pow((double) x - 50., 2.) + pow((double) y - 50., 2.)); };
        DRaster ignitions(100, 100, 0, 0, 1);
        for (size_t x = 0; x < 100; x++) {
            for (size_t y = 0; y < 100; y++) {
                ignitions.set(x, y, dist(x, y));

            }
        }

        DRaster elevation(100, 100, 0, 0, 1);

        FireData fd(ignitions, elevation);
        for(size_t i=0; i<100; i++) {
            const size_t x = rand(0, 100);
            const size_t y = rand(0, 100);
            auto res = fd.project_on_fire_front(Cell{x, y}, 25);
            BOOST_CHECK(res && abs(dist(res->x, res->y) - 25) <1.5);
        }
    }
}

void test_trajectory_as_waypoints() {
    Trajectory traj((TrajectoryConfig(uav)));
    traj.sampled(2);
}

void test_trajectory_slice() {

    TimeWindow tw1 = TimeWindow(10, 300);

    TrajectoryConfig config1 = TrajectoryConfig(uav, tw1.start, tw1.end);
    Trajectory traj = Trajectory(config1);
    traj.append_segment(Segment3d(Waypoint3d(0, 0, 0, 0)));
    traj.append_segment(Segment3d(Waypoint3d(100, 100, 0, 0), 50));
    traj.append_segment(Segment3d(Waypoint3d(300, 200, 0, 0), 50));
    traj.append_segment(Segment3d(Waypoint3d(500, 500, 0, 0)));

    Trajectory sliced1 = traj.slice(TimeWindow(tw1.start + 1, tw1.end - 1));
    Trajectory sliced2 = traj.slice(TimeWindow(tw1.start + 1, 85));

    BOOST_CHECK(sliced1.size() == traj.size() - 1);
    BOOST_CHECK(sliced2.size() == traj.size() - 2);

    BOOST_CHECK_CLOSE(traj.start_time(1), sliced1.start_time(0), 0.1);
    BOOST_CHECK_CLOSE(traj.start_time(1), sliced2.start_time(0), 0.1);
    BOOST_CHECK_CLOSE(traj.start_time(3), sliced1.start_time(2), 0.1);
    BOOST_CHECK_CLOSE(traj.start_time(2), sliced2.start_time(1), 0.1);

}

void test_time_window_order() {
    double s = 10;
    double e = 25;
    TimeWindow tw1 = TimeWindow(s, e);
    TimeWindow tw2 = TimeWindow(e, s);

    BOOST_CHECK(tw1.start == s);
    BOOST_CHECK(tw1.end == e);
    BOOST_CHECK(tw1.start == tw2.start);
    BOOST_CHECK(tw1.end == tw1.end);
}

test_suite* position_manipulation_test_suite()
{
    test_suite* ts2 = BOOST_TEST_SUITE( "position_manipulation_tests" );
    srand(time(0));
    ts2->add(BOOST_TEST_CASE(&test_trajectory_slice));
    ts2->add(BOOST_TEST_CASE(&test_time_window_order));
    ts2->add(BOOST_TEST_CASE(&test_trajectory_as_waypoints));
    ts2->add(BOOST_TEST_CASE(&test_segment_rotation));
    ts2->add(BOOST_TEST_CASE(&test_single_point_to_observe));
    ts2->add(BOOST_TEST_CASE(&test_many_points_to_observe));
    ts2->add(BOOST_TEST_CASE(&test_many_points_to_observe_with_start_end_positions));
    ts2->add(BOOST_TEST_CASE(&test_projection_on_firefront));

    return ts2;
}
