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

#ifndef PROJECT_TEST_REVERSIBLE_UPDATES_H
#define PROJECT_TEST_REVERSIBLE_UPDATES_H


#include "../../core/trajectories.hpp"
#include "../../core/updates/updates.hpp"
#include <boost/test/included/unit_test.hpp>

namespace SAOP {
    namespace Test {

        using namespace boost::unit_test;

        Segment3d seg(double xy) {
            return Segment3d(Waypoint3d(xy, xy, 0, 0));
        }

        Trajectories default_plan() {
            UAV uav("test", 10., 32. * M_PI / 180, 0.1);
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
            InsertSegmentUpdate(0, seg(1), 1).apply(ts);
            InsertSegmentUpdate(0, seg(2), 2).apply(ts);
            InsertSegmentUpdate(0, seg(3), 3).apply(ts);
            BOOST_CHECK(ts.num_segments() == 5);

            double dur = ts.duration();
            auto rev = DeleteSegmentUpdate(0, 1).apply(ts);
            BOOST_CHECK(ts.num_segments() == 4);
            BOOST_CHECK(ts.duration() < dur);
            BOOST_CHECK(ts[0][1].maneuver.start.x == 2);
            rev->apply(ts);
            BOOST_CHECK(ALMOST_EQUAL(ts.duration(), dur));

            Segment3d old_seg = ts[0][3].maneuver;
            Segment3d new_seg = seg(4);
            size_t orig_size = ts.num_segments();
            auto updt_rev = ReplaceSegmentUpdate(0, 3, new_seg).apply(ts);
            BOOST_CHECK(ts.num_segments() == orig_size);
            BOOST_CHECK(ts[0][3].maneuver.start.x == new_seg.start.x);
            updt_rev->apply(ts);
            BOOST_CHECK(ts[0][3].maneuver.start.x == old_seg.start.x);
            BOOST_CHECK(ALMOST_EQUAL(ts.duration(), dur));

            return ts;
        }

        test_suite* reversible_updates_test_suite() {
            test_suite* ts3 = BOOST_TEST_SUITE("reversible_updates_tests");
            ts3->add(BOOST_TEST_CASE(&default_plan));

            return ts3;
        }
    }
}
#endif //PROJECT_TEST_REVERSIBLE_UPDATES_H
