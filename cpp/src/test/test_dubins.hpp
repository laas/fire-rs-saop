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

#include "../ext/dubins.h"
#include "../dubins3d.hpp"
#include <iostream>
#include "../core/structures/waypoint.hpp"
#include <boost/test/included/unit_test.hpp>

namespace SAOP::Test {

    using namespace boost::unit_test;
    using namespace std;

    static double r_min = 25; // m
    static double gamma_max = 0.1; // rad

    void test_helix_optimization_convergence() {
        Waypoint3d orig{546003.90363695205, 6212215.7933926694, 514, 1.9718229235899096};
        Waypoint3d dest{545600, 6213400, 100, 0};

        Dubins3dPath path(orig, dest, 64.457751952217606, 0.10471975511965977); // Should not fail
    }

    void test_length_high_alt() {
        Waypoint3d orig{100, 100, 0, M_PI_2};
        Waypoint3d dest{0, 0, 200, 3 * M_PI_2};

        Dubins3dPath path(orig, dest, r_min, gamma_max);
        BOOST_CHECK(path.goal_altitude == Dubins3dGoalAltitude::High);
    }

    void test_length_medium_alt() {
        Waypoint3d orig{100, 100, 0, M_PI_2};
        Waypoint3d dest{0, 0, 25, 3 * M_PI_2};

        Dubins3dPath path(orig, dest, r_min, gamma_max);
        BOOST_CHECK(path.goal_altitude == Dubins3dGoalAltitude::Medium);
    }

    void test_medium_alt_SSLS() {
        Waypoint3d orig{100, 100, 0, M_PI_2};
        Waypoint3d dest{0, 0, 25, 3 * M_PI_2};

        Dubins3dPath path(orig, dest, r_min, gamma_max);
        BOOST_CHECK(path.goal_altitude == Dubins3dGoalAltitude::Medium);
        BOOST_CHECK(path.configuration == Dubins3dPathType::SSLS);
    }

    void test_length_low_alt() {
        Waypoint3d orig{100, 100, 0, M_PI_2};
        Waypoint3d dest{0, 0, 15, 3 * M_PI_2};

        Dubins3dPath path(orig, dest, r_min, gamma_max);
        BOOST_CHECK(path.goal_altitude == Dubins3dGoalAltitude::Low);

        double orig_array[3]{orig.x, orig.y, orig.dir};
        double dest_array[3]{dest.x, dest.y, dest.dir};
        DubinsPath path2d;
        dubins_init(orig_array, dest_array, r_min, &path2d);
        BOOST_CHECK_CLOSE(path.L_2d, dubins_path_length(&path2d), 1);
    }

/* Triangle inequality should exist for low altitude paths */
    void test_triangleineq_flat() {
        Waypoint3d a{0, 0, 0, 0};
        Waypoint3d b{100, 100, 0, M_PI_2};
        Waypoint3d c{0, 100, 0, M_PI};
        Dubins3dPath path_ab(a, b, r_min, gamma_max);
        Dubins3dPath path_bc(b, c, r_min, gamma_max);
        Dubins3dPath path_ac(a, c, r_min, gamma_max);

        BOOST_TEST(path_ab.L + path_bc.L > path_ac.L);
        BOOST_TEST(path_ab.L_2d + path_bc.L_2d > path_ac.L_2d);
    }

/* Triangle inequality should NOT exist for high altitude paths */
    void test_triangleineq_high() {
        Waypoint3d a{0, 0, 0, 0};
        Waypoint3d b{100, 100, 50, M_PI_2};
        Waypoint3d c{200, 100, 100, M_PI};
        Dubins3dPath path_ab(a, b, r_min, gamma_max);
        Dubins3dPath path_bc(b, c, r_min, gamma_max);
        Dubins3dPath path_ac(a, c, r_min, gamma_max);

        BOOST_CHECK_CLOSE(path_ab.L + path_bc.L, path_ac.L, 1);
        BOOST_CHECK_CLOSE(path_ab.L_2d + path_bc.L_2d, path_ac.L_2d, 1);
    }

    void test_length_flat() {
        DubinsPath path2d;
        Waypoint3d orig{100, 100, 0, M_PI_2};
        Waypoint3d dest{0, 0, 0, 3 * M_PI_2};

        double orig_array[3]{orig.x, orig.y, orig.dir};
        double dest_array[3]{dest.x, dest.y, dest.dir};

        Dubins3dPath path3d(orig, dest, r_min, gamma_max);
        dubins_init(orig_array, dest_array, r_min, &path2d);

        BOOST_TEST(path3d.L == dubins_path_length(&path2d));
    }

    test_suite* dubins_test_suite() {
        test_suite* ts1 = BOOST_TEST_SUITE("dubins_tests");
        ts1->add(BOOST_TEST_CASE(&test_length_flat));
        ts1->add(BOOST_TEST_CASE(&test_length_low_alt));
        ts1->add(BOOST_TEST_CASE(&test_medium_alt_SSLS));
        ts1->add(BOOST_TEST_CASE(&test_length_medium_alt));
        ts1->add(BOOST_TEST_CASE(&test_length_high_alt));
        ts1->add(BOOST_TEST_CASE(&test_length_flat));
        ts1->add(BOOST_TEST_CASE(&test_triangleineq_flat));
        ts1->add(BOOST_TEST_CASE(&test_triangleineq_high));
        ts1->add(BOOST_TEST_CASE(&test_helix_optimization_convergence));
        return ts1;
    }
}