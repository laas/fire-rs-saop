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

#ifndef PLANNING_CPP_TEST_DUBINSWIND_HPP
#define PLANNING_CPP_TEST_DUBINSWIND_HPP

#include "../core/dubinswind.hpp"
#include "test_dubins.hpp"
#include "../core/waypoint.hpp"
#include <iostream>
#include <boost/test/included/unit_test.hpp>

namespace SAOP {

    namespace Test {

        using namespace boost::unit_test;
        using namespace std;

        static double uav_speed = 15; // m/s

        void test_dubins_wind() {

            WindVector wind = WindVector(-5, -5);

            Waypoint orig{100, 100, M_PI_2};
            Waypoint dest{0, 0, 3 * M_PI_2};

            DubinsWind dubinswind_path = DubinsWind(orig, dest, wind,uav_speed, SAOP::Test::r_min);
        }

        test_suite* dubinswind_test_suite() {
            test_suite* ts1 = BOOST_TEST_SUITE("dubinswind_tests");
            ts1->add(BOOST_TEST_CASE(&test_dubins_wind));
            return ts1;
        }
    }
}


#endif //PLANNING_CPP_TEST_DUBINSWIND_HPP
