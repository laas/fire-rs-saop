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
#include "../dubins3d.h"
#include <iostream>
using namespace std;
//#include "../waypoint.h"
//#include <cmath>

static double r_min = 25; // m
static double gamma_max = 0.1; // rad

bool test_length_high_alt() {
    cout << "::test_length_high_alt::" << endl;

    Waypoint3d orig {100, 100, 0, M_PI_2};
    Waypoint3d dest {0, 0, 200, 3 * M_PI_2};

    Dubins3dPathLength path(orig, dest, r_min, gamma_max);

    cout << orig << " -> " << dest << " => " << path.L << "xyz / " << path.L_2d << "xy" << endl;
    return true;
}

bool test_length_medium_alt() {
    cout << "::test_length_medium_alt::" << endl;

    Waypoint3d orig {100, 100, 0, M_PI_2};
    Waypoint3d dest {0, 0, 50, 3 * M_PI_2};

    Dubins3dPathLength path(orig, dest, r_min, gamma_max);

    cout << orig << " -> " << dest << " => " << path.L << "xyz / " << path.L_2d << "xy" << endl;
    return true;
}

bool test_medium_alt_SSLS() {
    cout << "::test_medium_alt_SSLS::" << endl;
    Waypoint3d orig {100, 100, 0, M_PI_2};
    Waypoint3d dest {0, 0, 25, 3 * M_PI_2};

    Dubins3dPathLength path(orig, dest, r_min, gamma_max);

    ASSERT(path.R>=0);

    cout << orig << " -> " << dest << " => " << path.L << "xyz / " << path.L_2d << "xy" << endl;
    return true;
}

bool test_length_low_alt() {
    cout << "::test_length_low_alt::" << endl;

    Waypoint3d orig {100, 100, 0, M_PI_2};
    Waypoint3d dest {0, 0, 15, 3 * M_PI_2};

    Dubins3dPathLength path(orig, dest, r_min, gamma_max);

    cout << orig << " -> " << dest << " => " << path.L << "xyz / " << path.L_2d << "xy" << endl;
    return true;
}

bool test_length_flat() {
    DubinsPath path2d;
    Waypoint3d orig {100, 100, 0, M_PI_2};
    Waypoint3d dest {0, 0, 0, 3 * M_PI_2};

    double orig_array[3] {orig.x, orig.y, orig.dir};
    double dest_array[3] {dest.x, dest.y, dest.dir};

    Dubins3dPathLength path3d(orig, dest, r_min, gamma_max);
    dubins_init(orig_array, dest_array, r_min, &path2d);

    cout << orig << " -> " << dest << " => " << path3d.L << "xyz / " << path3d.L_2d << "xy" << endl;
    cout << dubins_path_length(&path2d) << " type=" << dubins_path_type(&path2d) << endl;

    assert(path3d.L == dubins_path_length(&path2d));
    return path3d.L == dubins_path_length(&path2d);
}


void all_dubins_tests()
{
    test_length_flat();
    test_length_low_alt();
    test_medium_alt_SSLS();
    test_length_medium_alt();
    test_length_high_alt();
    test_length_flat();
}