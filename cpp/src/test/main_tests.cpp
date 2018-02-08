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

#include "test_dubins.hpp"
#include "test_position_manipulation.hpp"
#include "core/test_reversible_updates.hpp"
#include <boost/test/included/unit_test.hpp>

using namespace boost::unit_test;

test_suite* init_unit_test_suite(int /*argc*/, char* /*argv*/[]) {
    auto dubins_ts = SAOP::Test::dubins_test_suite();
    auto position_manipulation_ts = SAOP::Test::position_manipulation_test_suite();
    auto reversible_updates_ts = SAOP::Test::reversible_updates_test_suite();

    framework::master_test_suite().add(dubins_ts);
    framework::master_test_suite().add(position_manipulation_ts);
    framework::master_test_suite().add(reversible_updates_ts);

    return nullptr;

}