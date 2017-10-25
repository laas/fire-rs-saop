
#include "test_dubins.h"
#include "test_position_manipulation.h"
#include "core/test_reversible_updates.h"
#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test;

test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/[] )
{
    auto dubins_ts = dubins_test_suite();
    auto position_manipulation_ts = position_manipulation_test_suite();
    auto reversible_updates_ts = reversible_updates_test_suite();

    framework::master_test_suite().add(dubins_ts);
    framework::master_test_suite().add(position_manipulation_ts);
    framework::master_test_suite().add(reversible_updates_ts);

    return nullptr;
}
