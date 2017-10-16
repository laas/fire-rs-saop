

#include "test_dubins.h"
#include "test_position_manipulation.h"
#include "core/test_reversible_updates.h"

int main() {
    all_dubins_tests();
    all_position_manipulation();
    all_reversible_updates_test();
    return 0;
};