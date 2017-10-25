#ifndef PLANNING_CPP_DEBUG_H
#define PLANNING_CPP_DEBUG_H


#include <stdio.h>
#include <signal.h>
#include <stdio.h>
#include <signal.h>
#include <execinfo.h>
#include <wait.h>
#include <zconf.h>
#include <cstdlib>
#include <cassert>

void print_trace();
double drand(double min, double max);
size_t rand(size_t min, size_t non_inclusive_max);
double positive_modulo(double left, double right);

/** Macro that test equality of to floating point number, disregarding rounding errors. */
#define ALMOST_EQUAL(x, y) (fabs((double) x - (double) y) < 0.000001)

/** Macro that test whether x >= y with tolerance to rounding errors. */
#define ALMOST_GREATER_EQUAL(x, y) ((double) x >= ((double) y - 0.000001))

/** Macro that test whether x >= y with tolerance to rounding errors. */
#define ALMOST_LESSER_EQUAL(x, y) ((double) x <= ((double) y + 0.000001))

#define ASSERT(test)                  \
if(!(test)) {                         \
  fprintf(stderr, "Failed assert\n"); \
  print_trace();                      \
}                                     \
assert(test);

#endif //PLANNING_CPP_DEBUG_H
