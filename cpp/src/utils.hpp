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

#ifndef PLANNING_CPP_DEBUG_H
#define PLANNING_CPP_DEBUG_H


#include <stdio.h>
#include <signal.h>
#include <stdio.h>
#include <signal.h>
#include <execinfo.h>
#include <wait.h>
#include <unistd.h>
#include <cstdlib>
#include <cassert>

 namespace SAOP {

     void print_trace();

     double drand(double min, double max);

     size_t rand(size_t min, size_t non_inclusive_max);

     double positive_modulo(double left, double right);

}

/** Macro that test equality of to floating point number, disregarding rounding errors. */
#define ALMOST_EQUAL(x, y) (fabs((double) (x) - (double) (y)) < 0.000001)

/** Macro that test whether x >= y with tolerance to rounding errors. */
#define ALMOST_GREATER_EQUAL(x, y) ((double) (x) >= ((double) (y) - 0.000001))

/** Macro that test whether x >= y with tolerance to rounding errors. */
#define ALMOST_LESSER_EQUAL(x, y) ((double) (x) <= ((double) (y) + 0.000001))


// http://cnicholson.net/2009/02/stupid-c-tricks-adventures-in-assert/
#ifdef DEBUG
#define ASSERT(test)                  \
do { if(!(test)) {                         \
  fprintf(stderr, "Failed assert\n"); \
  print_trace();                      \
}                                     \
assert(test); } while(0)
#else
#define ASSERT(test) \
        do { (void)sizeof(test); } while(0)
#endif

#endif //PLANNING_CPP_DEBUG_H
