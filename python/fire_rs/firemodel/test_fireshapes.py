# Copyright (c) 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import unittest

import numpy as np

import fire_rs.firemodel.fireshapes as fireshapes


class TestFireShapesConsistency(unittest.TestCase):

    def test_decreasing_speed(self):
        """Make sure the rate of spread is strictly decreasing.

        (as we move away from the main direction).
        """
        for target_angle in np.linspace(0.01, np.pi, num=100):
            prev_ros = 999999
            for windspeed in np.linspace(0, 100, num=100):
                shape = fireshapes.get_fire_shape(windspeed, 0, 10)
                ros = shape.speed(target_angle)
                self.assertLess(ros, prev_ros, 'Speed increases when going backward')
                prev_ros = ros

    def test_circular_when_no_wind(self):
        """Check that we get a circular shape when there is no wind."""
        shape = fireshapes.get_fire_shape(0, 0, 10)
        for target_angle in np.linspace(0, 2*np.pi):
            self.assertAlmostEqual(10, shape.speed(target_angle))

    def test_main_ros(self):
        """Make sure the ros in the main direction is the good one."""
        for ros in np.linspace(0.01, 100, num=100):
            for windspeed in np.linspace(0, 100, num=100):
                shape = fireshapes.get_fire_shape(windspeed, 0, ros)
                self.assertAlmostEqual(ros, shape.speed(0))


if __name__ == '__main__':
    unittest.main()
