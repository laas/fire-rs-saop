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

from fire_rs.planning.uav import UAV


class TestUAV(unittest.TestCase):

    def setUp(self):
        self.uav = UAV(10, 20, 2, 3)

    def test_uav_speed(self):
        self.assertAlmostEqual(5, self.uav.travel_time((0, 0), (100, 0)))
        self.assertAlmostEqual(5, self.uav.travel_time((0, 0), (0, 100)))
        self.assertAlmostEqual(5, self.uav.travel_time((0, 0, 0), (0, 100, 1)))
        self.assertAlmostEqual(5, self.uav.travel_time((0, 0, 4), (0, 100, 1)))

    def test_uav_climb(self):
        self.assertAlmostEqual(1, self.uav.travel_time((0, 0, 0), (0, 0, 2)))
        self.assertAlmostEqual(45, self.uav.travel_time((0, 0, 0), (0, 100, 90)))

    def test_uav_descent(self):
        self.assertAlmostEqual(1, self.uav.travel_time((0, 0, 100), (0, 0, 97)))
        self.assertAlmostEqual(30, self.uav.travel_time((0, 0, 90), (0, 100, 0)))