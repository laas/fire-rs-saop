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
import fire_rs.firemodel.propagation as propagation
from fire_rs.geodata.geo_data import TimedPoint


class TestPropagation(unittest.TestCase):

    def setUp(self):
        self.test_area = [[480060.0, 485060.0], [6210074.0, 6215074.0]]
        self.ignition_point = TimedPoint(480060 + 800, 6210074 + 2500, 0)

    def test_propagate(self):
        env = propagation.Environment(self.test_area, wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate_from_points(env, [self.ignition_point], until=3 * 3600)
        # prop.plot(blocking=True)

    def test_propagate_full(self):
        env = propagation.Environment(self.test_area, wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate_from_points(env, [self.ignition_point])
        # prop.plot(blocking=True)
