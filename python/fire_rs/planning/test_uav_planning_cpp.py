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

import matplotlib
matplotlib.use('Agg')  # set backend so that an X display is not required
import unittest
import fire_rs.uav_planning as up
import numpy as np

from fire_rs.geodata.geo_data import GeoData, TimedPoint
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.planning.benchmark import plot_plan, PlanDisplayExtension

# X8 UAS from Porto University
uav = up.UAV(18., 32.*np.pi/180, 0.1)

class TestUAV(unittest.TestCase):

    def setUp(self):
        self.wp1 = up.Waypoint(0, 0, 0, 0)
        self.wp2 = up.Waypoint(4, 0, 0, 0)
        self.test_area = [[480060.0, 490060.0], [6210074.0, 6220074.0]]

    def test_waypoint(self):
        wp1 = up.Waypoint(4, 3, 2, 1)
        self.assertEqual(wp1.x, 4)
        self.assertEqual(wp1.y, 3)
        self.assertEqual(wp1.z, 2)
        self.assertEqual(wp1.dir, 1)

    def test_straight_line_dubins(self):
        self.assertAlmostEqual(4, uav.travel_distance(self.wp1, self.wp2))
        traj = up.Trajectory(up.TrajectoryConfig.build(uav))
        for wp in [self.wp1, self.wp2]:
            traj = traj.with_waypoint_at_end(wp)
            print(traj)
        self.assertAlmostEqual(4, traj.length())

    def test_geodata_conversion(self):
        from fire_rs.geodata.environment import World
        world = World()
        gd = world.get_elevation([[475060.0, 485060], [6200074.0, 6210074]])
        raster = gd.as_cpp_raster()
        print(raster)
        gd2 = GeoData.from_cpp_raster(raster, "elevation_cpp")
        print(gd2[10,10][0])
        self.assertAlmostEqual(gd[10,10][0], gd2[10,10][0])
        self.assertAlmostEqual(gd.x_offset, gd2.x_offset)
        self.assertAlmostEqual(gd.y_offset, gd2.y_offset)
        self.assertEqual(gd.data.shape, gd2.data.shape)
        print(gd2)

    def test_vns_small(self):
        def pr(cpp_raster, blocking=False):  # plots a cpp raster
            return GeoData.from_cpp_raster(cpp_raster, "xx").plot(blocking=blocking)

        from fire_rs.firemodel import propagation
        env = propagation.Environment([[480060.0, 481060.0], [6210074.0, 6211074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate_from_points(env, TimedPoint(480460, 6210374, 0), horizon=3000)

        ignitions = prop.ignitions()
        res = up.make_plan_vns(uav, ignitions.as_cpp_raster(), env.raster.slice('elevation').as_cpp_raster(), 2500, 3000, 150, save_every=5)

        geodatadisplay = GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
        PlanDisplayExtension(None).extend(geodatadisplay)

        plot_plan(res.final_plan(), geodatadisplay, show=True)

        print("durations: ")
        for traj in res.final_plan().trajectories:
            print(traj.duration())

    def test_vns(self):
        def pr(cpp_raster, blocking=False):  # plots a cpp raster
            return GeoData.from_cpp_raster(cpp_raster, "xx").plot(blocking=blocking)

        from fire_rs.firemodel import propagation
        env = propagation.Environment([[480060.0, 485060.0], [6210074.0, 6214074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate_from_points(env, TimedPoint(482060, 6211074, 0), horizon=14000)

        ignitions = prop.ignitions()
        # ax = ignitions.plot(blocking=False)
        elev = env.raster.slice('elevation')
        res = up.make_plan_vns(uav, ignitions.as_cpp_raster(), elev.as_cpp_raster(), 9500, 12000, 1500, save_every=5)

        plan = res.final_plan()

        geodatadisplay = GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
        PlanDisplayExtension(None).extend(geodatadisplay)

        plot_plan(plan, geodatadisplay, show=True)

        for intermediate_plan in res.intermediate_plans:
            geodatadisplay = GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
            PlanDisplayExtension(None).extend(geodatadisplay)
            plot_plan(intermediate_plan, geodatadisplay, show=True)

        print("durations: ")
        for traj in plan.trajectories:
            print(traj.duration())
