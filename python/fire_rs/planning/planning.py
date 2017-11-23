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

import json
import logging
import types
from collections import namedtuple
from typing import Optional, Tuple, Union

import numpy as np

import fire_rs.uav_planning as up
from fire_rs.firemodel.propagation import Environment, FirePropagation
from fire_rs.geodata.geo_data import Area, GeoData, TimedPoint


class Waypoint(namedtuple('Waypoint', 'x, y, z, dir')):
    __slots__ = ()

    def as_cpp(self):
        return up.Waypoint(self.x, self.y, self.z, self.dir)


class UAVConf:

    def __init__(self, max_air_speed: float, max_angular_velocity: float, max_pitch_angle: float,
                 max_flight_time: float):
        self.max_air_speed = max_air_speed  # type: float
        self.max_angular_velocity = max_angular_velocity  # type: float
        self.max_pitch_angle = max_pitch_angle  # type: float
        self.max_flight_time = max_flight_time  # type: float

    @classmethod
    def X8(cls):
        return cls(max_air_speed=18., max_angular_velocity=32./180.*np.pi,
                   max_pitch_angle=6./180.*np.pi, max_flight_time=3000.)

    def as_cpp(self):
        return up.UAV(self.max_air_speed, self.max_angular_velocity, self.max_pitch_angle)

    def __repr__(self):
        return "".join(("UAVConf(max_air_speed=", repr(self.max_air_speed),
                        ", max_angular_velocity=", repr(self.max_angular_velocity),
                        ", max_pitch_angle=", repr(self.max_pitch_angle),
                        ", max_flight_time=", repr(self.max_flight_time), ")"))


class FlightConf:

    def __init__(self, uav: UAVConf, start_time: float, base_waypoint: 'Waypoint',
                 finalbase_waypoint: 'Optional[Waypoint]' = None):
        assert start_time > 0
        self.uav = uav  # type: UAVConf
        self.base_waypoint = base_waypoint  # type: Waypoint
        # type: Optional[Waypoint]
        self.finalbase_waypoint = finalbase_waypoint if finalbase_waypoint else base_waypoint
        self.start_time = start_time  # type: float

    @property
    def max_flight_time(self):
        return self.uav.max_flight_time

    def as_cpp(self):
        return up.TrajectoryConfig(self.uav.as_cpp(), self.base_waypoint.as_cpp(),
                                   self.finalbase_waypoint.as_cpp(), self.start_time,
                                   self.uav.max_flight_time)

    def __repr__(self):
        return "".join(("FlightConf(uav=", repr(self.uav),
                        ", base_waypoint=", repr(self.base_waypoint),
                        ", start_time=", repr(self.start_time), ")"))


class PlanningEnvironment(Environment):
    """Propagation environment with optional discrete elevation only for planning """

    def __init__(self, area, wind_speed, wind_dir, planning_elevation_mode: 'str'='dem',
                 discrete_elevation_interval:'int'=0):
        super().__init__(area, wind_speed, wind_dir)

        self.planning_elevation_mode = planning_elevation_mode
        self.discrete_elevation_interval = discrete_elevation_interval

        elev_planning = None

        if self.planning_elevation_mode == 'flat':
            elev_planning = self.raster.clone(fill_value=0.,
                                              dtype=[('elevation_planning', 'float64')])
        elif self.planning_elevation_mode == 'dem':
            elev_planning = self.raster.clone(data_array=self.raster["elevation"],
                                              dtype=[('elevation_planning', 'float64')])
        elif self.planning_elevation_mode == 'discrete':
            a = self.raster.data['elevation'] + self.discrete_elevation_interval
            b = np.fmod(self.raster.data['elevation'], self.discrete_elevation_interval)
            self.raster.data['elevation'] = a - b
        else:
            elev_planning = self.raster.clone(data_array=self.raster["elevation"],
                                              dtype=[('elevation_planning', 'float64')])
            raise ValueError("".join(["Wrong planning_elevation_mode '",
                                      self.planning_elevation_mode, "'"]))

        self.raster = self.raster.combine(elev_planning)


class Planner:
    def __init__(self, env: 'PlanningEnvironment', firemap: 'GeoData', flights: '[FlightConf]',
                 planning_conf: 'dict'):
        assert "ignition" in firemap.layers
        self._env = env  # type: PlanningEnvironment
        self._firemap = firemap  # type: GeoData
        self._flights = flights[:]  # type: [FlightConf]
        self._observed_firemap = GeoData.zeros_like(self._firemap)  # type: GeoData
        self._cpp_planning_conf = planning_conf  # type: dict

        self._searchresult = None  # type: Optional(up.SearchResult)

    def compute_plan(self, t_start=0.0) -> 'up.SearchResult':
        # Retrieve trajectory configurations in as C++ objects
        cpp_flights = [f.as_cpp() for f in self._flights]

        # Call the C++ library that calculates the plan
        res = up.plan_vns(cpp_flights,
                          self._firemap.as_cpp_raster(),
                          self._env.raster.slice('elevation_planning').as_cpp_raster(),
                          json.dumps(self._cpp_planning_conf))
        self._searchresult = res
        print(res.final_plan().trajectories()[0].segments)
        return res

    @property
    def environment(self) -> 'Environment':
        return self._env

    @property
    def firemap(self) -> 'GeoData':
        return self._firemap

    @firemap.setter
    def firemap(self, value: 'GeoData'):
        assert self._firemap.x_offset == value.x_offset
        assert self._firemap.y_offset == value.y_offset
        assert self._firemap.cell_width == value.cell_width
        assert self._firemap.cell_height == value.cell_height

        self._firemap._firemap = value

    @property
    def flights(self):
        return self._flights

    @property
    def observed_firemap(self):
        return self._observed_firemap

    @property
    def search_result(self):
        return self._searchresult


class PlanningManager:
    def __init__(self, planner: Planner):
        self._planner = planner  # type: Planner

    def update_area_wind(self, wind_velocity, wind_direction):
        self._planner.environment.update_area_wind(wind_velocity, wind_direction)

    def update_firemap(self, firemap: GeoData):
        self._planner.firemap = firemap

    def replan(self):
        """Replan should run Planner.compute_plan with the updated information."""
        pass


if __name__ == '__main__':
    import os
    import json
    from fire_rs.firemodel import propagation
    from fire_rs.geodata.geo_data import TimedPoint

    # Geographic environment (elevation, landcover, wind...)
    area = ((480060.0, 485060.0), (6210074.0, 6215074.0))
    env = PlanningEnvironment(area, wind_speed=5., wind_dir=0., planning_elevation_mode='dem')

    # Fire applied to the previous environment
    ignition_point = TimedPoint(area[0][0] + 1000.0, area[1][0] + 2000.0, 0)
    fire = propagation.propagate_from_points(env, ignition_point, 9000)

    # Configure some flight
    base_wp = Waypoint(area[0][0]+100., area[1][0]+100., 100., 0.)
    start_t = 30 * 60  # 30 minutes after the ignition
    fgconf = FlightConf(UAVConf.X8(), start_t, base_wp)

    # Write down the desired VNS configuration
    conf_vns = {
        "base": {
            "max_restarts": 5,
            "neighborhoods": [
                {"name": "dubins-opt",
                 "max_trials": 10,
                 "generators": [
                     {"name": "MeanOrientationChangeGenerator"},
                     {"name": "RandomOrientationChangeGenerator"},
                     {"name": "FlipOrientationChangeGenerator"}]},
                {"name": "one-insert",
                 "max_trials": 50,
                 "select_arbitrary_trajectory": False,
                 "select_arbitrary_position": False}
            ]
        }
    }

    conf = {
        'min_time': fgconf.start_time,
        'max_time': fgconf.start_time+fgconf.uav.max_flight_time,
        'save_every': 0,
        'save_improvements': False,
        'discrete_elevation_interval': 0,
        'vns': conf_vns['base']
    }
    conf['vns']['configuration_name'] = 'base'

    # Instantiate the planner
    pl = Planner(env, fire.ignitions(), [fgconf], conf)
    pl.compute_plan()
    print(pl.search_result)
