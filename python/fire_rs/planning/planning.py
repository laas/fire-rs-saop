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
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np

import fire_rs.uav_planning as up
import fire_rs.firemapping as fmapping

from fire_rs.firemodel.propagation import Environment, FirePropagation
from fire_rs.geodata.geo_data import GeoData


class Waypoint(namedtuple('Waypoint', 'x, y, z, dir')):
    __slots__ = ()

    def as_cpp(self):
        return up.Waypoint(self.x, self.y, self.z, self.dir)

    @classmethod
    def from_cpp(cls, wp: 'up.Waypoint'):
        return cls(wp.x, wp.y, wp.z, wp.dir)


class UAVConf:

    def __init__(self, max_air_speed: float, max_angular_velocity: float, max_pitch_angle: float,
                 max_flight_time: float):
        self.max_air_speed = max_air_speed  # type: float
        self.max_angular_velocity = max_angular_velocity  # type: float
        self.max_pitch_angle = max_pitch_angle  # type: float
        self.max_flight_time = max_flight_time  # type: float

    @classmethod
    def X8(cls):
        return cls(max_air_speed=17., max_angular_velocity=32. / 180. * np.pi * .5,
                   max_pitch_angle=6. / 180. * np.pi, max_flight_time=3000.)

    @classmethod
    def slow_X8(cls):
        return cls(max_air_speed=17. / 10., max_angular_velocity=32. / 180. * np.pi * .5,
                   max_pitch_angle=6. / 180. * np.pi, max_flight_time=3000. * 10.)

    def as_cpp(self):
        return up.UAV(self.max_air_speed, self.max_angular_velocity, self.max_pitch_angle)

    def __repr__(self):
        return "".join(("UAVConf(max_air_speed=", repr(self.max_air_speed),
                        ", max_angular_velocity=", repr(self.max_angular_velocity),
                        ", max_pitch_angle=", repr(self.max_pitch_angle),
                        ", max_flight_time=", repr(self.max_flight_time), ")"))


class FlightConf:

    def __init__(self, uav: UAVConf, start_time: float, base_waypoint: 'Waypoint',
                 finalbase_waypoint: 'Optional[Waypoint]' = None,
                 wind: 'Tuple[float, float]' = (0., 0.)):
        assert start_time > 0
        self.uav = uav  # type: UAVConf
        self.base_waypoint = base_waypoint  # type: Waypoint
        # type: Optional[Waypoint]
        self.finalbase_waypoint = finalbase_waypoint if finalbase_waypoint else base_waypoint
        self.start_time = start_time  # type: float
        self.wind = wind

    @property
    def max_flight_time(self):
        return self.uav.max_flight_time

    def as_cpp(self):
        return up.TrajectoryConfig(self.uav.as_cpp(), self.base_waypoint.as_cpp(),
                                   self.finalbase_waypoint.as_cpp(), self.start_time,
                                   self.uav.max_flight_time,
                                   up.WindVector(self.wind[0], self.wind[1]))

    def __repr__(self):
        return "".join(("FlightConf(uav=", repr(self.uav),
                        ", base_waypoint=", repr(self.base_waypoint),
                        ", start_time=", repr(self.start_time),
                        ", wind=", repr(self.wind), ")"))


class PlanningEnvironment(Environment):
    """Propagation environment with optional discrete elevation only for planning """

    def __init__(self, area, wind_speed, wind_dir, planning_elevation_mode: 'str' = 'dem',
                 discrete_elevation_interval: 'int' = 0, flat_altitude=0.):
        super().__init__(area, wind_speed, wind_dir)

        self.planning_elevation_mode = planning_elevation_mode
        self.discrete_elevation_interval = discrete_elevation_interval

        elev_planning = None

        if self.planning_elevation_mode == 'flat':
            elev_planning = self.raster.clone(fill_value=flat_altitude,
                                              dtype=[('elevation_planning', 'float64')])
        elif self.planning_elevation_mode == 'dem':
            elev_planning = self.raster.clone(data_array=self.raster["elevation"],
                                              dtype=[('elevation_planning', 'float64')])
        elif self.planning_elevation_mode == 'discrete':
            a = self.raster.data['elevation'] + self.discrete_elevation_interval
            b = np.fmod(self.raster.data['elevation'], self.discrete_elevation_interval)
            elev_planning = self.raster.clone(data_array=a - b,
                                              dtype=[('elevation_planning', 'float64')])
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
        self._env = env  # type: 'PlanningEnvironment'
        self._firemap = firemap  # type: 'GeoData'
        self._flights = flights[:]  # type: 'List[FlightConf]'
        self._planning_conf = planning_conf  # type: 'dict'

        self._searchresult = None  # type: 'Optional(up.SearchResult)'

        self._obs_fire_raster = None  # type: 'GeoData'

    def compute_plan(self) -> 'up.SearchResult':
        # Retrieve trajectory configurations as C++ objects
        cpp_flights = [f.as_cpp() for f in self._flights]

        # Call the C++ library that calculates the plan
        res = up.plan_vns(cpp_flights,
                          self._firemap.as_cpp_raster(),
                          self._env.raster.slice('elevation_planning').as_cpp_raster(),
                          json.dumps(self._planning_conf))
        self._searchresult = res
        return res

    def _replan(self, after_time) -> 'up.SearchResult':
        if not self.search_result:
            logging.warning("Computing an initial plan before replanning")
            self.compute_plan()
        # Call the C++ library that calculates the plan
        res = up.replan_vns(self.search_result, after_time,
                            self._firemap.as_cpp_raster(),
                            self._env.raster.slice('elevation_planning').as_cpp_raster(),
                            json.dumps(self._planning_conf))
        self._searchresult = res
        return res

    def replan_after_time(self, after_time) -> 'up.SearchResult':
        return self._replan(after_time)

    def expected_ignited_positions(self, time_window=None):
        """Get the percieved positions where the fire is suposed to be."""
        if time_window:
            tw_cpp = up.TimeWindow(time_window[0], time_window[1])
            return [o.as_tuple() for o in self._searchresult.final_plan().observations(tw_cpp)]
        else:
            return [o.as_tuple() for o in self._searchresult.final_plan().observations()]

    def expected_observed_positions(self, time_window=None):
        """Get all the positions percieved the UAV camera.
        Arguments:
            time_window: If None (default) all the trajectory segments"""
        if time_window:
            tw_cpp = up.TimeWindow(time_window[0], time_window[1])
            return [o.as_tuple() for o in self._searchresult.final_plan().view_trace(tw_cpp)]
        else:
            return [o.as_tuple() for o in self._searchresult.final_plan().view_trace()]

    def update_firemap(self, firemap: GeoData):
        self.firemap = firemap

    @property
    def environment(self) -> 'Environment':
        return self._env

    @property
    def time_window(self):
        if self._searchresult:
            return self._searchresult.final_plan().time_window.as_tuple()
        else:
            return self.planning_conf['min_time'], self.planning_conf['max_time']

    @time_window.setter
    def time_window(self, value):
        pass

    @property
    def firemap(self) -> 'GeoData':
        return self._firemap

    @firemap.setter
    def firemap(self, value: 'GeoData'):
        assert self._firemap.x_offset == value.x_offset
        assert self._firemap.y_offset == value.y_offset
        assert self._firemap.cell_width == value.cell_width
        assert self._firemap.cell_height == value.cell_height

        self._firemap = value

    @property
    def flights(self):
        return self._flights

    def expected_ignited_map(self, time_window=None, layer_name='observed') -> 'GeoData':
        observed_firemap = self._firemap.clone(fill_value=np.nan, dtype=[(layer_name, 'float64')])
        for (p, t) in self.expected_ignited_positions(time_window):
            array_pos = observed_firemap.array_index(p)
            observed_firemap[layer_name][array_pos] = t
        return observed_firemap

    def expected_observed_map(self, time_window=None, layer_name='observed') -> 'GeoData':
        """Get a GeoData with the time of all percieved cells by the UAV camera."""
        cells = self.expected_observed_positions(time_window)
        map = self._firemap.clone(fill_value=np.nan, dtype=[(layer_name, 'float64')])
        for (p, t) in self.expected_observed_positions(time_window):
            array_pos = map.array_index(p)
            map[layer_name][array_pos] = t
        return map

    @property
    def search_result(self):
        return self._searchresult

    @property
    def planning_conf(self) -> 'dict':
        return self._planning_conf


class FireMapper:
    """Easy interface to the C++ firemapper module"""

    def __init__(self, env: 'Environment', true_fire: 'GeoData', true_fire_layer='ignition',
                 output_fire_layer='ignition', output_observed_layer='observed'):
        elevation_draster = env.raster.as_cpp_raster('elevation')
        ignition_draster = true_fire.as_cpp_raster(true_fire_layer)

        self.output_fire_layer = output_fire_layer
        self.output_observed_layer = output_observed_layer

        self._gfmapper = fmapping.GhostFireMapper(up.FireData(ignition_draster, elevation_draster))

    @property
    def firemap(self) -> 'GeoData':
        return GeoData.from_cpp_raster(self._gfmapper.firemap, self.output_fire_layer)

    @property
    def observed(self) -> 'GeoData':
        return GeoData.from_cpp_raster(self._gfmapper.observed, self.output_fire_layer)

    def observe(self, executed_path: 'Sequence[List[up.Waypoint], List[float]]',
                uav: 'UAVConf'):
        self._gfmapper.observe(executed_path[0], executed_path[1], uav.as_cpp())
