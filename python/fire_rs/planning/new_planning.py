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

import copy
import json
import logging
import typing as ty
from collections.abc import Mapping, Sequence
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

import fire_rs.uav_planning as up

from fire_rs.geodata.geo_data import GeoData
from fire_rs.geodata.wildfire import rate_of_spread_map

logger = logging.getLogger(__name__)


class VNSConfDB(Mapping):
    """VNS configuration DB for SAOP Planner.

    This class provides a unique interface to fetch VNS configuration
    dictionaries, initialized with a json file, a json str or a dictionary
    """

    def __init__(self, vns_json: 'Dict[str, dict]'):
        if isinstance(vns_json, str):
            self._vns_confs = json.loads(vns_json)  # type: 'dict'
        if isinstance(vns_json, dict):
            self._vns_confs = vns_json  # type: 'dict'
        else:
            self._vns_confs = json.load(vns_json)  # type: 'dict'

        for k in self._vns_confs.keys():
            self._vns_confs[k]["configuration_name"] = k

    def __getitem__(self, item) -> 'Dict':
        return copy.deepcopy(self._vns_confs[item])

    def __iter__(self):
        return iter(self._vns_confs)

    def __len__(self):
        return len(self._vns_confs)

    def __repr__(self):
        return '{self.__class__.__name__}({self._vns_confs})'.format(self=self)

    @classmethod
    def demo_db(cls):
        demo_vns_key = 'demo'
        demo_vns = {
            "max_time": 30.,
            "neighborhoods": [
                {"name": "dubins-opt",
                 "max_trials": 100,
                 "generators": [
                     {"name": "MeanOrientationChangeGenerator"},
                     {"name": "RandomOrientationChangeGenerator"},
                     {"name": "FlipOrientationChangeGenerator"}]},
                {"name": "one-insert",
                 "max_trials": 50,
                 "select_arbitrary_trajectory": True,
                 "select_arbitrary_position": False},
            ]
        }
        full_vns_key = 'full'
        full_vns = {
            "max_time": 30.0,
            "neighborhoods": [
                {"name": "dubins-opt",
                 "max_trials": 100,
                 "generators": [
                     {"name": "MeanOrientationChangeGenerator"},
                     {"name": "RandomOrientationChangeGenerator"},
                     {"name": "FlipOrientationChangeGenerator"}]},
                {"name": "one-insert",
                 "max_trials": 50,
                 "select_arbitrary_trajectory": False,
                 "select_arbitrary_position": False},
                {"name": "one-insert",
                 "max_trials": 200,
                 "select_arbitrary_trajectory": True,
                 "select_arbitrary_position": False},
                {"name": "one-insert",
                 "max_trials": 200,
                 "select_arbitrary_trajectory": True,
                 "select_arbitrary_position": True}
            ]
        }
        return cls({demo_vns_key: demo_vns, full_vns_key: full_vns})


class SAOPPlannerConf(Mapping):
    """SAOP Planner configuration builder"""

    def __init__(self, vnsconf: 'ty.Mapping',
                 max_planning_time: 'Optional[float]' = None, save_improvements: 'bool' = False,
                 save_every: 'int' = 0):
        self._conf = {
            'save_improvements': save_improvements,
            'save_every': save_every,
            'vns': vnsconf
        }
        if max_planning_time:
            self._conf['vns']['max_time'] = max_planning_time

    @classmethod
    def from_dict(cls, d: 't.Mapping'):
        cls(d['vns'], d['vns'].get('max_time', None),
            d['save_improvements'], d['save_every'])

    def to_json(self) -> 'str':
        return json.dumps(self._conf)

    def __getitem__(self, item) -> 'Dict':
        return self._conf[item]

    def __iter__(self):
        return iter(self._conf)

    def __len__(self):
        return len(self._conf)

    def __str__(self):
        return 'self._conf'.format(self=self)


Waypoint = up.Waypoint

UAV = up.UAV

TrajectoryConfig = up.TrajectoryConfig

TrajectoryManeuver = up.TrajectoryManeuver

Trajectory = up.Trajectory

TimeWindow = up.TimeWindow

FireData = up.FireData

WindVector = up.WindVector

Plan = up.Plan


class UAVModels:

    @staticmethod
    def get(name: str):
        """Get a UAV model by its name. Only x8 units are supported"""
        return UAVModels.x8(name.split("-")[1])

    @property
    def x8_02(self):
        return UAVModels.x8("02")

    @property
    def x8_06(self):
        return UAVModels.x8("06")

    @staticmethod
    def x8(unit: str = "00"):
        return UAV("-".join(("x8", unit)), 17., 20. / 180. * np.pi, 6. / 180. * np.pi)

    @staticmethod
    def slow_x8(unit: str = "00"):
        return UAV("-".join(("x8", unit)), 17. / 10., 32. / 180. * np.pi, 6. / 180. * np.pi)


def make_fire_data(fire_map: GeoData, elevation_map: GeoData, fire_map_layer: str = 'ignition',
                   elevation_map_layer: str = 'elevation'):
    return FireData(fire_map.as_cpp_raster(fire_map_layer),
                    elevation_map.as_cpp_raster(elevation_map_layer))


def make_utility_map(firemap: GeoData, flight_window: TimeWindow = TimeWindow(-np.inf, np.inf),
                     layer="ignition", output_layer="utility") -> GeoData:
    """Compute a utility map from a wildfire map"""
    gradient = rate_of_spread_map(firemap, layer=layer, output_layer="ros")
    grad_array = gradient["ros"]
    grad_array[grad_array >= np.inf] = np.NaN

    utility = 1 - (grad_array - np.nanmin(grad_array)) / (
            np.nanmax(grad_array) - np.nanmin(grad_array))
    utility[np.isnan(utility)] = 0.
    utility[(firemap[layer] < flight_window.start) | (firemap[layer] > flight_window.end)] = 0.

    return firemap.clone(data_array=utility, dtype=[(output_layer, 'float64')])


class Planner:
    """Make a multi-UAV observation mission form an initial plan or improve an existing plan"""

    def __init__(self, initial_plan: Plan, vns_conf: ty.Mapping):
        """Initialize a planner object from a previous plan (can be an initial plan)"""
        self.current_plan = initial_plan
        self.vns_conf = vns_conf

        self._save_improvements = False
        self._save_every = 0

    def update_fire_data(self, fire_map: GeoData, elevation_map: GeoData,
                         fire_map_layer: str = 'ignition', elevation_map_layer: str = 'elevation'):
        """Update wildfire and elevation maps of the current plan"""
        self.current_plan.firedata(
            make_fire_data(fire_map, elevation_map, fire_map_layer, elevation_map_layer))

    def compute_plan(self, planning_duration: float, after_time=.0,
                     frozen_trajectories: ty.Sequence[str] = ()) -> up.SearchResult:
        """Improve the current plan using a VNS planner"""
        planning_conf = SAOPPlannerConf(self.vns_conf, planning_duration, self._save_improvements,
                                        self._save_every)

        # Call the C++ library that calculates the plan
        res = up.plan_vns(self.current_plan, json.dumps(dict(planning_conf)), after_time,
                          frozen_trajectories)
        self.current_plan = res.final_plan()
        return res


if __name__ == "__main__":
    tc = TrajectoryConfig("name", UAVModels.x8("02"),
                          Waypoint(480000. + 1000, 6210000. + 1000, 0., 0.),
                          Waypoint(480000. + 1000, 6210000. + 1000, 0., 0.), 0., 3000.,
                          WindVector(0., 0.))

    from fire_rs.firemodel.propagation import Environment

    wind = (10., 0.)
    area = ((480000.0, 485000.0), (6210000.0, 6215000.0))
    env = Environment(area, wind_speed=wind[0], wind_dir=wind[1])
    el = env.raster.as_cpp_raster("elevation")
    tw = TimeWindow(0., 1542362677.0 + 100000.0)
    p = Plan("nombre", [tc], el, el, tw)

    print(p)
