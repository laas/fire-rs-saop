#!/usr/bin/env python3

#  Copyright (c) 2019, CNRS-LAAS
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import typing as ty
import threading

import numpy as np

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header

import fire_rs.simulation.fakefiremapper
import fire_rs.planning.new_planning
from fire_rs.geodata.geo_data import GeoData

import serialization
from supersaop.msg import Timed2DPointStamped, PropagateCmd, MeanWindStamped, VehicleState, \
    WildfireMap, ElevationMap


class FakeMapperNode:
    def __init__(self, known_uavs: ty.Sequence[str]):
        self._known_uavs = known_uavs

        # type: ty.Mapping[str, rospy.Publisher]
        self.sub_uav_state_dict = {uav: rospy.Subscriber(
            "/".join(("uavs", serialization.ros_name_for(uav), 'state')), VehicleState,
            callback=self._on_vehicle_state, callback_args=uav, queue_size=10) for uav in
            self._known_uavs}

        # type: rospy.Subscriber
        self.sub_elevation_map = rospy.Subscriber("elevation", ElevationMap, self._on_elevation_map,
                                                  queue_size=10)

        # type: rospy.Subscriber
        self.sub_real_wildfire_map = rospy.Subscriber("wildfire_real", ElevationMap,
                                                      self._on_real_wildfire_map,
                                                      queue_size=10)

        # type: ty.Mapping[str, rospy.Publisher]
        self.pub_dict_firemap = {
            uav: rospy.Publisher(
                "/".join(("uavs", serialization.ros_name_for(uav), 'wildfire_observed')),
                WildfireMap, queue_size=10) for uav in self._known_uavs}

        self.location_history = {}  # type: ty.MutableMapping[str, ty.MutableSequence[ty.Any]]
        self.elevation_map = None
        self.real_wildfire_map = None
        self.firemapper = None  # type: ty.Optional[fire_rs.simulation.fakefiremapper.FakeFireMapper]

        self.lock = threading.Lock()

        rospy.init_node("fake_mapper")
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

    def _on_vehicle_state(self, msg: VehicleState, uav: str):
        with self.lock:
            if uav not in self.location_history:
                self.location_history[uav] = []
            self.location_history[uav].append(
                (fire_rs.planning.new_planning.Waypoint(msg.position.x,
                                                        msg.position.y,
                                                        msg.position.z,
                                                        msg.orientation.phi),
                 msg.header.stamp.to_sec()))

    def _on_elevation_map(self, elevation: ElevationMap):
        self.elevation_map = serialization.geodata_from_raster_msg(elevation.raster, "elevation")
        self._reset_firemapper()

    def _on_real_wildfire_map(self, wildfire: ElevationMap):
        self.real_wildfire_map = serialization.geodata_from_raster_msg(wildfire.raster, "ignition")
        self._reset_firemapper()

    def _reset_firemapper(self):
        if self.elevation_map is not None and self.real_wildfire_map is not None:
            self.firemapper = fire_rs.simulation.fakefiremapper.FakeFireMapper(
                self.elevation_map, self.real_wildfire_map)
        else:
            self.firemapper = None

    def do_mapping(self):
        if self.firemapper is not None:
            for uav_str, wp_list in self.location_history.items():
                self.firemapper.observe(list(zip(*wp_list)), fire_rs.planning.new_planning.UAVModels.get(uav_str))
                # obs_fire_map = GeoData.from_cpp_raster(self.firemapper.firemap, "ignition",
                #                                        projection=self.elevation_map.projection)

                wf_msg = WildfireMap(header=rospy.Header(stamp=rospy.Time.now()),
                                     raster=serialization.raster_msg_from_geodata(self.firemapper.firemap, 'ignition', invert=True))
                self.pub_dict_firemap[uav_str].publish(wf_msg)


if __name__ == '__main__':
    try:
        if rospy.has_param("known_uavs"):
            known_uavs = rospy.get_param("known_uavs")
        else:
            known_uavs = ["x8-02", "x8-06"]
        mapper_node = FakeMapperNode(known_uavs)

        r = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            mapper_node.do_mapping()
            r.sleep()

    except rospy.ROSInterruptException:
        pass
