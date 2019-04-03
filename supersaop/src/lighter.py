#!/usr/bin/env python3

# Copyright (c) 2018, CNRS-LAAS
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

import datetime
import os.path

import numpy as np
import rospy
from std_msgs.msg import Header
from supersaop.msg import MeanWindStamped, PropagateCmd, Timed2DPointStamped, WildfireMap

import fire_rs.firemodel.propagation
import fire_rs.geodata.display
import fire_rs.geodata.wildfire
import fire_rs.simulation.wildfire
import serialization


class LighterNode:
    def __init__(self, workdir: str):
        self.workdir = workdir
        rospy.init_node("wildfire_starter")
        rospy.loginfo("Starting {}".format(self.__class__.__name__))
        self.pub_w = rospy.Publisher("wildfire_point", Timed2DPointStamped, queue_size=1,
                                     tcp_nodelay=True)
        self.pub_wildfire_obs = rospy.Publisher("wildfire_observed", WildfireMap, queue_size=1,
                                                tcp_nodelay=True)
        self.pub_wind = rospy.Publisher("mean_wind", MeanWindStamped, queue_size=1,
                                        tcp_nodelay=True)

        self.pub_wildfire_real = rospy.Publisher("real_wildfire", WildfireMap, queue_size=1,
                                                 tcp_nodelay=True, latch=True)

        self.pub_p = rospy.Publisher("propagate", PropagateCmd, queue_size=1, tcp_nodelay=True)

    def notify_alarm_point(self, position, timestamp):
        if callable(timestamp):
            timestamp = timestamp()
        fire_pos = Timed2DPointStamped(header=Header(stamp=rospy.Time.now()), x=position[0],
                                       y=position[1], t=timestamp)

        self.pub_w.publish(fire_pos)
        rospy.loginfo("Notify alarm point")
        rospy.loginfo(fire_pos)

    def save_dem_of_firemap(self, environment: fire_rs.firemodel.propagation.Environment):
        environment.raster.write_to_file(os.path.join(self.workdir, "elevation.tif"), "elevation")

    def notify_alarm_map(self, firemap: fire_rs.geodata.geo_data.GeoData):
        if callable(firemap):
            firemap = firemap()
        wildfire_message = WildfireMap(header=rospy.Header(stamp=rospy.Time.now()),
                                       raster=serialization.raster_msg_from_geodata(firemap,
                                                                                    layer="ignition"))
        firemap.data["ignition"][firemap.data["ignition"] == np.inf] = 0
        firemap.data["ignition"][firemap.data["ignition"].nonzero()] = 65535
        firemap.write_to_image_file(os.path.join(self.workdir, "fire.png"), "ignition")

        self.pub_wildfire_obs.publish(wildfire_message)
        rospy.loginfo("Notify alarm map")
        rospy.loginfo(wildfire_message)

    def propagate_and_set_fire_front(self, origin, origin_time: rospy.Time,
                                     perimeter_time: rospy.Time,
                                     environment: fire_rs.firemodel.propagation.Environment):
        origin_pos = Timed2DPointStamped(header=Header(stamp=rospy.Time.now()), x=origin[0],
                                         y=origin[1], t=origin_time)

        fireprop = fire_rs.firemodel.propagation.propagate_from_points(
            environment, fire_rs.firemodel.propagation.TimedPoint(*origin, origin_time.to_sec()))

        firemap = fireprop.ignitions()

        array, cells, contours = fire_rs.geodata.wildfire._compute_perimeter(
            firemap, perimeter_time.to_sec())

        # Publish ignition points
        # initial alarm
        self.pub_w.publish(origin_pos)
        self.pub_w.publish(origin_pos)
        rospy.loginfo(origin_pos)

        wildfire_message = WildfireMap(header=rospy.Header(stamp=rospy.Time.now()),
                                       raster=serialization.raster_msg_from_geodata(
                                           firemap.clone(data_array=array,
                                                         dtype=firemap.data.dtype), 'ignition'))
        self.pub_wildfire_obs.publish(wildfire_message)
        rospy.loginfo(wildfire_message)

    def notify_wind(self, wind_speed, wind_direction):
        wind = MeanWindStamped(header=Header(stamp=rospy.Time.now()), speed=wind_speed,
                               direction=wind_direction)

        self.pub_wind.publish(wind)
        rospy.loginfo("Notify wind")
        rospy.loginfo(wind)

    def publish_real_fire(self, real_fire: fire_rs.simulation.wildfire.RealWildfire):
        current_fire_msg = WildfireMap(header=rospy.Header(stamp=rospy.Time.now()),
                                       raster=serialization.raster_msg_from_geodata(
                                           real_fire.fire_map, layer="ignition"))
        self.pub_wildfire_real.publish(current_fire_msg)
        rospy.loginfo("Publish real fire")
        rospy.loginfo(current_fire_msg)

        g = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(real_fire.fire_map)
        g.draw_ignition_shade(with_colorbar=True)
        g.figure.savefig("/home/rbailonr/fuego_ref.png")
        g.close()

    def propagate(self):
        pcmd = PropagateCmd()
        self.pub_p.publish(pcmd)
        rospy.loginfo("Propagate")


if __name__ == '__main__':
    try:
        w_starter = LighterNode("/home/rbailonr/")
        half_hour = rospy.Duration(secs=30 * 60)
        one_hour = rospy.Duration(secs=1 * 60 * 60)
        two_hours = rospy.Duration(secs=2 * 60 * 60)
        five_hours = rospy.Duration(secs=5 * 60 * 60)
        ten_hours = rospy.Duration(secs=10 * 60 * 60)

        actions = []

        location = rospy.get_param("~location", None)
        print(location)

        area = rospy.get_param("area")

        world_paths = None
        has_dem_dir = rospy.has_param("dem_dir")
        has_landcover_dir = rospy.has_param("landcover_dir")
        has_wind_dir = rospy.has_param("wind_dir")

        if has_dem_dir and has_landcover_dir and has_wind_dir:
            # Either all are defined ...
            world_paths = {}
            world_paths["elevation_path"] = rospy.get_param("dem_dir")
            world_paths["landcover_path"] = rospy.get_param("landcover_dir")
            world_paths["wind_path"] = rospy.get_param("wind_dir")
        else:
            # ... or none of them
            if has_dem_dir or has_landcover_dir or has_wind_dir:
                # This is an error
                rospy.logerr("dem_dir, landcover_dir and wind_dir have to be defined all toghether "
                             "or not be defined at all.")
                rospy.signal_shutdown("Geographic data directories are not well defined")
            else:
                # data folders are implicitely defined
                pass

        if location == "lab_1":
            environment = fire_rs.firemodel.propagation.Environment(
                area, 2, 1 * np.pi / 4, fire_rs.geodata.environment.World(
                    **world_paths,
                    landcover_to_fuel_remap=fire_rs.geodata.environment.EVERYTHING_FUELMODEL_REMAP))
            actions = [
                (w_starter.notify_alarm_point,
                 ((2776829.0, 2212180.0), rospy.Time.now() - ten_hours)),
                (w_starter.propagate_and_set_fire_front,
                 ((2776829.0, 2212180.0),
                  rospy.Time.now() - two_hours,
                  rospy.Time.now(),
                  environment)),
                (w_starter.notify_wind, (3, 1 * 3 * np.pi / 4)),
                (w_starter.propagate, None)
            ]
        elif location == "lab_2_large":
            environment = fire_rs.firemodel.propagation.Environment(
                area, 2, 1 * np.pi / 4, fire_rs.geodata.environment.World(
                    **world_paths,
                    landcover_to_fuel_remap=fire_rs.geodata.environment.EVERYTHING_FUELMODEL_REMAP))
            rw = fire_rs.simulation.wildfire.RealWildfire(
                datetime.datetime.fromtimestamp((rospy.Time.now() - one_hour*5).to_sec()),
                environment)

            ignitions = [np.array([area[0][0] + (area[0][1] - area[0][0]) * 0.25,
                                   area[1][0] + (area[1][1] - area[1][0]) * 0.25]),
                         np.array([area[0][0] + (area[0][1] - area[0][0]) * 0.75,
                                   area[1][0] + (area[1][1] - area[1][0]) * 0.75])
                         ]

            ignitions_cell = [rw.fire_map.array_index(p) for p in ignitions]

            actions = [
                (w_starter.save_dem_of_firemap, ((environment,))),
                (rw.ignite, (ignitions[0],)),
                (rw.propagate, (datetime.timedelta(minutes=120.),)),
                (rw.change_wind, (3, np.pi / 4)),
                (rw.propagate, (datetime.timedelta(minutes=120.),)),
                (rw.change_wind, (3, np.pi / 2)),
                # (rw.ignite, (ignitions[1],)),
                (rw.propagate, (datetime.timedelta(minutes=60.),)),
                (rw.change_wind, (3, 0.)),
                (rw.propagate, (datetime.timedelta(minutes=60.),)),
                # (rw.change_wind, (3, np.pi / 4)),
                # (rw.propagate, (datetime.timedelta(minutes=5000.),)),
                # (rw.change_wind, (3, np.pi / 2)),
                # (rw.propagate, (datetime.timedelta(minutes=35.),)),
                (w_starter.notify_wind, (3., 0.)),
                (w_starter.notify_alarm_point,
                 (ignitions[0],
                  lambda: rospy.Time.from_sec(rw.fire_map["ignition"][ignitions_cell[0]]))),
                (w_starter.notify_alarm_map,
                 (lambda: rw.perimeter(rospy.Time.now().to_sec() - 120 * 60).geodata,)),
                (w_starter.notify_alarm_map,
                 (lambda: rw.perimeter(rospy.Time.now().to_sec()-60*60).geodata,)),
                (w_starter.publish_real_fire, (rw,)),
                (w_starter.propagate, None)
            ]

        # elif location == "porto":
        #     actions = [
        #         (w_starter.notify_alarm_point,
        #          ((2776829.0, 2212180.0), rospy.Time.now() - ten_hours)),
        #         (w_starter.notify_alarm_point,
        #          ((2776829.0, 2212180.0), rospy.Time.now() - ten_hours)),
        #         (w_starter.propagate, None)]
        #
        # elif location == "porto_3":
        #     # Two fires around LIPA, one over the north, and the other on the south
        #     actions = [
        #         # North fire
        #         (w_starter.notify_alarm_point,
        #          ((2776958.0, 2212499.0), rospy.Time.now() - one_hour * 3)),
        #         (w_starter.notify_alarm_point,
        #          ((2776958.0, 2212499.0), rospy.Time.now() - one_hour * 3)),
        #
        #         # South fire
        #         (w_starter.notify_alarm_point,
        #          ((2776704.0, 2211865.0), rospy.Time.now() - two_hours)),
        #
        #         (w_starter.notify_wind, (1, 1 * np.pi / 4)),
        #
        #         (w_starter.propagate, None)]
        #
        # elif location == "porto_2":
        #     # Bigger fire around LIPA directed to the north-east
        #     actions = [
        #         (w_starter.notify_alarm_point,
        #          ((2776829.0, 2212180.0), rospy.Time.now() - five_hours)),
        #         (w_starter.notify_alarm_point,
        #          ((2776829.0, 2212180.0), rospy.Time.now() - five_hours)),
        #         (w_starter.notify_wind, (1, 1 * np.pi / 4)),
        #         (w_starter.propagate, None)]
        # elif location == "porto_1":
        #     # Small fire around LIPA
        #     actions = [
        #         (w_starter.notify_alarm_point,
        #          ((2776829.0, 2212180.0), rospy.Time.now() - five_hours)),
        #         (w_starter.notify_alarm_point,
        #          ((2776829.0, 2212180.0), rospy.Time.now() - five_hours)),
        #         (w_starter.propagate, None)]
        # else:
        #     # Default actions
        #     actions = [
        #         (
        #         w_starter.notify_alarm_point, ((483060.0, 6214074.0), rospy.Time.now() - one_hour)),
        #         (w_starter.propagate, None),
        #         (w_starter.notify_alarm_point,
        #          ((482060.0, 6213074.0), rospy.Time.now() - two_hours)),
        #         (w_starter.propagate, None)]

        r = rospy.Rate(1 / 1.)

        while actions:
            a = actions.pop(0)
            print(a)
            if a[1] is not None:
                a[0](*a[1])
            else:
                a[0]()
            r.sleep()

    except rospy.ROSInterruptException:
        pass
