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

# import pandas to workaround issue when pandas is imported after some other libraries
# https://github.com/pandas-dev/pandas/issues/23040
import pandas

import datetime
import logging
import threading
import uuid

import numpy as np
import matplotlib

matplotlib.use('Gtk3Agg')

import rospy
from std_msgs.msg import String

from supersaop.msg import ElevationMap, PredictedWildfireMap, PropagateCmd, Raster, RasterMetaData, \
    WildfireMap, MeanWindStamped, SurfaceWindMap, Timed2DPointStamped

from fire_rs.geodata.display import GeoData, GeoDataDisplay
from fire_rs.monitoring.supersaop import SituationAssessment

import serialization


class SituationAssessmentNode:

    def __init__(self, area, world_paths=None):
        rospy.init_node('situation_assessment')
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        self.sa = SituationAssessment(area, logging.getLogger(__name__), world_paths=world_paths)

        # Lock from new observations while doing predictions
        self.sa_lock = threading.Lock()

        self.last_wildfire_update = datetime.datetime.min
        self.last_elevation_timestamp = datetime.datetime.min

        self.horizon = rospy.Duration(secs=1 * 60 * 60)

        self.pub_wildfire_pred = rospy.Publisher('wildfire_prediction',
                                                 PredictedWildfireMap,
                                                 queue_size=10, latch=True)
        self.pub_wildfire_current = rospy.Publisher('wildfire', WildfireMap,
                                                    queue_size=10, latch=True)
        # self.pub_wildfire_real = rospy.Publisher('wildfire_real', WildfireMap,
        #                                          queue_size=10, latch=True)
        self.pub_elevation = rospy.Publisher('elevation', ElevationMap,
                                             queue_size=10, latch=True)
        # self.pub_surface_wind = rospy.Publisher('surface_wind', SurfaceWindMap,
        #                                         queue_size=10, latch=True)

        self.uavs = ('x8-02', 'x8-06')

        self.sub_firemap_obs_dict = {
            uav: rospy.Subscriber(
                "/".join(("uavs", serialization.ros_name_for(uav), 'wildfire_observed')),
                WildfireMap, callback=self._on_wildfire_map_observed, callback_args=uav,
                queue_size=10) for uav in self.uavs}

        self.sub_firemap_obs_alarm = rospy.Subscriber("wildfire_observed", WildfireMap,
                                                      callback=self._on_wildfire_map_observed,
                                                      callback_args="Alarm", queue_size=10)

        self.sub_mean_wind = rospy.Subscriber("mean_wind", MeanWindStamped, self.on_mean_wind)
        self.sub_propagate = rospy.Subscriber("propagate", PropagateCmd, self.on_propagate_cmd)
        self.sub_wildfire_point = rospy.Subscriber("wildfire_point", Timed2DPointStamped,
                                                   self.on_wildfire_obs_point)

        self.assessment_th = None  # self.assessment_th = threading.Thread(None, self.propagate, daemon=True)

    def propagate(self):
        with self.sa_lock:
            # Assess current condition
            rospy.loginfo("Assessment of current situation")
            self.sa.assess_current(datetime.datetime.fromtimestamp(rospy.Time.now().to_sec()))
            w = self.sa.wildfire.geodata
            w_time = self.sa.wildfire.time

            rospy.loginfo("Assessment of future situation")
            until = datetime.datetime.fromtimestamp((rospy.Time.now() + self.horizon).to_sec())
            self.sa.assess_until(until)
            p = self.sa.predicted_wildfire.geodata
            p_time = self.sa.predicted_wildfire.time

            self.emit_propagation(w, w_time, p, p_time, until)

    def emit_propagation(self, w: GeoData, w_timestamp: datetime.datetime, p: GeoData,
                         p_timestamp: datetime.datetime,
                         until: datetime.datetime):
        """Publish current situation assessment."""
        current = WildfireMap(
            header=rospy.Header(stamp=rospy.Time.from_sec(w_timestamp.timestamp())),
            raster=serialization.raster_msg_from_geodata(w, 'ignition'))
        pred = PredictedWildfireMap(
            header=rospy.Header(stamp=rospy.Time.from_sec(p_timestamp.timestamp())),
            last_valid=rospy.Time.from_sec(until.timestamp()),
            raster=serialization.raster_msg_from_geodata(p, 'ignition'))
        # self.pub_wildfire_real.publish(pred)
        self.pub_wildfire_pred.publish(pred)
        # self.pub_wildfire_real.publish(WildfireMap(
        #     header=rospy.Header(stamp=rospy.Time.from_sec(p_timestamp.timestamp())),
        #     raster=serialization.raster_msg_from_geodata(p, 'ignition')))
        self.pub_wildfire_current.publish(current)

        if self.sa.elevation_timestamp > self.last_elevation_timestamp:
            # pub_elevation is latching, don't bother other nodes with unnecessary elevation updates
            elevation = ElevationMap(
                header=rospy.Header(
                    stamp=rospy.Time.from_sec(self.sa.elevation_timestamp.timestamp())),
                raster=serialization.raster_msg_from_geodata(self.sa.elevation, 'elevation'))
            self.pub_elevation.publish(elevation)
            rospy.loginfo("A new Elevation Map has been generated")
            self.last_elevation_timestamp = self.sa.elevation_timestamp

        g = GeoDataDisplay.pyplot_figure(p)
        g.draw_ignition_shade(with_colorbar=True)
        g.figure.savefig("/home/rbailonr/fuego_pre.png")
        g.close()
        g = GeoDataDisplay.pyplot_figure(w)
        g.draw_ignition_shade(with_colorbar=True)
        g.figure.savefig("/home/rbailonr/fuego_cur.png")
        g.close()
        g = GeoDataDisplay.pyplot_figure(self.sa.observed_wildfire.geodata)
        g.draw_ignition_shade(with_colorbar=True)
        g.figure.savefig("/home/rbailonr/fuego_obs.png")
        g.close()
        del g

        rospy.loginfo("Wildfire propagation publishing ended")

    def on_mean_wind(self, msg: MeanWindStamped):
        with self.sa_lock:
            rospy.loginfo("Mean wind set to %s km/h direction %s °",
                          str(msg.speed), str(msg.direction / np.pi * 180))
            self.sa.set_surface_wind((msg.speed, msg.direction))

    def on_propagate_cmd(self, msg: PropagateCmd):
        if self.assessment_th is None:
            rospy.loginfo("Doing situation asessment")
            self.propagate()
        else:
            rospy.logwarn("Previous situation assessment action has not finished yet")

    def _on_wildfire_map_observed(self, msg: WildfireMap, uav: str):
        with self.sa_lock:
            local_firemap = serialization.geodata_from_raster_msg(msg.raster, "ignition")
            g = GeoDataDisplay.pyplot_figure(local_firemap)
            g.draw_ignition_shade(with_colorbar=True)
            g.figure.savefig("local_firemap_" + str(uav) + ".png")
            g.close()
            del g
            rospy.loginfo("Local wildfire map received from %s", str(uav))

            for cell in zip(*np.where(local_firemap.data["ignition"] < np.inf)):
                self.sa.observed_wildfire.set_cell_ignition(
                    (*cell, local_firemap["ignition"][cell]))

    def on_wildfire_obs_point(self, msg: Timed2DPointStamped):
        # if SA is locked, new observations should be stored in a queue
        with self.sa_lock:
            # TODO: Catch IndexError
            rospy.loginfo("New fire location received (%s, %s) with time %s",
                          str(msg.x), str(msg.y),
                          str(datetime.datetime.fromtimestamp(msg.t.to_sec())))
            self.sa.observed_wildfire.set_point_ignition((msg.x, msg.y, msg.t.to_sec()))
            self.last_wildfire_update = datetime.datetime.now()

    # TODO: Service providing the current elevation map
    # TODO: Service providing wind


if __name__ == '__main__':
    try:
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
        if not rospy.is_shutdown():
            sa = SituationAssessmentNode(area, world_paths=world_paths)
            # rospy.spin()
            # # Memory profiling
            # import objgraph
            # r = rospy.Rate(0.1)
            # while not rospy.is_shutdown():
            #     objgraph.show_refs([sa], filename='/home/rbailonr/sample-graph.png', max_depth=5)
            #     with open("/home/rbailonr/typestats.txt", 'w') as f:
            #         f.write(str(objgraph.typestats()))
            #     r.sleep()
    except rospy.ROSInterruptException:
        pass
