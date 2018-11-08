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
import logging
import threading
import uuid

import matplotlib

matplotlib.use('Gtk3Agg')

import rospy
from std_msgs.msg import String

from supersaop.msg import ElevationMap, PredictedWildfireMap, PropagateCmd, Raster, RasterMetaData, \
    WildfireMap, MeanWindStamped, SurfaceWindMap, Timed2DPointStamped

from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.monitoring.supersaop import SituationAssessment

import serialization

TEST_AREA = ((480060.0, 489060.0), (6210074.0, 6217074.0))


class SituationAssessmentNode:

    def __init__(self):
        rospy.init_node('situation_assessment')
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        self.sa = SituationAssessment(TEST_AREA, logging.getLogger(__name__))

        # Lock from new observations while doing predictions
        self.sa_lock = threading.Lock()

        self.wildfire_id = str(uuid.uuid4())

        self.horizon = datetime.timedelta(hours=3)

        self.pub_wildfire_pred = rospy.Publisher('wildfire_prediction',
                                                 PredictedWildfireMap,
                                                 queue_size=10, latch=True)
        self.pub_wildfire_obs = rospy.Publisher('wildfire', WildfireMap,
                                                queue_size=10, latch=True)
        self.pub_elevation = rospy.Publisher('elevation', ElevationMap,
                                             queue_size=10, latch=True)
        # self.pub_surface_wind = rospy.Publisher('surface_wind', SurfaceWindMap,
        #                                         queue_size=10, latch=True)

        self.sub_mean_wind = rospy.Subscriber("mean_wind", MeanWindStamped, self.on_mean_wind)
        self.sub_propagate = rospy.Subscriber("propagate", PropagateCmd, self.on_propagate_cmd)
        self.sub_wildfire_point = rospy.Subscriber("wildfire_point", Timed2DPointStamped,
                                                   self.on_wildfire_obs_point)

        self.assessment_th = None  # self.assessment_th = threading.Thread(None, self.propagate,                                                                   daemon=True)

    def propagate(self):
        with self.sa_lock:
            until = datetime.datetime.now() + self.horizon
            self.sa.assess_until(until)
            w = self.sa.wildfire
            w_uuid = self.wildfire_id
            p_uuid = str(self.sa.predicted_wildfire_id)
            p = self.sa.predicted_wildfire.clone()
        self.emit_propagation(w, w_uuid, p, p_uuid, until)

    def emit_propagation(self, w, w_id, p, p_id, until):
        """Publish current situation assessment."""
        obs = WildfireMap(header=rospy.Header(stamp=rospy.Time.now()),
                          wildfire_id=w_id,
                          raster=serialization.raster_msg_from_geodata(w, 'ignition'))
        pred = PredictedWildfireMap(header=rospy.Header(stamp=rospy.Time.now()),
                                    wildfire_id=w_id, prediction_id=p_id,
                                    last_valid=rospy.Time(secs=int(until.timestamp())),
                                    raster=serialization.raster_msg_from_geodata(p, 'ignition'))
        elevation = ElevationMap(header=rospy.Header(stamp=rospy.Time.now()),
                                 raster=serialization.raster_msg_from_geodata(self.sa.environment.raster, 'elevation'))
        self.pub_wildfire_pred.publish(pred)
        self.pub_wildfire_obs.publish(obs)
        self.pub_elevation.publish(elevation)

    def on_mean_wind(self, msg: MeanWindStamped):
        with self.sa_lock:
            self.sa.set_surface_wind((msg.speed, msg.direction))

    def on_propagate_cmd(self, msg: PropagateCmd):
        if self.assessment_th is None:
            rospy.loginfo("Doing wildfire propagation")
            self.propagate()
            # self.assessment_th = threading.Thread(None, self.propagate, daemon=True)
            # self.assessment_th.start()
        else:
            rospy.loginfo("Previous propagation has not ended")

    def on_wildfire_obs_point(self, msg: Timed2DPointStamped):
        # if SA is locked, new observations should be stored in a queue
        with self.sa_lock:
            # TODO: Catch IndexError
            rospy.loginfo("New fire location received (%s, %s) with time %s",
                          str(msg.x), str(msg.y),
                          str(datetime.datetime.fromtimestamp(msg.t.to_sec())))
            self.sa.set_point_onfire((msg.x, msg.y, msg.t.to_sec()))
            self.wildfire_id = str(uuid.uuid4())

    # TODO: Service providing the current elevation map
    # TODO: Service providing wind


def talker():
    print("hello")
    pub = rospy.Publisher('chatter', String, queue_size=10)

    rate = rospy.Rate(1)  # 1 hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        sa = SituationAssessmentNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
