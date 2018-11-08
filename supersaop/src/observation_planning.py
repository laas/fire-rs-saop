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
import typing as ty
import uuid

import matplotlib
import numpy as np

matplotlib.use('Gtk3Agg')

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

from supersaop.msg import Euler, ElevationMap, PredictedWildfireMap, PropagateCmd, Plan, PlanCmd, \
    Trajectory, Maneuver, Raster, RasterMetaData, WildfireMap, MeanWindStamped, SurfaceWindMap, \
    Timed2DPointStamped, PoseEuler

from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.planning.display import TrajectoryDisplayExtension, plot_plan_trajectories
from fire_rs.monitoring.supersaop import ObservationPlanning

import serialization

TEST_AREA = ((480060.0, 489060.0), (6210074.0, 6217074.0))


class ObservationPlanningNode:

    def __init__(self):
        rospy.init_node('observation_planning')
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        self.op = ObservationPlanning(logging.getLogger(__name__),
                                      ObservationPlanning.DEFAULT_UAV_MODELS,
                                      ObservationPlanning.DEFAULT_VNS_CONFS)

        # Lock from new observations while doing predictions
        self.op_lock = threading.Lock()

        self.plan_id = str(uuid.uuid4())

        self.pub_plan = rospy.Publisher('plan', Plan, queue_size=10)

        self.sub_elevation = rospy.Subscriber("elevation", ElevationMap, self.on_elevation_map)
        self.sub_wildfire_precition = rospy.Subscriber("wildfire_prediction", PredictedWildfireMap,
                                                       self.on_wildfire_prediction)
        self.sub_elevation = rospy.Subscriber("initial_plan", PlanCmd, self.on_plan_cmd)

        self.planning_th = None  # threading.Thread(None, self.propagate, daemon=True)

    def plan(self, vns_conf_name: str, planning_duration: float, plan_prototype: Plan):
        rospy.loginfo("Initial plan request")

        with self.op_lock:
            traj_confs = [ObservationPlanning.TrajectoryConf(
                msg_tc.conf.name, msg_tc.conf.uav_model,
                (msg_tc.conf.start_wp.position.x, msg_tc.conf.start_wp.position.y,
                 msg_tc.conf.start_wp.position.z, msg_tc.conf.start_wp.orientation.psi),
                (msg_tc.conf.end_wp.position.x, msg_tc.conf.end_wp.position.y,
                 msg_tc.conf.end_wp.position.z, msg_tc.conf.end_wp.orientation.psi),
                rospy.Time.to_sec(msg_tc.conf.start_time), msg_tc.conf.max_duration, (
                    msg_tc.conf.wind.speed * np.cos(msg_tc.conf.wind.direction),
                    msg_tc.conf.wind.speed * np.sin(msg_tc.conf.wind.direction))) for msg_tc
                in plan_prototype.trajectories]
            self.op.initial_plan(plan_prototype.conf.name, plan_prototype.conf.prediction_id,
                                 vns_conf_name, traj_confs, (
                                     rospy.Time.to_sec(plan_prototype.conf.flight_window[0]),
                                     rospy.Time.to_sec(plan_prototype.conf.flight_window[1])))
            saop_plan = self.op.compute_plan(planning_duration)
            self.publish_plan(plan_prototype, saop_plan)

    def publish_plan(self, plan_prototype: Plan, saop_plan):
        """Publish current plan."""
        for (traj_msg, traj_plan) in zip(plan_prototype.trajectories, saop_plan.trajectories()):
            names = traj_plan.maneuver_names
            starts = traj_plan.start_times
            segs = traj_plan.segments
            maneuver_msg_list = [Maneuver(n, rospy.Time.from_sec(t),
                                          PoseEuler(position=Point(s.start.x, s.start.y, s.start.z),
                                                    orientation=Euler(.0, .0, s.start.dir)))
                                 for n, t, s in zip(names, starts, segs)]
            traj_msg.maneuvers = maneuver_msg_list
        g = GeoDataDisplay.pyplot_figure(self.op._tagged_elevation[1])
        g.add_extension(TrajectoryDisplayExtension, (None,), {})
        plot_plan_trajectories(saop_plan, g, trajectories=slice(None),
                               layers=["bases", "arrows", "trajectory_solid"])
        g.figure.savefig("plan.svg")
        self.pub_plan.publish(plan_prototype)

    def on_elevation_map(self, msg: ElevationMap):
        with self.op_lock:
            rospy.loginfo("Elevation map %s received", str(msg.elevation_id))
            decoded = serialization.geodata_from_raster_msg(msg.raster, layer="elevation")
            rospy.logwarn("Ignoring data from ElevationMap message. Using flat terrain")
            decoded.data["elevation"][...] = 0.
            self.op.logger.info("%s", str(decoded))
            self.op.set_elevation(msg.elevation_id, decoded)

    def on_wildfire_prediction(self, msg: PredictedWildfireMap):
        with self.op_lock:
            rospy.loginfo("Predicted wildfire map %s received", str(msg.prediction_id))
            self.op.set_wildfire_map(msg.prediction_id, serialization.geodata_from_raster_msg(
                msg.raster, layer="ignition"))

    def on_plan_cmd(self, msg: PlanCmd):
        rospy.loginfo("Initial plan request")
        if self.planning_th is None:
            rospy.loginfo("Initial plan request")
            self.plan(msg.vns_conf, msg.planning_duration, msg.plan_prototype)
            # self.assessment_th = threading.Thread(None, self.propagate, daemon=True)
            # self.assessment_th.start()
        else:
            rospy.loginfo("Previous planning has not ended")


if __name__ == '__main__':
    try:
        op = ObservationPlanningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
