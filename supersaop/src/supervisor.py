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

import numpy as np
import enum

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from supersaop.msg import Euler, Plan, PlanConf, PlanCmd, Trajectory, TrajectoryConf, Maneuver, \
    MeanWind, PoseEuler, WildfireMap, Timed2DPointStamped, PropagateCmd, PredictedWildfireMap

import serialization


@enum.unique
class SupervisorNodeState(enum.Enum):
    Alarm_recv = 1,
    Propagation = 2,
    Planning = 3,
    FireMonitoring = 4,


class SupervisorNode:
    def __init__(self, uav_bases):
        rospy.init_node("supervisor")
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        self.uav_bases = uav_bases

        self.pub_plan_request = rospy.Publisher("initial_plan", PlanCmd, queue_size=10)
        self.pub_propagate_request = rospy.Publisher("propagate", PropagateCmd, queue_size=10)
        self.sub_firemap_obs_alarm = rospy.Subscriber("wildfire_observed", WildfireMap,
                                                      callback=self._on_fire_alarm_recv,
                                                      callback_args="Alarm", queue_size=10)

        # self.sub_wildfire_point = rospy.Subscriber("wildfire_point", Timed2DPointStamped,
        #                                            callback=self._on_fire_alarm_recv,
        #                                            callback_args="Alarm")

        self.sub_wildfire_precition = rospy.Subscriber("wildfire_prediction", PredictedWildfireMap,
                                                       self.on_wildfire_prediction)
        self.sub_saop_plan = rospy.Subscriber("plan", Plan, self.on_saop_plan, queue_size=10)

        self.supervision_state = SupervisorNodeState.Alarm_recv
        self.timer = None

    def next_state(self):
        if self.supervision_state == SupervisorNodeState.Alarm_recv:
            # switch to Propagation
            rospy.loginfo("%s -> %s", self.supervision_state.name,
                          SupervisorNodeState.Propagation.name)
            self.supervision_state = SupervisorNodeState.Propagation
            self.pub_propagate_request.publish(PropagateCmd())
            self.pub_propagate_request.publish(PropagateCmd())
        elif self.supervision_state == SupervisorNodeState.Propagation:
            # switch to Planning
            rospy.loginfo("%s -> %s", self.supervision_state.name,
                          SupervisorNodeState.Planning.name)
            self.supervision_state = SupervisorNodeState.Planning
            self.request_demo1_plan()
        elif self.supervision_state == SupervisorNodeState.Planning:
            # switch to FireMonitoring
            rospy.loginfo("%s -> %s", self.supervision_state.name,
                          SupervisorNodeState.FireMonitoring.name)
            self.supervision_state = SupervisorNodeState.FireMonitoring

    def timer_callback(self, event):
        self.next_state()

    def _on_fire_alarm_recv(self, msg, who: str):
        if self.supervision_state == SupervisorNodeState.Alarm_recv:
            self.timer = rospy.Timer(rospy.Duration(secs=1, nsecs=0), self.timer_callback,
                                     oneshot=True)

    def on_wildfire_prediction(self, msg: PredictedWildfireMap):
        if self.supervision_state == SupervisorNodeState.Propagation:
            self.timer = rospy.Timer(rospy.Duration(secs=1, nsecs=0), self.timer_callback,
                                     oneshot=True)

    def request_demo1_plan(self):
        start_wp = PoseEuler(
            position=Point(self.uav_bases["x8-06"][0], self.uav_bases["x8-06"][1], 400.0),
            orientation=Euler(.0, .0, .0))
        end_wp = PoseEuler(
            position=Point(self.uav_bases["x8-06"][0], self.uav_bases["x8-06"][1], 400.0),
            orientation=Euler(.0, .0, .0))

        p_conf = PlanConf(name="firers 1 plan",
                          flight_window=(rospy.Time.from_sec(.0), rospy.Time.from_sec(4294967295)))
        t_conf = TrajectoryConf(name="firers 1 traj", uav_model="x8-06",
                                start_wp=start_wp, end_wp=end_wp,
                                start_time=rospy.Time.now() + rospy.Duration.from_sec(60),
                                max_duration=300, wind=MeanWind(.0, .0))
        p = Plan(header=Header(stamp=rospy.Time.now()),
                 conf=p_conf,
                 trajectories=[Trajectory(conf=t_conf, maneuvers=[])])

        plan_command = PlanCmd(vns_conf="demo", planning_duration=2.0, plan_prototype=p)
        self.pub_plan_request.publish(plan_command)
        rospy.loginfo(plan_command)

    def on_saop_plan(self, msg: Plan):
        if self.supervision_state == SupervisorNodeState.Planning:
            self.timer = rospy.Timer(rospy.Duration(secs=1, nsecs=0), self.timer_callback,
                                     oneshot=True)


if __name__ == '__main__':
    try:
        known_uavs = rospy.get_param("known_uavs")
        uav_bases = {}
        for uav in known_uavs:
            uav_bases[uav] = rospy.get_param(
                "/".join(("uavs", serialization.ros_name_for(uav), "base")))
        supnode = SupervisorNode(uav_bases)

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
