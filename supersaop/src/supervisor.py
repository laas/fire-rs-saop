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

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from supersaop.msg import Euler, Plan, PlanConf, PlanCmd, Trajectory, TrajectoryConf, Maneuver, \
    MeanWind, PoseEuler


class SupervisorNode:
    def __init__(self):
        rospy.init_node("supervisor")
        rospy.loginfo("Starting {}".format(self.__class__.__name__))
        self.pub = rospy.Publisher("initial_plan", PlanCmd, queue_size=10)
        self.sub = rospy.Subscriber("plan", Plan, self.sub_plan)

    def request_demo_plan(self):
        start_wp = PoseEuler(position=Point(485060.0 - 1000.0, 6214074.0 - 1000.0, 0.0),
                             orientation=Euler(.0, .0, .0))
        end_wp = PoseEuler(position=Point(485060.0 - 1000.0, 6214074.0 - 1000.0, 0.0),
                           orientation=Euler(.0, .0, .0))

        p_conf = PlanConf(name="A stupid plan",
                          flight_window=(rospy.Time.from_sec(.0), rospy.Time.from_sec(4294967295)))
        t_conf = TrajectoryConf(name="One trajectory is enough", uav_model="x8-06",
                                start_wp=start_wp, end_wp=end_wp,
                                start_time=rospy.Time.now() + rospy.Duration.from_sec(1. * 60 * 60),
                                max_duration=6000, wind=MeanWind(.0, .0))
        p = Plan(header=Header(stamp=rospy.Time.now()),
                 conf=p_conf,
                 trajectories=[Trajectory(conf=t_conf, maneuvers=[])])

        plan_command = PlanCmd(vns_conf="demo", planning_duration=30.0, plan_prototype=p)
        self.pub.publish(plan_command)
        rospy.loginfo(plan_command)

    def sub_plan(self, msg: Plan):
        rospy.loginfo(msg)


if __name__ == '__main__':
    try:
        supnode = SupervisorNode()

        r = rospy.Rate(1)
        r.sleep()
        supnode.request_demo_plan()
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
