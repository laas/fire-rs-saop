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

from supersaop.msg import Timed2DPointStamped, PropagateCmd


class LighterNode:
    def __init__(self):
        rospy.init_node("wildfire_starter")
        rospy.loginfo("Starting {}".format(self.__class__.__name__))
        self.pub_w = rospy.Publisher("wildfire_point", Timed2DPointStamped, queue_size=1)
        self.pub_p = rospy.Publisher("propagate", PropagateCmd, queue_size=1)

    def set_on_fire(self, position, timestamp):
        fire_pos = Timed2DPointStamped(header=Header(stamp=rospy.Time.now()), x=position[0],
                                       y=position[1], t=timestamp)

        self.pub_w.publish(fire_pos)
        rospy.loginfo(fire_pos)

    def propagate(self):
        pcmd = PropagateCmd()
        self.pub_p.publish(pcmd)
        rospy.loginfo("Propagate")


if __name__ == '__main__':
    try:
        w_starter = LighterNode()
        half_hour = rospy.Duration(secs=30 * 60)
        one_hour = rospy.Duration(secs=1 * 60 * 60)
        two_hours = rospy.Duration(secs=2 * 60 * 60)

        actions = [(w_starter.set_on_fire, ((483060.0, 6214074.0), rospy.Time.now() - one_hour)),
                   (w_starter.propagate, None),
                   (w_starter.set_on_fire, ((484060.0, 6214574.0), rospy.Time.now() - two_hours)),
                   (w_starter.propagate, None)]

        r = rospy.Rate(1 / 2.)

        while actions:
            a = actions.pop(0)
            if a[1] is not None:
                a[0](*a[1])
            else:
                a[0]()
            r.sleep()

    except rospy.ROSInterruptException:
        pass
