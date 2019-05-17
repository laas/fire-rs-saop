#!/usr/bin/env python3
# Copyright (c) 2018, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
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
import threading
import subprocess
import pymorse
import tempfile

import numpy as np

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from fire_rs.simulation.morse import MorseWildfire

import serialization

from supersaop.msg import Timed2DPointStamped, WildfireMap


class Resource:
    def __init__(self):
        self._value = None
        self._ready = threading.Event()

    def set(self, value):
        self._value = value
        self._ready.set()

    def get(self):
        val = self._value
        self._ready.clear()
        return val

    def is_ready(self):
        return self._ready.is_set()

    def wait(self, timeout=None):
        self._ready.wait(timeout)


class MorseProcess:
    def __init__(self, path, sim_env, python_path=None):
        self.path = path
        self.sim_env = sim_env
        self.python_path = None
        if python_path is not None:
            self.python_path = {'PYTHONPATH': python_path}
        self.process = None

    def run(self):
        self.process = subprocess.Popen(args=['morse', 'run', self.sim_env], cwd=self.path,
                                        env=self.python_path)
        if self.is_alive():
            rospy.loginfo("Morse is running")

    def kill(self):
        if self.process:
            self.process.kill()

    def is_alive(self):
        if self.process:
            if self.process.poll() is None:
                return True
            else:
                return False


class MorseWildfireNode:
    def __init__(self, wildfire_res):
        rospy.init_node("morse_wildfire")
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        self.sub_predicted_wildfire = rospy.Subscriber("real_wildfire", WildfireMap,
                                                       callback=self.on_predicted_wildfire,
                                                       queue_size=1)
        self.shared_wildfire_map = wildfire_res

    def on_predicted_wildfire(self, msg: WildfireMap):
        rospy.loginfo("Wildfire map received")
        raster = serialization.geodata_from_raster_msg(msg.raster, "ignition")
        self.shared_wildfire_map.set(raster)


if __name__ == '__main__':

    address = ('localhost', 4000)
    terrain_obj_name = "elevation"

    wf = Resource()
    node = MorseWildfireNode(wf)
    m = MorseWildfire(address, terrain_obj_name)

    stop_signal = False

    def update_map():
        while not (rospy.is_shutdown() or stop_signal):
            wf.wait(timeout=1.)
            if wf.is_ready():
                m.set_wildfire_prediction_map(wf.get())

    th = threading.Thread(target=update_map, daemon=True)
    th.start()

    try:
        r = rospy.Rate(1 / 60.)  # Once a minute
        rospy.sleep(1.)
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Updating wildfire in morse")
                m.update(rospy.Time.now().to_sec())
            except ConnectionError as e:
                rospy.logerr(e)
            except ValueError as e:
                rospy.logwarn(e)
            r.sleep()
    except rospy.ROSInterruptException:
        stop_signal = True
        th.join()
