#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <morse_sim> environment

Feel free to edit this template as you like!
"""


import os
import os.path

import numpy as np

os.chdir(os.path.dirname(__file__))

from morse.builder import *
from morse_sim.builder.actuators.absolute_teleport import AbsoluteTeleport


# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> morse_sim' can help you to build custom robots.
drone = RMax("drone")

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
#drone.translate(485444.0, 6214440.0, 2500.0)
drone.translate(2776735.,2212618., 1000.)
drone.rotate(0.0, 0.0, 0)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> morse_sim' can help you with the creation of a custom
# actuator.
motion = AbsoluteTeleport("teleport")
motion.add_service('socket')
drone.append(motion)

# Add a keyboard controller to move the robot with arrow keys.
keyboard = Keyboard()
drone.append(keyboard)
keyboard.properties(ControlType='Position')
keyboard.properties(Speed=25)

# Add a pose sensor that exports the current location and orientation
# of the robot in the world frame
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#sensors
#
# 'morse add sensor <name> morse_sim' can help you with the creation of a custom
# sensor.
pose = Pose()
drone.append(pose)

# Append a camera
ircam = VideoCamera("ircam")
ircam.properties(cam_width=640, cam_height=480,
                 cam_far=5000, cam_fov=44)
ircam.frequency(0)
ircam._n = 0
ircam.add_service('socket')
ircam.add_stream('socket', 'morse.middleware.sockets.video_camera.Video8uPublisher')
drone.append(ircam)

# add pose sensor for the camera
ircam_pose = Pose("ircam_pose")
ircam_pose.frequency(1)
ircam_pose.add_service('socket')
drone.append(ircam_pose)

ircam.rotate(0, -np.pi/2, 0)
ircam_pose.rotate(np.pi, 0, -np.pi/2)

# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html 
# the other available interfaces (like ROS, YARP...)
drone.add_default_interface('socket')

# set 'fastmode' to True to switch to wireframe mode
#env = Environment('land-1/trees', fastmode=False)
workdir = os.getcwd()
env = Environment(os.path.join(workdir, 'environment', 'porto.blend'), fastmode=False)
env.set_camera_location([0, 0, 2500])
env.set_camera_clip(0.1, 3000)
env.set_camera_speed(100)
env.set_camera_rotation([0, 0, -np.pi])
