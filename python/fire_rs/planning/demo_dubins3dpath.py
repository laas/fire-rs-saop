# Copyright (c) 2017, CNRS-LAAS
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

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import fire_rs.uav_planning as up

if __name__ == '__main__':

    def traj(uav, orig, dest):
        origin = up.Waypoint(orig[0], orig[1], orig[2], orig[3])
        target = up.Waypoint(dest[0], dest[1], dest[2], dest[3])

        samples = uav.path_sampling(origin, target, 1)
        x = [wp.x for wp in samples]
        y = [wp.y for wp in samples]
        z = [wp.z for wp in samples]
        return x, y, z

    uav_speed = 18.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav = up.UAV(uav_speed, uav_max_turn_rate, uav_max_pitch_angle)


    fig = plt.figure()
    # Low altitude
    ax = fig.add_subplot(131, projection='3d')
    wp_ma = [(0, 0, 0, np.pi), (100, 50, 10, np.pi), (0, 100, 20, 0)]
    for i in range(len(wp_ma) - 1):
        ax.plot(*traj(uav, wp_ma[i], wp_ma[i + 1]))
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Low Altitude")

    # Medium altitude
    ax2 = fig.add_subplot(132, projection='3d')
    wp_ma = [(0, 0, 0, 0), (100, 50, 20, np.pi/4), (0, 50, 60, np.pi/4)]
    for i in range(len(wp_ma)-1):
        ax2.plot(*traj(uav, wp_ma[i], wp_ma[i + 1]))
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Medium Altitude")

    # High altitude
    ax3 = fig.add_subplot(133, projection='3d')
    wp_ha = [(0, 0, 0, 0), (100, 50, 100, np.pi/4), (150, 150, 200, 3*np.pi/4), (0, 0, 0, 0),]
    for i in range(len(wp_ha) - 1):
        ax3.plot(*traj(uav, wp_ha[i], wp_ha[i + 1]))
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("High Altitude")

    plt.show(block=True)