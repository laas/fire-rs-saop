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
import random
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

    ran = random.Random()

    # Random trajectories
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    def random_wp_pair(n=np.inf):
        for i in range(n):
            yield ((ran.randrange(-200, 200), ran.randrange(-200, 200), ran.randrange(-200, 200), ran.random()*2*np.pi),
                   (ran.randrange(-200, 200), ran.randrange(-200, 200), ran.randrange(-200, 200), ran.random()*2*np.pi))

    for wp_pair in random_wp_pair(50):
        print(wp_pair)
        ax.plot(*traj(uav, wp_pair[0], wp_pair[1]))

    plt.xlabel("x")
    plt.ylabel("y")
    plt.show(block=False)

    # Examples of failing trajectories while planing but not here.
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    wp_ma = [(480160, 6210174, 100, 0), (480160, 6210174, 100, 0),
             (483844.51013562991, 6216373.3243948203, 1073, 1.0377441685192268), (483869.91835593828, 6216416.387399137, 1073, 1.0377441685192268),
             (484047.24726410303, 6216721.4141843673, 1104, 0.49659006833227348), (484091.20787735051, 6216745.2356973942, 1104, 0.49659006833227348),
             (484144.84432897501, 6216823.1286639366, 1027, 1.0441990609703493), (484169.97405482916, 6216866.3547773231, 1027, 1.0441990609703493),
             (480160, 6210174, 100, 0), (480160, 6210174, 100, 0),]
    for i in range(len(wp_ma) - 1):
        ax.plot(*traj(uav, wp_ma[i], wp_ma[i + 1]))
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show(block=False)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    wp_ma = [(485046.97901929758, 6212440.4672314832, 1088, 1.2639035346324197),
             (485195.52938023448, 6212499.8414709819, 1055, 0.4326838402098257)]
    for i in range(len(wp_ma) - 1):
        ax.plot(*traj(uav, wp_ma[i], wp_ma[i + 1]))
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show(block=False)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    wp_ma = [(485309.30963665573, 6213078.6511183679, 983, -2.7678397231859977),
             (485467.56226040138, 6212850.4756678976, 875, -0.99961171783044567)]
    for i in range(len(wp_ma) - 1):
        ax.plot(*traj(uav, wp_ma[i], wp_ma[i + 1]))
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show(block=False)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    wp_ma = [(485290, 6213075, 983, 0), (485397.89720828738, 6213320.2460238663, 897, 0.51887048029539962)]
    for i in range(len(wp_ma) - 1):
        ax.plot(*traj(uav, wp_ma[i], wp_ma[i + 1]))
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show(block=True)


