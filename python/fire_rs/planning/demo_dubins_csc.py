# Copyright (c) 2020, CNRS-LAAS
# All rights reserved.
#
# #Distribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * #Distributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * #Distributions in binary form must reproduce the above copyright notice,
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

import matplotlib
matplotlib.use('Cairo')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import fire_rs.uav_planning as up

matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['text.usetex'] = True
output_format = "svg"

if __name__ == '__main__':
    def traj(uav, orig, dest):
        origin = up.Waypoint(orig[0], orig[1], orig[2], orig[3])
        target = up.Waypoint(dest[0], dest[1], dest[2], dest[3])

        samples = uav.path_sampling(origin, target, 1)
        # samples = uav.path_sampling(origin, target, up.WindVector(3, 3), 1)

        x = [wp.x for wp in samples]
        y = [wp.y for wp in samples]
        z = [wp.z for wp in samples]
        return x, y, z


    uav_speed = 18.  # m/s
    uav_max_turn_rate = 0.75 * 32. * np.pi / 180
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav = up.UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    wp_ma = [(-75, 0, 0, np.pi), (75, 0, 0, np.pi)]
    print(traj(uav, wp_ma[0], wp_ma[1]))
    ax.plot(*traj(uav, wp_ma[0], wp_ma[1])[0:2], c="k", zorder=-1000)
    # ax.annotate('start', size=15, xy=(wp_ma[0][0], wp_ma[0][1]), xytext=(wp_ma[0][0]-, wp_ma[0][1] - 20))
    # ax.annotate('end', size=15, xy=(wp_ma[0 + 1][0], wp_ma[0 + 1][1]),
    #             xytext=(wp_ma[0 + 1][0], wp_ma[0 + 1][1] - 20))
    ax.scatter(wp_ma[0][0], wp_ma[0][1], c="#1b9e77")
    ax.scatter(wp_ma[1][0], wp_ma[1][1], c="#d95f02")
    ax.set_xbound(-150, 150)
    ax.set_ybound(-150, 150)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("LSL")
    plt.axis('off')
    plt.show(block=True)
    plt.savefig('.'.join(("dubins_paths_LSL", output_format)), dpi=72, bbox_inches='tight')

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    wp_ma = [(-40, 0, 0, 0.5 * np.pi), (40, 0, 0, 0 * np.pi)]
    print(traj(uav, wp_ma[0], wp_ma[1]))
    ax.plot(*traj(uav, wp_ma[0], wp_ma[1])[0:2], c="k", zorder=-1000)
    ax.scatter(wp_ma[0][0], wp_ma[0][1], c="#1b9e77")
    ax.scatter(wp_ma[1][0], wp_ma[1][1], c="#d95f02")
    ax.set_xbound(-150, 150)
    ax.set_ybound(-150, 150)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("LRL")
    plt.axis('off')
    plt.show(block=True)
    plt.savefig('.'.join(("dubins_paths_LRL", output_format)), dpi=72, bbox_inches='tight')

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    wp_ma = [(-75, 0, 0, 0.75 * np.pi), (75, 0, 0, -0.75 * np.pi)]
    print(traj(uav, wp_ma[0], wp_ma[1]))
    ax.plot(*traj(uav, wp_ma[0], wp_ma[1])[0:2], c="k", zorder=-1000)
    ax.scatter(wp_ma[0][0], wp_ma[0][1], c="#1b9e77")
    ax.scatter(wp_ma[1][0], wp_ma[1][1], c="#d95f02")
    ax.set_xbound(-150, 150)
    ax.set_ybound(-150, 150)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("RSR")
    plt.axis('off')
    plt.show(block=True)
    plt.savefig('.'.join(("dubins_paths_RSR", output_format)), dpi=72, bbox_inches='tight')

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    wp_ma = [(-75, 0, 0, 1.25 * np.pi), (75, 0, 0, 1.25 * np.pi)]
    print(traj(uav, wp_ma[0], wp_ma[1]))
    ax.plot(*traj(uav, wp_ma[0], wp_ma[1])[0:2], c="k", zorder=-1000)
    ax.scatter(wp_ma[0][0], wp_ma[0][1], c="#1b9e77")
    ax.scatter(wp_ma[1][0], wp_ma[1][1], c="#d95f02")
    ax.set_xbound(-150, 150)
    ax.set_ybound(-150, 150)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("LSR")
    plt.axis('off')
    plt.show(block=True)
    plt.savefig('.'.join(("dubins_paths_LSR", output_format)), dpi=72, bbox_inches='tight')

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    wp_ma = [(-75, 0, 0, 0.75 * np.pi), (75, 0, 0, .75 * np.pi)]
    print(traj(uav, wp_ma[0], wp_ma[1]))
    ax.plot(*traj(uav, wp_ma[0], wp_ma[1])[0:2], c="k", zorder=-1000)
    ax.scatter(wp_ma[0][0], wp_ma[0][1], c="#1b9e77")
    ax.scatter(wp_ma[1][0], wp_ma[1][1], c="#d95f02")
    ax.set_xbound(-150, 150)
    ax.set_ybound(-150, 150)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("RSL")
    plt.axis('off')
    plt.savefig('.'.join(("dubins_paths_RSL", output_format)), dpi=72, bbox_inches='tight')
    plt.show(block=True)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    wp_ma = [(-40, 0, 0, 1.5 * np.pi), (40, 0, 0, 0 * np.pi)]
    print(traj(uav, wp_ma[0], wp_ma[1]))
    ax.plot(*traj(uav, wp_ma[0], wp_ma[1])[0:2], c="k", zorder=-1000)
    ax.scatter(wp_ma[0][0], wp_ma[0][1], c="#1b9e77")
    ax.scatter(wp_ma[1][0], wp_ma[1][1], c="#d95f02")
    ax.set_xbound(-150, 150)
    ax.set_ybound(-150, 150)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("RLR")
    plt.axis('off')

    plt.savefig('.'.join(("dubins_paths_RLR", output_format)), dpi=72, bbox_inches='tight')
    plt.show(block=True)
