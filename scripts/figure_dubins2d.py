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

import matplotlib
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

    def traj2d(uav, orig, dest):
        origin = up.Waypoint(orig[0], orig[1], orig[2], orig[3])
        target = up.Waypoint(dest[0], dest[1], dest[2], dest[3])

        samples = uav.path_sampling(origin, target, 1)
        x = [wp.x for wp in samples]
        y = [wp.y for wp in samples]
        return x, y

    uav_speed = 18.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180 * 4
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav = up.UAV(uav_speed, uav_max_turn_rate, uav_max_pitch_angle)

    matplotlib.rcParams['font.family'] = 'sans-serif'
    matplotlib.rcParams['text.usetex'] = True

    # wp_ma = [(0, 0, 0, np.pi/4), (50, 0, 0, np.pi/4)]
    # wp_ma = [(0, 0, 0, np.pi), (50, 0, 0, np.pi)]  # LSL
    # wp_ma = [(0, 0, 0, np.pi/2), (50, 0, 0, np.pi/4)]  # RSL

    wp_ma_lrl = [(-5, 0, 0, np.pi/2), (5, 0, 0, 3*np.pi/2)]  # LRL
    wp_ma_rlr = [(-5-5, -5+15, 0, 0), (5-5, 2+15, 0, np.pi/2)]  # RLR

    wp_dict = {"LRL": wp_ma_lrl,
               "RLR": wp_ma_rlr}

    for wp_conf, wp_ma in wp_dict.items():
        fig = plt.figure()
        fig.set_size_inches(5, 5)
        ax = fig.add_subplot(111, aspect='equal')
        for i in range(len(wp_ma) - 1):
            ll = traj2d(uav, wp_ma[i], wp_ma[i + 1])
            ax.plot(*ll)
            ax.set_ylim([-15, 35])
            ax.set_xlim([-20, 20])
            ax.arrow(wp_ma[i][0], wp_ma[i][1], 4*np.cos(wp_ma[i][3]), 4*np.sin(wp_ma[i][3]),
                     fc="darkgreen", ec="darkgreen", head_width=1, head_length=1, zorder=100)
            ax.arrow(wp_ma[i+1][0], wp_ma[i+1][1], 4*np.cos(wp_ma[i+1][3]), 4*np.sin(wp_ma[i+1][3]),
                     fc="r", ec="r", head_width=1, head_length=1, zorder=100)
            ax.scatter(wp_ma[i][0], wp_ma[i][1], c='darkgreen', zorder=101)  # Start
            ax.scatter(wp_ma[i + 1][0], wp_ma[i + 1][1], c='r', zorder=101)  # End
            ax.annotate('start', xy=(wp_ma[i][0], wp_ma[i][1]), xytext=(wp_ma[i][0]-6, wp_ma[i][1]))
            ax.annotate('end', xy=(wp_ma[i+1][0], wp_ma[i+1][1]), xytext=(wp_ma[i+1][0]+2, wp_ma[i+1][1]))
            ax.set_axis_off()

            plt.savefig(".".join((str(wp_conf), "svg")), dpi=100, bbox_inches='tight')
        plt.show(block=True)