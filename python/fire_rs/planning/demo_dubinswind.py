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
import numpy as np
import fire_rs.uav_planning as up


def traj_2d(uav, orig, dest):
    origin = up.Waypoint(orig[0], orig[1], 0, orig[2])
    target = up.Waypoint(dest[0], dest[1], 0, dest[2])

    samples = uav.path_sampling(origin, target, 1)
    x = [wp.x for wp in samples]
    y = [wp.y for wp in samples]
    return x, y


def traj_wind_groundframe(uav, orig, dest, wind):
    origin = up.Waypoint2d(orig[0], orig[1], orig[2])
    target = up.Waypoint2d(dest[0], dest[1], dest[2])

    samples = uav.path_sampling(origin, target, wind, 1)
    x = [wp.x for wp in samples]
    y = [wp.y for wp in samples]
    return x, y


def traj_wind_airframe(uav, orig, dest, wind):
    origin = up.Waypoint2d(orig[0], orig[1], orig[2])
    target = up.Waypoint2d(dest[0], dest[1], dest[2])

    samples = uav.path_sampling_airframe(origin, target, wind, 1)
    x = [wp.x for wp in samples]
    y = [wp.y for wp in samples]
    return x, y


if __name__ == '__main__':

    uav_speed = 18.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180
    uav_max_pitch_angle = 6. / 180. * np.pi
    wind = up.WindVector(-2, -2)
    no_wind = up.WindVector(0, 0)
    a_uav = up.UAV(uav_speed, uav_max_turn_rate, uav_max_pitch_angle)

    fig = plt.figure()
    # Low altitude
    ax = fig.add_subplot(111)
    ax.set_xlim((-200, 100))
    ax.set_ylim((-100, 200))
    wp_ma = [(-100, 0, np.pi), (0, 100, np.pi)]  # , (0, 100, 20, 0)]
    for i in range(len(wp_ma) - 1):
        ax.plot(*traj_wind_groundframe(a_uav, wp_ma[i], wp_ma[i + 1], no_wind), alpha=.66,
                label="W/o wind")
        ax.plot(*traj_wind_groundframe(a_uav, wp_ma[i], wp_ma[i + 1], wind), label="Ground frame")
        ax.plot(*traj_wind_airframe(a_uav, wp_ma[i], wp_ma[i + 1], wind), linestyle='--',
                label="Air frame")

        ax.arrow(wp_ma[i][0], wp_ma[i][1], 4 * np.cos(wp_ma[i][2]), 4 * np.sin(wp_ma[i][2]),
                 fc="darkgreen", ec="darkgreen", head_width=4, head_length=4, zorder=100)
        ax.arrow(wp_ma[i + 1][0], wp_ma[i + 1][1], 4 * np.cos(wp_ma[i + 1][2]),
                 4 * np.sin(wp_ma[i + 1][2]),
                 fc="r", ec="r", head_width=4, head_length=4, zorder=100)
        ax.scatter(wp_ma[i][0], wp_ma[i][1], c='darkgreen', zorder=101)  # Start
        ax.scatter(wp_ma[i + 1][0], wp_ma[i + 1][1], c='r', zorder=101)  # End
        ax.annotate('start', xy=(wp_ma[i][0], wp_ma[i][1]), xytext=(wp_ma[i][0], wp_ma[i][1] - 15))
        ax.annotate('end', xy=(wp_ma[i + 1][0], wp_ma[i + 1][1]),
                    xytext=(wp_ma[i + 1][0], wp_ma[i + 1][1] + 10))

        wind_location = (-150, 150)
        ax.annotate('wind ' + format(wind.speed(), '.2f') + 'm/s', xy=(wind_location[0], wind_location[1]),
                    xytext=(wind_location[0] - 30 * np.cos(wind.dir()),
                            wind_location[1] - 30 * np.cos(wind.dir())))
        ax.arrow(wind_location[0], wind_location[1],
                 wind.speed() * np.cos(wind.dir()),
                 wind.speed() * np.sin(wind.dir()),
                 fc="k", ec="k", head_width=5, head_length=5, zorder=100)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend(loc='upper right')
    plt.title("Optimal trajectory of a UAV under the presence of constant wind")

    plt.show(block=True)
    print("end")
