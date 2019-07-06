#  Copyright (c) 2019, CNRS-LAAS
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

"""A sequence of dubins paths with wind"""
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import fire_rs.uav_planning as up


def traj_2d(uav, orig, dest):
    origin = up.Waypoint(orig[0], orig[1], orig[2], orig[3])
    target = up.Waypoint(dest[0], dest[1], orig[2], dest[3])

    samples = uav.path_sampling(origin, target, 1)
    x = [wp.x for wp in samples]
    y = [wp.y for wp in samples]
    return x, y


def traj_wind_groundframe(uav, orig, dest, wind):
    origin = up.Waypoint(orig[0], orig[1], orig[2], orig[3])
    target = up.Waypoint(dest[0], dest[1], orig[2], dest[3])

    samples = uav.path_sampling(origin, target, wind, 1)
    x = [wp.x for wp in samples]
    y = [wp.y for wp in samples]
    return x, y


def traj_wind_airframe(uav, orig, dest, wind):
    origin = up.Waypoint(orig[0], orig[1], orig[2], orig[3])
    target = up.Waypoint(dest[0], dest[1], orig[2], dest[3])

    samples = uav.path_sampling_airframe(origin, target, wind, 1)
    x = [wp.x for wp in samples]
    y = [wp.y for wp in samples]
    return x, y


if __name__ == '__main__':
    matplotlib.rcParams['font.family'] = 'sans-serif'
    matplotlib.rcParams['text.usetex'] = True

    uav_speed = 10.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180
    uav_max_pitch_angle = 6. / 180. * np.pi
    wind = up.WindVector(0, -3)
    no_wind = up.WindVector(0, 0)
    a_uav = up.UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)

    wp_s, wp_e = up.Waypoint(0, 0, 0, 0), up.Waypoint(100, 0, 0, 0)
    seg = up.Segment(wp_s, wp_e)
    tr_time = a_uav.travel_time(seg, wind)
    print(tr_time)
    print(a_uav.travel_time(wp_s, wp_e, wind))

    fig = plt.figure()
    # Low altitude
    ax = fig.add_subplot(111, aspect='equal')
    # ax.set_xlim((-10, 70))
    # ax.set_ylim((-20, 80))

    ax.set_axis_off()  #Hide axes

    wp_ma = [(-8, 0, 0, 0), (50, 50, 0, 5*np.pi/4), (20, 50, 0, 0.)]  # , (0, 100, 20, 0)]
    for i in range(len(wp_ma) - 1):
        t_no_wind = traj_wind_groundframe(a_uav, wp_ma[i], wp_ma[i + 1], no_wind)
        t_wind = traj_wind_groundframe(a_uav, wp_ma[i], wp_ma[i + 1], wind)
        t_air = traj_wind_airframe(a_uav, wp_ma[i], wp_ma[i + 1], wind)
        # ax.plot(*t_no_wind, label="No wind")
        ax.plot(*t_wind, c='orange')
        # ax.plot(*t_air, linestyle='--', label="Air frame")

        ax.arrow(wp_ma[i][0], wp_ma[i][1], 4 * np.cos(wp_ma[i][3]), 4 * np.sin(wp_ma[i][3]),
                 fc="darkred", ec="darkred", head_width=4, head_length=4, zorder=100)
        ax.arrow(wp_ma[i + 1][0], wp_ma[i + 1][1], 4 * np.cos(wp_ma[i + 1][3]),
                 4 * np.sin(wp_ma[i + 1][3]),
                 fc="darkred", ec="darkred", head_width=4, head_length=4, zorder=100)
        ax.scatter(wp_ma[i][0], wp_ma[i][1], c='darkred', zorder=100)  # Start
        ax.scatter(wp_ma[i + 1][0], wp_ma[i + 1][1], c='darkred', zorder=100)  # End
        # ax.annotate('start', xy=(wp_ma[i][0], wp_ma[i][1]), xytext=(wp_ma[i][0], wp_ma[i][1] - 10))
        # ax.annotate('end', xy=(wp_ma[i + 1][0], wp_ma[i + 1][1]),
        #             xytext=(wp_ma[i + 1][0], wp_ma[i + 1][1] - 10))

        wind_location = (-10, 50)
        # ax.annotate("wind",  # + " " + format(wind.speed(), '.0f') + 'm/s',
        #             xy=(wind_location[0], wind_location[1]),
        #             xytext=(wind_location[0] - 0 * np.cos(wind.dir()),
        #                     wind_location[1] - 5 * np.cos(wind.dir())))
        ax.arrow(wind_location[0], wind_location[1],
                 wind.speed() * np.cos(wind.dir()),
                 wind.speed() * np.sin(wind.dir()),
                 fc="k", ec="k", head_width=4, head_length=5, zorder=100)

        # virtual target
        # ax.scatter(t_air[0][-1], t_air[1][-1], c='k', zorder=100)  # vt
        # ax.annotate('virtual\ntarget', xy=(t_air[0][-1], t_air[1][-1]),
        #             xytext=(t_air[0][-1] - 10, t_air[1][-1] - 10))
        # ax.annotate("", xy=(t_air[0][-1], t_air[1][-1]), xytext=(wp_ma[i + 1][0], wp_ma[i + 1][1]),
        #             arrowprops=dict(arrowstyle="->"), zorder=101)

    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.legend(loc='upper right')
    # plt.title("Optimal trajectory of a UAV under the presence of constant wind")

    # fig.set_size_inches(5, 6)
    # fig.savefig("dubinswind-method.pdf", dpi=300, bbox_inches='tight')
    # fig.savefig("dubinswind-method.svg", dpi=300, bbox_inches='tight')
    plt.show(block=True)

    print("end")
