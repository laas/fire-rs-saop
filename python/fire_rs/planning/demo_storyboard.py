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


if __name__ == '__main__':
    import logging

    # logging.basicConfig(level=logging.DEBUG)

    import numpy as np
    import matplotlib
    import matplotlib.cm

    matplotlib.rcParams['font.family'] = 'sans-serif'
    matplotlib.rcParams['text.usetex'] = True

    import fire_rs.uav_planning as op
    import fire_rs.neptus_interface as neptus
    import fire_rs.firemapping as fmapping

    import fire_rs.geodata.display
    from fire_rs.firemodel import propagation
    from fire_rs.geodata.geo_data import TimedPoint, GeoData
    from fire_rs.planning.planning import FireMapper, FlightConf, Planner, PlanningEnvironment, \
        UAVConf, Waypoint
    from fire_rs.planning.display import TrajectoryDisplayExtension, plot_plan_trajectories

    # Geographic environment (elevation, landcover, wind...)
    wind = (10., np.pi / 4)  # 10m/s = 36km/h
    area = ((478500.0, 483500.0), (6210000.0, 6215000.0))
    env = PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=0)

    # Fire applied to the previous environment
    ignition_point = TimedPoint(area[0][0] + 2500.0, area[1][0] + 2100.0, 0)

    terrain_figure = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster, frame=(0, 0))
    terrain_figure.draw_elevation_shade()
    terrain_figure.draw_ignition_points(ignition_point)
    terrain_figure.figure.show()

    wind_figure = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster, frame=(0, 0))
    wind_figure.draw_wind_quiver()
    wind_figure.draw_ignition_points(ignition_point)
    wind_figure.figure.show()

    logging.info("Start of propagation")
    fire = propagation.propagate_from_points(env, ignition_point, 120 * 60)  # 60 minutes
    logging.info("End of propagation")

    fire_figure = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(fire.prop_data, frame=(0, 0))
    fire_figure.draw_ignition_contour(with_labels=True)
    fire_figure.draw_ignition_points(ignition_point, zorder=100000)
    fire_figure.figure.show()
    print()

    # ANIMATION OF FIRE
    import matplotlib.animation as animation

    anim_fire_figure = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(fire.prop_data,
                                                                            frame=(0, 0))

    # animation function
    # i: frame number
    # 1 frame = 1 minute
    def animate(i):
        print("frame {}".format(i))

        igni = np.array(fire.prop_data['ignition'])
        igni[igni >= np.finfo(np.float64).max] = np.nan  # mask non ignited cells
        igni = igni.T / 60.  # To minutes

        # contour(X, Y, Z, N, **kwargs)
        return anim_fire_figure.axes.contourf(anim_fire_figure._x_ticks,
                                              anim_fire_figure._y_ticks, igni,
                                              [0, i + 0.1, i + 1], cmap=matplotlib.cm.YlOrRd)


    anim = animation.FuncAnimation(anim_fire_figure.figure, animate, frames=120)
    anim.save('fire_animation.mp4', fps=12, dpi=200)
    anim_fire_figure.figure.show()
    print()

    # CREATE VNS PLAN

    # Configure some flight
    base_wp_1 = Waypoint(area[0][1] - 150., area[1][1] - 100., 0., 0.)
    # base_wp_2 = Waypoint(area[0][1] - 150., area[1][1] - 100., 0., 0.)
    start_t = 60 * 60  # 30 minutes after the ignition
    uavconf = UAVConf.X8()
    uavconf.max_flight_time = 30 * 60
    fgconf_1 = FlightConf(uavconf, start_t, base_wp_1)
    # fgconf_2 = FlightConf(uavconf, start_t, base_wp_2)

    # Write down the desired VNS configuration
    conf_vns = {
        "full": {
            "max_restarts": 5,
            "max_time": 20.0,
            "neighborhoods": [
                {"name": "dubins-opt",
                 "max_trials": 100,
                 "generators": [
                     {"name": "MeanOrientationChangeGenerator"},
                     {"name": "RandomOrientationChangeGenerator"},
                     {"name": "FlipOrientationChangeGenerator"}]},
                {"name": "one-insert",
                 "max_trials": 100,
                 "select_arbitrary_trajectory": True,
                 "select_arbitrary_position": False},
            ]
        }
    }

    conf = {
        'min_time': fgconf_1.start_time,
        'max_time': fgconf_1.start_time + fgconf_1.uav.max_flight_time,
        'save_every': 1,
        'save_improvements': True,
        'discrete_elevation_interval': 0,
        'vns': conf_vns['full']
    }
    conf['vns']['configuration_name'] = 'full'

    ####################################
    # 1st PLAN
    fire1 = fire.ignitions()
    # pl = Planner(env, fire1, [fgconf_1, fgconf_2], conf)
    pl = Planner(env, fire1, [fgconf_1], conf)
    pl.compute_plan()
    sr_1 = pl.search_result

    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire1),
                                                               frame=(0, 0))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})
    gdd.draw_elevation_shade()
    gdd.draw_ignition_contour()
    plot_plan_trajectories(pl.search_result.final_plan(), gdd, trajectories=slice(None),
                           colors=["blue", "darkgreen"],
                           layers=["bases", "arrows", "trajectory_solid"])
    gdd.figure.show()

    import os

    for i in range(len(sr_1.intermediate_plans)):
        int_gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(
            env.raster.combine(fire1), frame=(0, 0))
        int_gdd.add_extension(TrajectoryDisplayExtension, (None,), {})
        # int_gdd.draw_elevation_shade()
        int_gdd.draw_ignition_contour()
        plot_plan_trajectories(sr_1.intermediate_plans[i], int_gdd, trajectories=slice(None),
                               colors=["blue", "darkgreen"],
                               layers=["bases", "arrows", "trajectory_solid"])

        i_plan_dir = "./intermediateplans"
        if not os.path.exists(i_plan_dir):
            os.makedirs(i_plan_dir)

        print("saving as: " + str(
            os.path.join(i_plan_dir, str(i) + "." + "svg|png")))
        # int_gdd.axes.get_figure().savefig(str(
        #     os.path.join(i_plan_dir, str(i) + "." + "svg")), bbox_inches='tight')
        int_gdd.axes.get_figure().savefig(str(
            os.path.join(i_plan_dir, str(i) + "." + "png")), bbox_inches='tight', dpi=150)
        matplotlib.pyplot.close(int_gdd.axes.get_figure())

    print(
        "TO CREATE A VIDEO: ffmpeg -framerate 12 -f image2 -i %d.png -c:v h264 -crf 1 plan_animation.mp4")

    # OBSERVED FIRE
    fmapper_1 = FireMapper(env, fire1)  # fire3 is the ground thruth

    executed_path = sr_1.final_plan().trajectories()[0].sampled_with_time()
    fmapper_1.observe(executed_path, pl.flights[0].uav)
    observedfire_figure = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster,
                                                                               frame=(0, 0))
    observedfire_figure.draw_elevation_shade()
    observedfire_figure.draw_ignition_shade(geodata=fmapper_1.firemap, cmap=matplotlib.cm.Greens)
    if len(sr_1.final_plan().trajectories()) > 1:
        executed_path = sr_1.final_plan().trajectories()[1].sampled_with_time()
        fmapper_2 = FireMapper(env, fire1)
        fmapper_2.observe(executed_path, pl.flights[1].uav)
        observedfire_figure.draw_ignition_shade(geodata=fmapper_2.firemap, cmap=matplotlib.cm.Blues)

    observedfire_figure.figure.show()
    print()

    # EXECUTION!
    import threading
    import fire_rs.neptus_interface as nifc

    f1 = lambda x: print(x)
    f2 = lambda x: None

    a_comm = nifc.IMCCommManager()
    a_th = threading.Thread(target=a_comm.run, daemon=True)
    a_th.run()
    print("IMCCommManager esta funcionando")
    a_pem = nifc.PlanExecutionManager(a_comm, f1, f2)
    print("PlanExecutionManager esta funcionando")
    a_pem.start(pl.search_result.final_plan())
    print("wait for it :)")
    print("bye bye")
