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


if __name__ == "__main__":
    import threading
    import fire_rs.neptus_interface as nifc


    def give_me_a_plan():
        import numpy as np
        import fire_rs.uav_planning as op
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
        fire = propagation.propagate_from_points(env, ignition_point, 120 * 60)  # 60 minutes
        # Configure some flight
        base_wp_1 = Waypoint(area[0][1] - 150., area[1][1] - 100., 0., 0.)
        start_t = 60 * 60  # 30 minutes after the ignition
        uavconf = UAVConf.X8()
        uavconf.max_flight_time = 30 * 60
        fgconf_1 = FlightConf(uavconf, start_t, base_wp_1)

        # Write down the desired VNS configuration
        conf_vns = {
            "full": {
                "max_restarts": 5,
                "max_time": 10.0,
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
        pl = Planner(env, fire1, [fgconf_1], conf)
        pl.compute_plan()
        return pl.search_result.final_plan()


    f1 = lambda x: print(x)
    f2 = lambda x: None

    imccomm = nifc.IMCComm()
    gcs = None


    def gcs_run():
        global gcs
        gcs = nifc.GCS(imccomm, f1, f2)


    t_imc = threading.Thread(target=imccomm.run, daemon=True)
    t_gcs = threading.Thread(target=gcs_run, daemon=True)
    t_imc.run()
    t_gcs.run()
    print("IMCComm esta funcionando")

    a_plan = give_me_a_plan()
    k = nifc.GCSCommandOutcome.Unknown
    retries = 2
    while retries > 0 and k != nifc.GCSCommandOutcome.Success:
        k = gcs.load(a_plan, 0, a_plan.name(), "x8-02")
        if k != nifc.GCSCommandOutcome.Success:
            print("Load plan failed")
            print("Retrying")
            retries -= 1
        else:
            print("Plan loaded")
    k = gcs.start("saop_" + a_plan.name(), "x8-02")
    if k != nifc.GCSCommandOutcome.Success:
        print("Start plan failed")
    else:
        print("Plan started")

    while True:
        lll = input("e for exit")
        if lll == "e":
            break
