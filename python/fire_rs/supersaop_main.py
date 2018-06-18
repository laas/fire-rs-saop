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

    import logging
    import fire_rs.geodata.geo_data as geo_data
    import fire_rs.planning.planning as cpp_planning
    import fire_rs.geodata.display as gdisplay
    import fire_rs.monitoring.supersaop as supersaop

    # Set up logging
    FORMAT = '%(asctime)-23s %(levelname)-8s [%(name)s]: %(message)s'
    logging.basicConfig(format=FORMAT)

    logger = logging.getLogger("__main__")
    logger.setLevel(logging.DEBUG)

    logger_up = logger.getChild("saop_cpp")
    cpp_planning.up.set_logger(logger_up)

    # Start Situation Assessment
    an_alarm = supersaop.poll_alarm()
    logger.info("Alarm raised: %s", an_alarm)

    the_hangar = supersaop.Hangar()

    situation_assessment = supersaop.SituationAssessment(the_hangar,
                                                         logger.getChild("SituationAssessment"))

    observation_planning = supersaop.ObservationPlanning(the_hangar,
                                                         logger.getChild("ObservationPlanning"))

    environment, firepropagation = situation_assessment.expected_situation(an_alarm)

    # Show wildfire Situation Assessment
    alarm_figure = gdisplay.get_pyplot_figure_and_axis()
    alarm_gdd = gdisplay.GeoDataDisplay(*gdisplay.get_pyplot_figure_and_axis(),
                                        environment.raster, frame=(0., 0.))
    supersaop.show_situation(alarm_gdd, an_alarm, environment, firepropagation, the_hangar)
    alarm_gdd.figure.show()

    # Observation Planning
    response = observation_planning.respond_to_alarm(
        an_alarm, expected_situation=(environment, firepropagation))

    execution_monitor = supersaop.ExecutionMonitor(logger.getChild("ExecutionMonitor"))
    if execution_monitor.start_response(response) or True:
        for report in execution_monitor.monitor(response):
            print(str(report))
    else:
        logger.error("Execution of plan failed")
