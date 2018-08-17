from morse.builder.creator import ActuatorCreator

class AbsoluteTeleport(ActuatorCreator):
    _classpath = "morse_sim.actuators.absolute_teleport.AbsoluteTeleport"
    _blendname = "AbsoluteTeleport"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

