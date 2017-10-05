import scipy.constants
import matplotlib.pyplot as plt
import numpy as np


class UAV:
    """A very simple UAV model defined by its speed limits (airspeed, climb and descent)."""

    def __init__(self, min_airspeed, max_airspeed, max_rate_of_climb, max_rate_of_descent):
        """All speeds are in m/s"""
        self.min_airspeed = min_airspeed
        self.max_airspeed = max_airspeed
        self.max_rate_of_climb = max_rate_of_climb
        self.max_rate_of_descent = max_rate_of_descent

    def travel_time(self, origin, destination):
        """Returns the travel time (s) to go from a a point 'origin' to a point 'destination'.
    
        Both origin and destination are of the form (x, y) or (x, y, z) 
        """
        assert 2 <= len(origin) <= 3, "Origin should by (x, y) or (x, y, z)"
        assert 2 <= len(destination) <= 3, "Origin should by (x, y) or (x, y, z)"
        assert len(origin) == len(destination), "Elevation should be present in origin and destination or absent in both"
        if len(origin) == 2:
            xo, yo = origin
            xd, yd = destination
            zo = zd = 0
        else:
            assert len(origin) == 3
            xo, yo, zo = origin
            xd, yd, zd = destination

        ground_distance = np.sqrt((xd-xo)**2 + (yd-yo)**2)
        elevation_diff = zd - zo
        if elevation_diff >= 0:
            return max(ground_distance / self.max_airspeed, elevation_diff / self.max_rate_of_climb)
        else:
            return max(ground_distance / self.max_airspeed, -elevation_diff / self.max_rate_of_descent)

    def trajectory_length(self, trajectory):
        duration = 0
        for i in range(1, len(trajectory)):
            duration += self.travel_time(trajectory[i - 1], trajectory[i])
        return duration


class DubinsUAV2D(UAV):
    """Fixed-wing plane trajectory generator modelled as a Dubins car.
    
    Assumptions:
        - The UAV is always travelling full-speed.
        - The UAV always turns at the maximum rate.
        - The UAV follows a planar trajectory.
    """

    def __init__(self, max_turn_rate, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.max_turn_rate = max_turn_rate  # ψ̇ in rad/s
        self.min_turn_radius = self.max_airspeed / self.max_turn_rate  # R_min in m

    @staticmethod
    def _sawtooth(x, d=1):
        """Get value sawtooth-like function for x.
        d=1 -> to the left
        d=-1 -> to the right
        """
        assert d == 1 or d == -1
        return d * np.pi + np.mod(x + np.pi - d * np.pi, 2 * np.pi) - np.pi

    @staticmethod
    def _angle(*vectors):
        """Calculate the angle between two vectors"""
        if len(vectors) == 1:
            return DubinsUAV2D._sawtooth(np.arctan2(vectors[0][1], vectors[0][0]))
        elif len(vectors) == 2:
            return DubinsUAV2D._sawtooth(np.arctan2(vectors[1][1], vectors[1][0]) - np.arctan2(vectors[0][1], vectors[0][0]))
        else:
            raise AttributeError()

    @staticmethod
    def _draw_arc(c, a, theta, **kwargs):
        """Draw a circunference arc.
        c: center
        a: starting point of the arc
        theta: end angle
        **kwargs: other arguments to plt.plot"""
        s = np.arange(0, abs(theta), 0.01)
        s = np.sign(theta) * s
        d = a - c
        r = np.linalg.norm(d)
        alpha = DubinsUAV2D._angle(d)
        w = np.empty((len(s), 2))
        for i, t in enumerate(s):
            w[i] = c + r * np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]]) @ np.array(
                    [np.cos(t), np.sin(t)])
        plt.plot(w[:, 0], w[:, 1], **kwargs)

    def plot_dubins(self, origin, destination, r, co, epsa, do, betao, cd, epsb, dd, betad):
        plt.plot([co[0], cd[0]], [co[1], cd[1]], 'x', color='lightgrey')
        DubinsUAV2D._draw_arc(co, origin[0:2], betao, color='darkcyan')
        DubinsUAV2D._draw_arc(cd, destination[0:2], -betad, color='blue')
        plt.plot([do[0], dd[0]],[do[1], dd[1]], color='blueviolet')

    def travel_time(self, origin, destination):
        """Return the travel time (s) to go from a point 'origin' to a point 'destination'.
        
        Both origin and destination are of the form (x, y, z, ψ), where x, y, z are position 
        coordinates and ψ a direction.
        """
        dubins_paths = ["LSL", "LSR", "RSL", "RSR"]

        # if dist(o, d) < 4*r_min CCC paths could be optimal
        if np.linalg.norm(destination[0:2] - origin[0:2]) < 4 * self.min_turn_radius:
            dubins_paths.extend(["RLR", "LRL"])

        dubins_lenght = np.empty(len(dubins_paths))
        shortest_path = (np.inf, )
        for i, candidate in enumerate(dubins_paths):
            if candidate[1].lower() == 's':
                d = self.create_csc_trajectory(origin, destination, rot_orig=candidate[0], rot_dest=candidate[2])
                if d[0] < shortest_path[0]:
                    shortest_path = d
                dubins_lenght[i] = d[0]
                # self.plot_dubins(origin, destination, *d[1:])
            else:  # candidate[1] != 's'
                dubins_lenght[i] = np.inf
                # TODO implement create_ccc_trajectory

        return shortest_path[0], shortest_path[1:]

    def create_csc_trajectory(self, origin, destination, rot_orig='l', rot_dest='l'):
        """Calculate the lenght of a Dubins path of CSC (Circle, Straight line, Circle) type."""

        assert len(origin) == 4
        assert len(destination) == 4

        assert rot_orig.lower() == 'l' or rot_orig.lower() == 'r'
        assert rot_dest.lower() == 'l' or rot_dest.lower() == 'r'

        epsa = 1 if rot_orig.lower() == 'l' else -1
        epsb = 1 if rot_dest.lower() == 'l' else -1

        r = self.min_turn_radius

        po = origin[0:2]
        po_ang = origin[3]
        po_vec = (np.cos(po_ang), np.sin(po_ang))
        pd = destination[0:2]
        pd_ang = destination[3]
        pd_vec = (np.cos(pd_ang), np.sin(pd_ang))

        # po_vec and pd_vec are rotated ±90º in order to find the direction of the circle center
        co = po + epsa * r * np.array([-np.sin(po_ang), np.cos(po_ang)])  # po_vec is rotated ±90º
        cd = pd + epsb * r * np.array([-np.sin(pd_ang), np.cos(pd_ang)])  # pd_vec is rotated ±90º

        # plt.plot([co[0], cd[0]], [co[1], cd[1]], 'x')
        # plt.plot([po[0], pd[0]], [po[1], pd[1]], 'o')

        alpha = 0  #
        ell = 0  # Half the length of the straight part
        if np.sign(epsa) != np.sign(epsb): # LSR or RSL
            ell2 = 0.25 * np.linalg.norm(cd - co)**2 - r**2
            if ell2 < 0:
                return np.inf, None
            ell = np.sqrt(ell2)
            alpha = -epsa * np.arctan2(ell, r)

        if epsa * epsb == 1: # RSR or LSL
            ell = 0.5 * np.linalg.norm(cd - co)
            alpha = -epsa * np.pi / 2

        rot = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]])
        do = co + (r / (np.linalg.norm(cd - co))) * rot @ (cd - co)
        dd = cd + epsa * epsb * (do - co)
        betao = DubinsUAV2D._sawtooth(DubinsUAV2D._angle(po - co, do - co), epsa)
        betad = DubinsUAV2D._sawtooth(DubinsUAV2D._angle(dd - cd, pd - cd), epsb)

        L = r * (abs(betad) + abs(betao) + 2 * ell)

        # DubinsUAV2D._draw_arc(co, po, betao,'black')
        # DubinsUAV2D._draw_arc(cd, pd, -betad,'blue')
        # plt.plot([do[0], dd[0]],[do[1], dd[1]], 'r')

        # returns:
        #   L: lenght of the trajectory
        #   r: radius of both circumferences
        #   co: center of origin circumference
        #   epsa: rotation orientation of origin circumference (1 = left or -1 = right
        #   do: switching point between origin circ and line
        #   betao: angle of do
        #   cd: center of destination circumference
        #   epsb: rotation orientation of destination circumference  (1 = left or -1 = right
        #   dd: switching point between line and destination circ
        #   betad: angle of dd
        return L, r, co, epsa, do, betao, cd, epsb, dd, betad


class _FixedWingRollRate:
    """2D FixedWing kinematic model commanded by roll rate.
    
        ⎛ẋ₁⎞   ⎛ẋ⎞   ⎛ V·cos(ψ) ⎞   ⎛0⎞  
    Ẋ = ⎜ẋ₂⎟ = ⎜ẏ⎟ = ⎜ V·sin(ψ) ⎟ + ⎜0⎟ · u₁
        ⎜ẋ₃⎟   ⎜ψ̇⎟   ⎜g/V·tan(ϕ)⎟   ⎜0⎟
        ⎝ẋ₄⎠   ⎝ϕ̇⎠   ⎝    0     ⎠   ⎝1⎠
        
    State is position_x, position_y, heading and roll angles
    Input is roll rate 
    Output == state
    
    V is fix and roll is bounded
    """

    def __init__(self, velocity, roll_max, initial_state=np.array([0,0,0,0])):
        assert len(initial_state) == 4
        assert velocity != 0

        self.velocity = velocity
        self.roll_max = roll_max

        self.state = initial_state
        self.input = np.array([0])
        self.output = self.state

    def _xdot(self):
        return np.array([self.velocity * np.cos(self.state[2]),
                         self.velocity * np.sin(self.state[2]),
                         scipy.constants.g / self.velocity * np.tan(self.state[3]),
                         self.input[0]])

    def step(self, delta):
        self.state = self.state + delta * self._xdot()
        self.state[3] = np.clip(self.state[3], -self.roll_max, self.roll_max)
        self.output = self.state

    def __str__(self):
        return str(", ".join([": ".join((a, "{0: .5f}".format(b))) for a, b in zip(('x','y','ψ','ϕ'), self.state)]))


class FixedWing:
    """2D fixed wing plane with heading as input
    
    Using feedback linearization on the heading:
    ψ̈ = g/V·(1+tan²(ϕ))·ϕ̇ = g/V·(1+tan²(ϕ))·u₁
    ψ̈ = v
    u₁ = (g/V·(1+tan²(ϕ)))⁻¹·v
    
    and then using a proportional-derivative controller on the linearized system.
    
      ----------- u -----------  y
      | F. Lin  |--→|  Plane  |----|
      -----------   -----------    |
           ↑                       |
           |   v   --------------  |
           |-------| Controller |←-|
                   -------------- 
                         ↑
                         Desired Heading
    """

    def __init__(self, velocity, roll_max, initial_state=np.array([0.,0.,0.,0.])):
        self.fixedwing = _FixedWingRollRate(velocity, roll_max, initial_state)  # [x, y, ψ, ϕ]
        self.state = self.fixedwing.state
        self.input = np.array([0.])  # desired heading, (1st derivative & second derivative will be always zero)
        self.output = self.fixedwing.state  # [x, y, ψ, ϕ]

    def _xdot(self):
        y = self.fixedwing.state[2]  # ψ
        y_dot = scipy.constants.g / self.fixedwing.velocity * np.tan(self.fixedwing.state[3])
        w = self.input[0]
        w_dot = 0
        w_dotdot = 0

        v = (w - y) + 2*(w_dot-y_dot) + w_dotdot
        # v = y_dotdot

        u = self.fixedwing.velocity/(scipy.constants.g*(1+np.tan(self.fixedwing.state[3])**2)) * v
        return np.array([u,])

    def step(self, delta):
        self.fixedwing.input = self._xdot()
        self.fixedwing.step(delta)
        self.state = self.fixedwing.state
        self.output = self.fixedwing.state

    def run(self, delta, n_iterations=np.inf):
        n = 0
        while n < n_iterations:
            self.step(delta)
            n += 1
            yield self.output

    def reset_state(self, default_state=np.array([0.,0.,0.,0.])):
        self.fixedwing.state = default_state
        self.state = self.fixedwing.state
        self.fixedwing.output = default_state
        self.output = self.fixedwing.output

    def __str__(self):
        return str(self.fixedwing)

    def copy(self):
        new_uav = FixedWing(velocity=self.fixedwing.velocity,
                            roll_max=self.fixedwing.roll_max,
                            initial_state=self.fixedwing.state.copy())
        new_uav.output = self.output.copy()
        new_uav.input = self.input.copy()
        return new_uav

# the X8 Skywalker UAV from PORTO
skywalker = UAV(10, 20, 2, 3)


if __name__ == "__main__":
    def dubins_demo():
        dubins_uav = DubinsUAV2D(1, 10, 20, 2, 3)
        fig = plt.figure(0)
        ax = fig.add_subplot(111, aspect='equal')
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)
        print(dubins_uav.create_csc_trajectory((0,50,0,np.pi/2), (50,0,0,3*np.pi/2), 'r', 'r'))
        print(dubins_uav.travel_time(np.array((0, 0, 0, np.pi / 2)), np.array((50, 0, 0, 3 * np.pi / 2))))

    def uav_controlled_demo():
        import matplotlib.pyplot as plt
        avion_cont = FixedWing(10, np.pi / 6)
        t_end = 500
        estados = np.zeros((t_end, 4))
        u = np.pi*np.sin(0.01*np.arange(t_end))
        for i in range(300):
            avion_cont.input = np.array([u[i]])
            avion_cont.step(0.1)
            estados[i] = avion_cont.output
            print(str(avion_cont))
        for i in range(300, 500):
            avion_cont.input = np.array([u[i]])
            avion_cont.step(0.1)
            estados[i] = avion_cont.output
            print(str(avion_cont))
        fig = plt.figure(0)
        ax = fig.add_subplot(121, aspect='equal')
        ax.set_xlim(-200, 200)
        ax.set_ylim(-200, 200)
        plt.plot(estados[:, 0], estados[:, 1])
        ax = fig.add_subplot(122)
        plt.plot(estados[:, 2])
        plt.plot(u)

        plt.show()

    uav_controlled_demo()