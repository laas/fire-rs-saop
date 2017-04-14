

from fire_rs.planning.uav import UAV


class Plan:
    """Immutable data structure that records a trajectory for a given UAV as a sequence of waypoints"""

    def __init__(self, uav: 'UAV', waypoints, duration=None):
        self.uav = uav
        self.waypoints = list(waypoints)  # make a copy to avoid any mutation of the plan
        self.duration = duration

        # duration not provided, compute it based on the UAV model
        if self.duration is None:
            self.duration = self.uav.trajectory_length(self.waypoints)

    @property
    def length(self):
        return len(self.waypoints)

    def __repr__(self):
        return "duration: {} -- length: {} -- {}".format(round(self.duration, 2), self.length, self.waypoints)

    def __getitem__(self, item):
        return self.waypoints[item]

    def extend(self, waypoint, position_in_plan: int) -> 'Plan':
        """Builds a new Plan by inserting a new waypoint at the given position in the plan.
         The current plan is not modified and a new one is returned.
         """
        assert 0 <= position_in_plan <= len(self.waypoints)

        # incrementally compute new duration
        new_duration = self.duration
        if len(self.waypoints) == 0:
            pass  # only one point in the resulting trajectory, duration is still 0
        elif position_in_plan == 0:
            new_duration += self.uav.travel_time(waypoint, self.waypoints[0])
        elif position_in_plan == len(self.waypoints):
            new_duration += self.uav.travel_time(self.waypoints[-1], waypoint)
        else:
            new_duration -= self.uav.travel_time(self.waypoints[position_in_plan - 1], self.waypoints[position_in_plan])
            new_duration += self.uav.travel_time(self.waypoints[position_in_plan - 1], waypoint)
            new_duration += self.uav.travel_time(waypoint, self.waypoints[position_in_plan])
        new_points = list(self.waypoints)
        new_points.insert(position_in_plan, waypoint)

        return Plan(self.uav, new_points, new_duration)

    def remove(self, position_in_plan) -> 'Plan':
        """Builds a new Plan by removing the waypoint at the given position in the plan.
        The current plan is not modified and a new one is returned.
        """
        assert 0 <= position_in_plan < len(self.waypoints)

        # incrementally compute new duration
        new_duration = self.duration
        if len(self.waypoints) == 1:
            pass  # only one point in the resulting trajectory, duration is still 0
        if position_in_plan == 0:
            new_duration -= self.uav.travel_time(self.waypoints[0], self.waypoints[1])
        elif position_in_plan == len(self.waypoints) -1:
            new_duration -= self.uav.travel_time(self.waypoints[-2], self.waypoints[-1])
        else:
            new_duration -= self.uav.travel_time(self.waypoints[position_in_plan - 1],
                                                 self.waypoints[position_in_plan])
            new_duration -= self.uav.travel_time(self.waypoints[position_in_plan],
                                                 self.waypoints[position_in_plan + 1])
            new_duration += self.uav.travel_time(self.waypoints[position_in_plan - 1],
                                                 self.waypoints[position_in_plan + 1])

        new_points = list(self.waypoints)
        del new_points[position_in_plan]

        return Plan(self.uav, new_points, new_duration)

    def plot(self, axes=None, blocking=False):
        """Plot the current plan on the given axes (or a new one if None is given).
        Returns the axes on which the plan was drawn and the patch drawn."""
        from matplotlib import patches
        from matplotlib.path import Path
        import matplotlib.pyplot as plt
        path = Path(self.waypoints)
        patch = patches.PathPatch(path, facecolor='none', lw=2)
        if axes is None:
            # no axes given, generate our own
            fig = plt.figure()
            axes = fig.add_subplot(111)
            xs = [x for (x, y) in self.waypoints]
            ys = [y for (x, y) in self.waypoints]
            axes.set_xlim(min(xs), max(xs))
            axes.set_ylim(min(ys), max(ys))
        axes.add_patch(patch)

        plt.show(block=blocking)
        return axes, patch
