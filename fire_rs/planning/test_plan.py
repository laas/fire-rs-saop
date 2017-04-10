import unittest

from fire_rs.planning.plan import Plan
from fire_rs.planning.uav import UAV


class TestPlan(unittest.TestCase):

    def setUp(self):
        self.uav = UAV(1, 1, 1, 1)

    def test_plan_incremental_duration(self):
        plan = Plan(self.uav, [(0, 0), (0,10), (0, 20), (0, 0)])
        self.check_duration(plan)

        p2 = plan.remove(2)
        self.assertEquals(p2.length, plan.length - 1)
        self.check_duration(plan)
        self.check_duration(p2)

        p3 = p2.extend((30, 5), 3)
        self.assertEquals(p3.length, p2.length + 1)
        self.check_duration(plan)
        self.check_duration(p2)
        self.check_duration(p3)

        for p in [plan, p2, p3]:
            print(p.duration)

    def check_duration(self, plan):
        self.assertAlmostEquals(plan.duration, plan.uav.trajectory_length(plan.trajectory))
