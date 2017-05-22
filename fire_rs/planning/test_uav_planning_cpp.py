import unittest
import fire_rs.uav_planning as up


class TestUAV(unittest.TestCase):

    def setUp(self):
        self.wp1 = up.Waypoint(4, 0, 0)

    def test_waypoint(self):
        print(self.wp1)
        self.assertEquals(self.wp1.x, 4)
        self.assertEquals(self.wp1.y, 0)
        self.assertEquals(self.wp1.dir, 0)

