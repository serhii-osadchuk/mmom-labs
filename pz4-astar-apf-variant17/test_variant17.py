# Автор: Осадчук Сергій, аспірант 1 р. н., ЗУНУ (спец. G7), 2026
"""Автотест варіанта 17: усі дрони прибули, просвіт до перешкод >= r_safety, A*-шляхи не зачіпають перешкод."""
import unittest
import numpy as np
import astar_apf_variant17 as v


class Variant17(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = v.run_simulation()

    def test_all_arrived(self):
        self.assertTrue(all(t is not None for t in self.res["t_arrive"]), self.res["t_arrive"])

    def test_clearance_not_below_r_safety(self):
        self.assertGreaterEqual(self.res["min_clearance_m"], v.R_SAFETY)

    def test_astar_nodes_outside_inflated_obstacles(self):
        for d in self.res["drones"]:
            for x, y in d.path[1:-1]:
                for ox, oy, r in v.OBSTACLES:
                    self.assertGreater(np.hypot(x - ox, y - oy), r + v.PLANNER_MARGIN)

    def test_goal_is_last_waypoint(self):
        for d in self.res["drones"]:
            self.assertTrue(np.allclose(d.path[-1], d.goal))

    def test_without_margin_compensation_requirement_fails(self):
        """Підтверджує потребу правки C2: з margin = r_safety (як у прототипі) вимога порушується."""
        r = v.run_simulation(planner_margin=v.R_SAFETY, max_steps=700)
        self.assertLess(r["min_clearance_m"], v.R_SAFETY)


if __name__ == "__main__":
    unittest.main()
