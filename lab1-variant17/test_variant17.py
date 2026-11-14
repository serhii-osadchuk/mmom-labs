"""Автотест: перевіряє критерій успіху завдання для варіанта 17.

Запуск:  python -m unittest -v
"""
import unittest

from run_variant17 import run, D_MIN


class Variant17Test(unittest.TestCase):
    def test_main_configuration_meets_requirements(self):
        sim, res = run(use_right_hand=True, use_yield=False)
        self.assertTrue(res.all_reached, "не всі дрони досягли цілей")
        self.assertGreaterEqual(res.min_pair, D_MIN, "порушено d_min між дронами")
        self.assertGreaterEqual(res.min_clearance, D_MIN, "порушено d_min до перешкоди")

    def test_time_close_to_lower_bound(self):
        sim, res = run(use_right_hand=True, use_yield=False)
        from swarm import Params
        import numpy as np
        shortest = max(float(np.sum(np.linalg.norm(np.diff(d.path, axis=0), axis=1)))
                       for d in sim.drones)
        lower_bound = shortest / Params().v_max
        self.assertLess(res.makespan, lower_bound * 1.03)   # не більше +3 % до межі


if __name__ == "__main__":
    unittest.main()
