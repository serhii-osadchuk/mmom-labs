# Автор: Осадчук Сергій, аспірант 1 року навчання (спеціальність G7), ЗУНУ, Тернопіль
"""Автотест варіанта 17: властивості мережі Петрі (безпека, завершеність).
Запуск:  python -m unittest -v
"""
import os
os.environ["MPLBACKEND"] = "Agg"
import unittest
import matplotlib.pyplot as plt
import pn_abm_variant17 as v


class Variant17Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        plt.show = lambda *a, **k: None
        v.SNAPSHOT_PREFIX = None
        cls.sup, cls.amrs = v.run_production_simulation()

    def test_cz_never_exceeds_capacity(self):
        ok_cz, max_in_cz, _ = v.check_invariants(self.sup)
        self.assertTrue(ok_cz)
        self.assertLessEqual(max_in_cz, v.CZ_CAPACITY)

    def test_all_machines_finished(self):
        _, _, done = v.check_invariants(self.sup)
        self.assertTrue(done)
        self.assertTrue(all(a.state == 'DONE' for a in self.amrs))

    def test_third_robot_had_to_wait(self):      # K = 2, роботів 3 -> хтось чекає
        self.assertGreater(self.amrs[2].wait_time, 0.0)


if __name__ == "__main__":
    unittest.main()
