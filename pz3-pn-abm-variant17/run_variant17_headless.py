"""Безголовий запуск варіанта 17: зберігає кадри в results/ та друкує журнал переходів і перевірки."""
import os
os.environ["MPLBACKEND"] = "Agg"
import matplotlib.pyplot as plt
import pn_abm_variant17 as v

here = os.path.dirname(os.path.abspath(__file__))
v.SNAPSHOT_PREFIX = os.path.join(here, "results", "snapshot")
plt.show = lambda *a, **k: None
sup, amrs = v.run_production_simulation()

print(f"{'t, с':>6s}  {'перехід':20s}  маркування M")
for t, tid, m in sup.log:
    print(f"{t:6.1f}  {tid:20s}  {m.tolist()}")
ok_cz, max_in_cz, done = v.check_invariants(sup)
print("\nІнваріант 'місць у CZ = K' виконується завжди:", ok_cz)
print("Максимум роботів у CZ одночасно:", max_in_cz, "(K =", v.CZ_CAPACITY, ")")
print("Усі верстати обробили по одній деталі:", done)
print("Час очікування дозволу, с:", {f"AMR{a.id}": round(a.wait_time, 1) for a in amrs})
print("Кінцеві стани:", {f"AMR{a.id}": a.state for a in amrs})
