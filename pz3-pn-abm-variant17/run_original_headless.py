"""Безголовий запуск ОРИГІНАЛЬНОГО прототипу: зберігає кінцевий кадр у PNG і друкує журнал переходів.
Сам оригінал не змінюється (patch лише в цьому скрипті)."""
import os, sys
os.environ["MPLBACKEND"] = "Agg"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "original"))
import matplotlib.pyplot as plt
import pn_abm_original as orig

log, holder = [], {}
_orig_trigger = orig.PetriNetSupervisor.trigger
def traced(self, tid):
    holder["sup"] = self
    ok = _orig_trigger(self, tid)
    if ok:
        log.append((tid, self.M.tolist()))
    return ok
orig.PetriNetSupervisor.trigger = traced

out = os.path.join(os.path.dirname(__file__), "results", "original_final_frame.png")
plt.show = lambda *a, **k: plt.gcf().savefig(out, dpi=150)
orig.run_production_simulation()
print("Спрацьовані переходи (перехід -> маркування M після нього):")
for tid, m in log:
    print(f"  {tid:20s} -> {m}")
print("Кінцеве маркування M =", holder["sup"].M.tolist())
print("Кадр збережено:", out)
