# Автор: Осадчук Сергій, аспірант 1 р. н., ЗУНУ (спец. G7), 2026
"""Чому саме такі правки: (1) розгортка margin планувальника, (2) розгортка параметрів відштовхування і правої руки.
Запуск: python sensitivity.py  ->  results/sensitivity.json"""
import itertools
import json
from pathlib import Path
import astar_apf_variant17 as v


def row(**kw):
    r = v.run_simulation(**kw)
    return {"t_arrive_s": r["t_arrive"], "makespan_s": r["makespan"],
            "min_pair_m": round(r["min_pair_m"], 2), "min_clearance_m": round(r["min_clearance_m"], 2),
            "success": r["success"]}


out = {"margin": {}, "repulsion": {}}
print(f"1) margin планувальника (вимога: просвіт до перешкоди >= d_min = {v.D_MIN} м)")
for m in (2.0, 2.5, 3.0, 4.0):
    out["margin"][str(m)] = row(planner_margin=m)
    print(f"  margin={m}: {out['margin'][str(m)]}")
print("2) відштовхування між дронами: r_safe, k_rep, k_right (margin = 3.0); None у часі = дрон застряг")
for rs, kr, kright in itertools.product((2.5, 4.5, 8.0), (3.0, 12.0, 40.0), (0.0, 10.0)):
    key = f"r_safe={rs}, k_rep={kr}, k_right={kright}"
    out["repulsion"][key] = row(r_safe=rs, k_rep=kr, k_right=kright)
    print(f"  {key}: {out['repulsion'][key]}")
(Path(__file__).parent / "results").mkdir(exist_ok=True)
(Path(__file__).parent / "results" / "sensitivity.json").write_text(
    json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
