# Автор: Осадчук Сергій, аспірант 1 р. н., ЗУНУ (спец. G7), 2026
"""Чому саме такі правки: (1) розгортка margin планувальника, (2) розгортка зони/ваги відштовхування між дронами.
Запуск: python sensitivity.py  ->  results/sensitivity.json"""
import json
from pathlib import Path
import astar_apf_variant17 as v


def row(**kw):
    r = v.run_simulation(**kw)
    return {"makespan_s": r["makespan"], "t_arrive_s": r["t_arrive"],
            "min_pair_m": round(r["min_pair_m"], 2), "min_clearance_m": round(r["min_clearance_m"], 2),
            "success": r["success"]}


out = {"margin": {}, "repulsion": {}}
print("1) margin планувальника (вимога: просвіт >= r_safety = 1.1 м)")
for m in (1.1, 1.5, 2.1, 2.5):
    out["margin"][m] = row(planner_margin=m, max_steps=700)
    print(f"  margin={m}: {out['margin'][m]}")
print("2) відштовхування між дронами (margin = 2.1)")
for rs in (2.5, 3.5, 4.5):
    for g in (3.0, 6.0, 12.0):
        key = f"r_safe={rs}, k_rep={g}"
        out["repulsion"][key] = row(r_safe=rs, k_rep=g, max_steps=700)
        print(f"  {key}: {out['repulsion'][key]}")
Path(__file__).with_name("results").mkdir(exist_ok=True)
(Path(__file__).parent / "results" / "sensitivity.json").write_text(
    json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
