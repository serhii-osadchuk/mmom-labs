# Автор: Осадчук Сергій, аспірант 1 р. н., ЗУНУ (спец. G7), 2026
"""Запуск варіанта 17 (A* + APF, 3 дрони): графік, показники, results/metrics.json.
Запуск: python run_variant17.py"""
import json
import os
os.environ.setdefault("MPLBACKEND", "Agg")
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import astar_apf_variant17 as v

OUT = Path(__file__).parent / "results"
COLORS = ['tab:blue', 'tab:green', 'tab:orange']


def draw(res, fname, title):
    plt.figure(figsize=(9, 9))
    plt.title(title, fontsize=11, fontweight='bold')
    plt.xlim(0, v.GRID_SIZE[0]); plt.ylim(0, v.GRID_SIZE[1])
    plt.grid(True, linestyle='--', alpha=0.5)
    for ox, oy, r in res["obstacles"]:
        plt.gca().add_patch(plt.Circle((ox, oy), r, color='darkred', alpha=0.6))
        # крапки - межа вимоги d_min (радіус + d_min)
        plt.gca().add_patch(plt.Circle((ox, oy), r + res["d_min"], fill=False, ls=':', color='darkred'))
    for i, d in enumerate(res["drones"]):
        h, a = np.array(d.history), np.array(d.path)
        plt.plot(a[:, 0], a[:, 1], '--', color=COLORS[i], alpha=0.4, label=f'A* План Дрон {d.id}')
        plt.plot(h[:, 0], h[:, 1], color=COLORS[i], linewidth=2.5, label=f'Траєкторія Дрон {d.id}')
        plt.plot(h[0][0], h[0][1], 'o', color=COLORS[i], markersize=8)
        plt.plot(d.goal[0], d.goal[1], '*', color=COLORS[i], markersize=12)
    plt.xlabel("X (метри)"); plt.ylabel("Y (метри)")
    plt.legend(loc='upper right', fontsize=8)
    plt.gca().set_aspect('equal')
    plt.savefig(OUT / fname, dpi=130, bbox_inches='tight')
    plt.close()


def main():
    OUT.mkdir(exist_ok=True)
    res = v.run_simulation()
    draw(res, "variant17_final.png", "Варіант 17: 3 дрони, A* + APF, перешкода R=20 (крапки: R + d_min)")
    print("Час прибуття дронів, с:", res["t_arrive"], "| загальний час:", res["makespan"])
    print(f"Довжини A*-шляхів, м: {[round(x, 1) for x in res['astar_len_m']]}")
    print(f"Мін. відстань між дронами: {res['min_pair_m']:.2f} м (вимога d_min = {v.D_MIN} м)")
    print(f"Мін. просвіт до краю перешкоди: {res['min_clearance_m']:.2f} м (вимога d_min = {v.D_MIN} м)")
    print("Успіх (усі прибули, відстані >= d_min):", res["success"])
    metrics = {
        "variant": 17, "t_arrive_s": res["t_arrive"], "makespan_s": res["makespan"],
        "astar_len_m": [round(x, 2) for x in res["astar_len_m"]],
        "min_pair_m": round(res["min_pair_m"], 3), "min_clearance_m": round(res["min_clearance_m"], 3),
        "d_min": v.D_MIN, "planner_margin": v.PLANNER_MARGIN, "success": res["success"],
    }
    (OUT / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    return res


if __name__ == "__main__":
    main()
