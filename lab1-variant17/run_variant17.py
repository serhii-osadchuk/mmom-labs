"""
Лабораторна робота №1, варіант 17.

Запуск:
    python run_variant17.py            # розрахунок + графіки + таблиця метрик
    python run_variant17.py --gif      # додатково зберегти анімацію руху

Результати зберігаються в папці results/.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")                       # без вікон: лише збереження у файли
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle

from swarm import Params, Simulation

# ---------------------------------------------------------------------------
# ВАРІАНТ 17 (таблиця Prakt_4_2_var21, завдання 17 - "Обхід по великому радіусу")
# ---------------------------------------------------------------------------
FIELD = (100, 100)
OBSTACLES = [(50, 50, 20)]                              # (x, y, R)
MISSIONS = [((5, 5), (95, 95)),                         # Робот 1
            ((5, 95), (95, 5)),                         # Робот 2
            ((95, 5), (5, 95))]                         # Робот 3
D_MIN = 2.0
COLORS = ["#2b6cb0", "#dd6b20", "#2f855a"]
OUT = Path(__file__).parent / "results"


def run(**overrides):
    prm = Params(**overrides)
    sim = Simulation(FIELD, OBSTACLES, MISSIONS, prm)
    return sim, sim.run(t_max=300.0)


def draw_scene(ax, sim):
    """Поле, перешкода, зона запасу A* та маршрути A*."""
    prm = sim.prm
    for ox, oy, r in OBSTACLES:
        ax.add_patch(Circle((ox, oy), r, color="#e53e3e", alpha=0.6, zorder=2, label="Перешкода"))
        ax.add_patch(Circle((ox, oy), r + prm.margin, fill=False, ls=":", ec="#9b2c2c",
                            lw=1.2, zorder=2, label=f"Запас A* (+{prm.margin:g} м)"))
        ax.add_patch(Circle((ox, oy), r + D_MIN, fill=False, ls="--", ec="black",
                            lw=1.0, zorder=2, label=f"Межа d_min = {D_MIN:g} м"))
    for d, c in zip(sim.drones, COLORS):
        ax.plot(d.path[:, 0], d.path[:, 1], "--", color=c, alpha=0.35, lw=1.5, zorder=3)
    ax.set_xlim(0, FIELD[0]); ax.set_ylim(0, FIELD[1])
    ax.set_aspect("equal"); ax.grid(True, ls="--", alpha=0.4)
    ax.set_xlabel("X, м"); ax.set_ylabel("Y, м")


def plot_trajectories(sim, res, path, title):
    fig, ax = plt.subplots(figsize=(7.5, 7.5), dpi=200)
    draw_scene(ax, sim)
    snap_every = 10.0                                   # позначки поточного положення
    idx = [int(round(t / sim.prm.dt)) for t in np.arange(0, res.t[-1], snap_every)]
    for k, c in enumerate(COLORS):
        tr = res.traj[:, k]
        ax.plot(tr[:, 0], tr[:, 1], "-", color=c, lw=2.4, zorder=4, label=f"Робот {k + 1}")
        ax.plot(*tr[0], "o", color=c, ms=9, mec="white", zorder=6)           # старт
        ax.plot(*sim.drones[k].goal, "x", color=c, ms=11, mew=3, zorder=6)    # ціль
        for i in idx[1:]:
            ax.plot(*tr[i], "o", mfc="white", mec=c, ms=6, mew=1.6, zorder=5)
    ax.set_title(title, fontsize=11, fontweight="bold")
    h, l = ax.get_legend_handles_labels()
    uniq = dict(zip(l, h))
    ax.legend(uniq.values(), uniq.keys(), loc="upper center", fontsize=8,
              ncol=3, bbox_to_anchor=(0.5, -0.08))
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def plot_distances(res, path):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 7), dpi=200, sharex=True)
    for (i, j), d in res.pair_dist.items():
        a1.plot(res.t, d, lw=1.8, label=f"Роботи {i}-{j}")
    a1.axhline(D_MIN, color="red", ls="--", label=f"d_min = {D_MIN:g} м")
    a1.set_ylabel("Відстань між роботами, м"); a1.set_ylim(0, 60)
    a1.set_title("Відстані між роботами", fontsize=10, fontweight="bold")
    a2.axhline(D_MIN, color="red", ls="--", label=f"d_min = {D_MIN:g} м")
    for k, c in enumerate(COLORS):
        a2.plot(res.t, res.obs_clear[:, k], color=c, lw=1.8, label=f"Робот {k + 1}")
    a2.set_ylabel("Щілина до краю перешкоди, м"); a2.set_xlabel("Час, с")
    a2.set_ylim(0, 40)
    a2.set_title("Відстань від центра робота до краю перешкоди", fontsize=10, fontweight="bold")
    for a in (a1, a2):
        a.grid(True, ls="--", alpha=0.4); a.legend(fontsize=8, loc="upper right")
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def make_gif(sim, res, path, fps=20, stride=5):
    fig, ax = plt.subplots(figsize=(6, 6), dpi=100)
    draw_scene(ax, sim)
    lines, dots, rings, gx = [], [], [], []
    for k, c in enumerate(COLORS):
        (ln,) = ax.plot([], [], "-", color=c, lw=2.2, zorder=4)
        (dt_,) = ax.plot([], [], "o", color=c, ms=8, zorder=6)
        ring = Circle((0, 0), D_MIN / 2, fill=False, ec=c, lw=1.0, zorder=5)
        ax.add_patch(ring)
        ax.plot(*sim.drones[k].goal, "x", color=c, ms=10, mew=3, zorder=6)
        lines.append(ln); dots.append(dt_); rings.append(ring)
    title = ax.set_title("")
    frames = range(0, len(res.t), stride)

    def update(f):
        for k in range(len(COLORS)):
            tr = res.traj[: f + 1, k]
            lines[k].set_data(tr[:, 0], tr[:, 1])
            dots[k].set_data([tr[-1, 0]], [tr[-1, 1]])
            rings[k].center = tr[-1]
        title.set_text(f"t = {res.t[f]:.1f} с")
        return lines + dots + rings + [title]

    FuncAnimation(fig, update, frames=frames, blit=False).save(
        path, writer=PillowWriter(fps=fps))
    plt.close(fig)


def summarize(label, sim, res):
    return {
        "config": label,
        "all_reached": bool(res.all_reached),
        "finish_times_s": [None if f is None else round(float(f), 1) for f in res.finish_times],
        "makespan_s": None if not res.all_reached else round(float(res.makespan), 1),
        "min_robot_robot_m": round(res.min_pair, 2),
        "min_clearance_to_obstacle_m": round(res.min_clearance, 2),
        "wait_steps": res.wait_steps,
        "path_lengths_m": [round(x, 1) for x in res.path_lengths],
        "astar_lengths_m": [round(float(np.sum(np.linalg.norm(np.diff(d.path, axis=0), axis=1))), 1)
                            for d in sim.drones],
        "d_min_ok": bool(res.success),
    }


def main(make_animation):
    OUT.mkdir(exist_ok=True)

    # ---- основна конфігурація: A* + APF + правило правої руки ----
    sim, res = run(use_right_hand=True, use_yield=False)
    plot_trajectories(sim, res, OUT / "trajectories.png",
                      "Варіант 17: A* + APF + правило правої руки")
    plot_distances(res, OUT / "distances.png")
    if make_animation:
        make_gif(sim, res, OUT / "animation.gif")

    # ---- порівняння конфігурацій (абляція) ----
    configs = [
        ("A* + відштовхування (як у прикладі)", dict(use_right_hand=False, use_yield=False)),
        ("A* + відштовхування + права рука",    dict(use_right_hand=True,  use_yield=False)),
        ("A* + відштовхування + зупинки",       dict(use_right_hand=False, use_yield=True)),
        ("A* + права рука + зупинки",           dict(use_right_hand=True,  use_yield=True)),
    ]
    rows = []
    for label, kw in configs:
        s, r = run(**kw)
        rows.append(summarize(label, s, r))
        if label.startswith("A* + відштовхування (як"):
            plot_trajectories(s, r, OUT / "trajectories_baseline.png",
                              "Без правої руки (як у прикладі)")
    (OUT / "metrics.json").write_text(json.dumps(rows, ensure_ascii=False, indent=2),
                                      encoding="utf-8")

    # ---- таблиця у консоль ----
    print(f"{'Конфігурація':42s} {'T, с':>6s} {'min роб-роб':>12s} {'min до перешк.':>15s}  d_min")
    for r in rows:
        print(f"{r['config']:42s} {r['makespan_s']!s:>6s} {r['min_robot_robot_m']:>12.2f} "
              f"{r['min_clearance_to_obstacle_m']:>15.2f}  {'OK' if r['d_min_ok'] else 'ПОРУШЕНО'}")
    main_row = rows[1]
    print("\nОсновна конфігурація:")
    print("  час досягнення цілей, с:", main_row["finish_times_s"])
    print("  довжина A*-маршруту, м:", main_row["astar_lengths_m"])
    print("  довжина фактичного шляху, м:", main_row["path_lengths_m"])
    print(f"  нижня межа часу: {max(main_row['astar_lengths_m']) / Params().v_max:.1f} с")
    print("Файли збережено в:", OUT)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gif", action="store_true", help="зберегти анімацію results/animation.gif")
    main(ap.parse_args().gif)
