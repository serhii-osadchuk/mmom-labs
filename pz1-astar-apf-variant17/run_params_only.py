# Автор: Осадчук Сергій, аспірант 1 р. н., ЗУНУ (спец. G7), 2026
"""КРОК 2 порівняння: у ОРИГІНАЛ підставляються ЛИШЕ числа варіанта 17 (жодних інших правок).

Навіщо: показати, що простої підстановки чисел недостатньо (у прототипі є обмеження, див. CHANGES.md, група B).
Що робить: читає текст оригіналу, у пам'яті замінює дані сценарію на дані варіанта 17 і запускає.
  Файл original/ не змінюється; запуск «безголовий» (без вікна, малюнок зберігається у файл).
Що виводить: залишок до цілі, мінімальну відстань між дронами, просвіт до перешкоди.
Очікувано: дрони не долітають, відстань між дронами менша за d_min.
Результат: results/params_only.png
Запуск: python run_params_only.py"""
import os
os.environ["MPLBACKEND"] = "Agg"
import runpy, tempfile
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

HERE = Path(__file__).parent
src = (HERE / "original" / "astar_apf_original.py").read_text(encoding="utf-8")

# Єдині підстановки (дані варіанта 17 із Prakt_4_2_var21.pdf):
subs = [
    ("grid_size = (50, 50)", "grid_size = (100, 100)"),
    ("(20, 20, 4.0),\n    (30, 28, 5.0),\n    (15, 35, 3.5)", "(50, 50, 20.0)"),
    ("starts = [[5.0, 5.0], [5.0, 40.0], [40.0, 5.0]]", "starts = [[5.0, 5.0], [5.0, 95.0], [95.0, 5.0]]"),
    ("goals = [[45.0, 45.0], [45.0, 10.0], [10.0, 45.0]]", "goals = [[95.0, 95.0], [95.0, 5.0], [5.0, 95.0]]"),
    ("AStarPlanner(grid_size, obstacles)", "AStarPlanner(grid_size, obstacles, safety_margin=2.0)"),
]
for old, new in subs:
    assert src.count(old) == 1, old
    src = src.replace(old, new)

out = HERE / "results" / "params_only.png"
plt.show = lambda *a, **k: plt.gcf().savefig(out, dpi=130)
with tempfile.TemporaryDirectory() as tmp:
    f = Path(tmp) / "params_only.py"
    f.write_text(src, encoding="utf-8")
    ns = runpy.run_path(str(f))

drones, obstacles = ns["drones"], ns["obstacles"]
print("Дрон | кінцева відстань до цілі | досяг цілі (<0.5 м)")
for d in drones:
    dist = float(np.linalg.norm(d.pos - d.goal))
    print(f"  {d.id}  | {dist:5.2f} м | {'так' if dist < 0.5 else 'НІ'}")
H = [np.array(d.history) for d in drones]
n = min(len(h) for h in H)
pair = min(float(np.linalg.norm(H[i][:n] - H[j][:n], axis=1).min()) for i in range(3) for j in range(i + 1, 3))
clear = min(float(np.hypot(h[:, 0] - ox, h[:, 1] - oy).min() - r) for h in H for ox, oy, r in obstacles)
print(f"Мін. відстань між дронами: {pair:.2f} м; мін. просвіт до перешкоди: {clear:.2f} м (вимога d_min = 2.0 м)")
print("Кадр збережено:", out)
