"""КРОК 1 порівняння: запуск прототипу викладача БЕЗ змін.

Навіщо: перевірити, що прототип працює, і зафіксувати його поведінку на ВЛАСНИХ даних (з PDF).
Що робить: виконує original/astar_apf_original.py; «безголово» (headless) = без вікна на екрані:
  замість plt.show() малюнок зберігається у файл. Сам оригінал не змінюється.
Що виводить: залишок до цілі для кожного дрона, мінімальну відстань між дронами, мінімальний просвіт до перешкод.
Результат: results/original_final_frame.png (збігається з малюнком у PDF).
Запуск: python run_original_headless.py"""
import os, runpy
os.environ["MPLBACKEND"] = "Agg"
import numpy as np
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results", "original_final_frame.png")
os.makedirs(os.path.dirname(OUT), exist_ok=True)
plt.show = lambda *a, **k: plt.gcf().savefig(OUT, dpi=150)

ns = runpy.run_path(os.path.join(HERE, "original", "astar_apf_original.py"))
drones, obstacles, dt = ns["drones"], ns["obstacles"], ns["dt"]

print("Дрон | довжина A*-шляху | кінцева відстань до цілі | досяг цілі (<0.5 м)")
for d in drones:
    plen = float(np.sum(np.linalg.norm(np.diff(d.path, axis=0), axis=1)))
    dist = float(np.linalg.norm(d.pos - d.goal))
    print(f"  {d.id}  | {plen:6.1f} м | {dist:5.2f} м | {'так' if dist < 0.5 else 'НІ'}"
          f"  (вейпоінт {d.target_idx + 1}/{len(d.path)})")

H = [np.array(d.history) for d in drones]
n = min(len(h) for h in H)
pair = min(float(np.linalg.norm(H[i][:n] - H[j][:n], axis=1).min())
           for i in range(3) for j in range(i + 1, 3))
clear = min(float(np.hypot(h[:, 0] - ox, h[:, 1] - oy).min() - r)
            for h in H for ox, oy, r in obstacles)
print(f"Мінімальна відстань між дронами: {pair:.2f} м (r_safe = 2.5)")
print(f"Мінімальний просвіт до краю перешкоди: {clear:.2f} м")
print("Кадр збережено:", OUT)
