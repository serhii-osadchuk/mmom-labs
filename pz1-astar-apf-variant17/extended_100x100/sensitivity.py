# Автор: Осадчук Сергій, аспірант 1 року навчання (спеціальність G7), ЗУНУ, Тернопіль
# Дисципліна: «Математичне моделювання та обчислювальні методи»
# Практичне заняття 1, індивідуальне завдання №1, варіант 17. 2026 р.
"""
Перевірка стійкості рішення до зміни параметрів (варіант 17).

Запуск:  python sensitivity.py
Результат: results/sensitivity.json та таблиця в консолі.
"""
import itertools
import json
from pathlib import Path

import numpy as np

from run_variant17 import run

OUT = Path(__file__).parent / "results"


def one(**kw):
    sim, res = run(use_right_hand=True, use_yield=False, **kw)
    return {
        **kw,
        "ok": bool(res.success),
        "T": round(float(res.makespan), 1),
        "min_pair": round(res.min_pair, 2),
        "min_clear": round(res.min_clearance, 2),
    }


def main():
    OUT.mkdir(exist_ok=True)
    data = {
        "margin": [one(margin=m) for m in (2.5, 3.0, 3.5, 4.0, 5.0)],
        "dt": [one(dt=dt) for dt in (0.05, 0.1, 0.2)],
        "force": [one(k_right=kr, r_safe=rs)
                  for kr, rs in itertools.product((2, 4, 6, 10, 15), (5, 8, 12))],
    }
    (OUT / "sensitivity.json").write_text(json.dumps(data, ensure_ascii=False, indent=2),
                                          encoding="utf-8")
    for name, rows in data.items():
        ok = sum(r["ok"] for r in rows)
        Ts = [r["T"] for r in rows]
        print(f"{name:7s}: успішних {ok}/{len(rows)}, T = {min(Ts)}..{max(Ts)} с, "
              f"min між дронами = {min(r['min_pair'] for r in rows)} м, "
              f"min до перешкоди = {min(r['min_clear'] for r in rows)} м")


if __name__ == "__main__":
    main()
