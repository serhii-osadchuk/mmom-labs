"""
Практичне заняття 4 (індивідуальне завдання №1).
Керування групою безпілотних літальних апаратів (дронів) із глобальним плануванням A* та локальним
уникненням зіткнень методом штучних потенційних полів (APF).

Модуль містить:
    * AStarPlanner  - глобальний планувальник маршруту на сітці;
    * Drone         - агент: слідує за маршрутом і реагує на сусідів;
    * Simulation    - двофазна синхронна симуляція та збір метрик.

Умовні позначення (збігаються з прикладом Prakt_4_1General+++):
    p      - позиція, v - швидкість, goal - ціль, path - маршрут A*;
    r_safe - радіус, у межах якого дрони починають відштовхуватися;
    d_min  - мінімально допустима відстань за умовою завдання.
"""
from __future__ import annotations

import heapq
from dataclasses import dataclass, field

import numpy as np


# ---------------------------------------------------------------------------
# 1. ПАРАМЕТРИ
# ---------------------------------------------------------------------------
@dataclass
class Params:
    """Усі налаштування алгоритму в одному місці."""

    # --- кінематика ---
    v_max: float = 2.0          # максимальна швидкість, м/с
    dt: float = 0.1             # крок симуляції, с
    arrive_eps: float = 0.3     # дрон вважається таким, що досяг цілі
    waypoint_eps: float = 1.5   # радіус "захоплення" проміжної точки маршруту

    # --- глобальний планувальник A* ---
    grid_step: float = 1.0      # крок сітки, м
    margin: float = 4.0         # запас навколо перешкоди для A* (має бути > d_min)

    # --- APF: відштовхування від інших дронів ---
    r_safe: float = 12.0        # радіус впливу сусіда
    k_rep: float = 40.0         # коефіцієнт відштовхування
    use_right_hand: bool = True  # правило "правої руки" (тангенціальний доданок)
    k_right: float = 10.0       # коефіцієнт "правої руки"

    # --- APF: відштовхування від перешкоди ---
    obs_influence: float = 8.0  # на якій відстані від краю перешкоди діє поле
    k_obs: float = 40.0

    # --- тимчасова зупинка за пріоритетом ---
    use_yield: bool = False     # у варіанті 17 не потрібна: збільшує час на ~3 с
    d_yield: float = 3.5        # якщо дрон вищого пріоритету ближче - чекаємо

    # --- вимога завдання ---
    d_min: float = 2.0


# ---------------------------------------------------------------------------
# 2. ГЛОБАЛЬНИЙ ПЛАНУВАЛЬНИК A*
# ---------------------------------------------------------------------------
class AStarPlanner:
    """A* на рівномірній сітці з 8 напрямками руху.

    Круглі перешкоди роздуваються на `margin`: вузол сітки вважається
    забороненим, якщо він ближче ніж (R + margin) до центра перешкоди.
    """

    NEIGHBORS = [(0, 1), (1, 0), (0, -1), (-1, 0),
                 (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def __init__(self, field_size, obstacles, margin, step=1.0):
        self.w, self.h = field_size
        self.obstacles = obstacles          # [(x, y, R), ...]
        self.margin = margin
        self.step = step

    # --- допоміжні перевірки ---
    def point_free(self, x, y):
        if not (0 <= x <= self.w and 0 <= y <= self.h):
            return False
        return all(np.hypot(x - ox, y - oy) > r + self.margin
                   for ox, oy, r in self.obstacles)

    def segment_free(self, a, b):
        """Чи вільний відрізок a-b (перевірка через щільну вибірку точок)."""
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        n = max(int(np.linalg.norm(b - a) / 0.25), 1)
        return all(self.point_free(*(a + (b - a) * (i / n))) for i in range(n + 1))

    # --- власне A* ---
    def plan(self, start, goal):
        s = (round(start[0] / self.step), round(start[1] / self.step))
        g = (round(goal[0] / self.step), round(goal[1] / self.step))
        to_xy = lambda n: (n[0] * self.step, n[1] * self.step)
        if not self.point_free(*to_xy(s)) or not self.point_free(*to_xy(g)):
            raise ValueError("Старт або ціль лежать у забороненій зоні")

        h = lambda n: np.hypot(n[0] - g[0], n[1] - g[1])   # евристика (евклідова)
        open_set = [(h(s), s)]
        came_from, g_score = {}, {s: 0.0}

        while open_set:
            f, cur = heapq.heappop(open_set)
            if cur == g:
                break
            if f > g_score[cur] + h(cur) + 1e-9:            # застарілий запис у купі
                continue
            for dx, dy in self.NEIGHBORS:
                nb = (cur[0] + dx, cur[1] + dy)
                if not self.point_free(*to_xy(nb)):
                    continue
                tentative = g_score[cur] + np.hypot(dx, dy)
                if tentative < g_score.get(nb, np.inf):
                    came_from[nb] = cur
                    g_score[nb] = tentative
                    heapq.heappush(open_set, (tentative + h(nb), nb))
        else:
            raise RuntimeError("Маршрут не знайдено")

        # відновлення шляху від цілі до старту
        path, node = [g], g
        while node in came_from:
            node = came_from[node]
            path.append(node)
        path = [to_xy(n) for n in reversed(path)]
        path[0], path[-1] = tuple(start), tuple(goal)       # точні кінці
        return np.array(self._smooth(path), dtype=float)

    def _smooth(self, path):
        """Спрощення "драбинки" A*: лишаємо лише вузли, між якими є пряма видимість."""
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.segment_free(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out


# ---------------------------------------------------------------------------
# 3. АГЕНТ (ДРОН)
# ---------------------------------------------------------------------------
class Drone:
    def __init__(self, drone_id, start, goal, path, priority):
        self.id = drone_id
        self.pos = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.path = path
        self.priority = priority            # 1 - найвищий
        self.target_idx = 1                 # path[0] - це старт
        self.reached = False
        self.finish_time = None
        self.waiting = False
        self.history = [self.pos.copy()]
        self.wait_steps = 0

    # --- крок 1: розрахунок швидкості (позиції ще НЕ змінюємо) ---
    def compute_velocity(self, others, obstacles, prm: Params):
        if self.reached:
            return np.zeros(2)

        # 1. тяга до поточної проміжної точки маршруту A*
        while (self.target_idx < len(self.path) - 1 and
               np.linalg.norm(self.path[self.target_idx] - self.pos) < prm.waypoint_eps):
            self.target_idx += 1
        to_target = self.path[self.target_idx] - self.pos
        dist = np.linalg.norm(to_target)
        v_goal = to_target / (dist + 1e-9) * prm.v_max

        # 2. тимчасова зупинка: чекаємо, якщо поруч дрон вищого пріоритету
        self.waiting = False
        if prm.use_yield:
            for o in others:
                if (o.id != self.id and not o.reached and o.priority < self.priority
                        and np.linalg.norm(o.pos - self.pos) < prm.d_yield):
                    self.waiting = True
                    return np.zeros(2)

        # 3. відштовхування від сусідів + правило правої руки
        v_rep = np.zeros(2)
        v_right = np.zeros(2)
        for o in others:
            if o.id == self.id:
                continue
            diff = self.pos - o.pos
            d = np.linalg.norm(diff)
            if 1e-9 < d < prm.r_safe:
                push = prm.k_rep * (1.0 / d - 1.0 / prm.r_safe) / d ** 2
                v_rep += push * diff / d
                if prm.use_right_hand and not o.reached:
                    e_to_other = -diff / d                          # напрям на сусіда
                    e_right = np.array([e_to_other[1], -e_to_other[0]])  # поворот на 90° за год. стрілкою
                    v_right += prm.k_right * (1.0 / d - 1.0 / prm.r_safe) * e_right

        # 4. відштовхування від перешкод
        v_obs = np.zeros(2)
        for ox, oy, r in obstacles:
            diff = self.pos - np.array([ox, oy])
            d = np.linalg.norm(diff)
            gap = d - r                                             # щілина до краю перешкоди
            if gap < prm.obs_influence:
                gap = max(gap, 0.1)
                v_obs += (prm.k_obs * (1.0 / gap - 1.0 / prm.obs_influence)
                          / gap ** 2 * diff / (d + 1e-9))

        # 5. сума та обмеження швидкості
        v = v_goal + v_rep + v_right + v_obs
        speed = np.linalg.norm(v)
        if speed > prm.v_max:
            v = v / speed * prm.v_max
        return v

    # --- крок 2: застосування швидкості ---
    def apply(self, v, t, prm: Params):
        if self.reached:
            self.history.append(self.pos.copy())
            return
        if self.waiting:
            self.wait_steps += 1
        self.pos = self.pos + v * prm.dt
        self.history.append(self.pos.copy())
        if np.linalg.norm(self.goal - self.pos) < prm.arrive_eps:
            self.reached = True
            self.finish_time = t


# ---------------------------------------------------------------------------
# 4. СИМУЛЯЦІЯ ТА МЕТРИКИ
# ---------------------------------------------------------------------------
@dataclass
class Result:
    t: np.ndarray                    # (T,)
    traj: np.ndarray                 # (T, N, 2)
    finish_times: list
    path_lengths: list
    min_pair: float                  # мінімальна відстань між дронами
    min_clearance: float             # мінімальна щілина до краю перешкоди
    pair_dist: dict = field(default_factory=dict)   # (i, j) -> масив відстаней
    obs_clear: np.ndarray = None     # (T, N) щілина до найближчої перешкоди
    wait_steps: list = field(default_factory=list)
    all_reached: bool = False
    d_min: float = 2.0

    @property
    def success(self) -> bool:
        return (self.all_reached and self.min_pair >= self.d_min
                and self.min_clearance >= self.d_min)

    @property
    def makespan(self):
        return max(self.finish_times) if self.all_reached else float("nan")


class Simulation:
    def __init__(self, field_size, obstacles, missions, prm: Params):
        """missions: [((x0, y0), (xg, yg)), ...] - у порядку пріоритету 1, 2, 3."""
        self.prm = prm
        self.obstacles = obstacles
        self.planner = AStarPlanner(field_size, obstacles, prm.margin, prm.grid_step)
        self.drones = []
        for i, (s, g) in enumerate(missions):
            path = self.planner.plan(s, g)
            self.drones.append(Drone(i + 1, s, g, path, priority=i + 1))

    def run(self, t_max=300.0) -> Result:
        prm = self.prm
        steps = int(t_max / prm.dt)
        t = 0.0
        for _ in range(steps):
            # фаза 1: усі дрони рахують швидкості за ПОТОЧНИМИ позиціями
            vels = [d.compute_velocity(self.drones, self.obstacles, prm) for d in self.drones]
            # фаза 2: синхронне оновлення позицій
            t += prm.dt
            for d, v in zip(self.drones, vels):
                d.apply(v, t, prm)
            if all(d.reached for d in self.drones):
                break
        return self._metrics(np.arange(len(self.drones[0].history)) * prm.dt)

    def _metrics(self, t) -> Result:
        traj = np.stack([np.array(d.history) for d in self.drones], axis=1)  # (T, N, 2)
        n = traj.shape[1]
        pair = {}
        for i in range(n):
            for j in range(i + 1, n):
                pair[(i + 1, j + 1)] = np.linalg.norm(traj[:, i] - traj[:, j], axis=1)
        obs_clear = np.full((traj.shape[0], n), np.inf)
        for ox, oy, r in self.obstacles:
            gap = np.linalg.norm(traj - np.array([ox, oy]), axis=2) - r
            obs_clear = np.minimum(obs_clear, gap)
        lengths = [float(np.sum(np.linalg.norm(np.diff(traj[:, k], axis=0), axis=1)))
                   for k in range(n)]
        return Result(
            t=t, traj=traj,
            finish_times=[d.finish_time for d in self.drones],
            path_lengths=lengths,
            min_pair=float(min(v.min() for v in pair.values())),
            min_clearance=float(obs_clear.min()),
            pair_dist=pair, obs_clear=obs_clear,
            wait_steps=[d.wait_steps for d in self.drones],
            all_reached=all(d.reached for d in self.drones),
            d_min=self.prm.d_min,
        )
