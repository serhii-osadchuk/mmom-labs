# Прототип викладача: Prakt_4_1General+++.pdf (Практичне заняття 4, A* + APF для 3 дронів).
# Перенесено з PDF БЕЗ змін логіки. Відновлено лише відступи (4 пробіли) за малюнком сторінок
# і скорочено довгі пояснювальні коментарі (позначено "[...]"). Змінні/числа не чіпались.
import heapq
import numpy as np
import matplotlib.pyplot as plt

# === 1. РЕАЛІЗАЦІЯ АЛГОРИТМУ A* ДЛЯ ГРІДУ ===
class AStarPlanner:  # Клас планувальника
    def __init__(self, grid_size, obstacles, safety_margin=1.2):
        self.grid_size = grid_size
        self.obstacles = obstacles  # Список [(x, y, radius), ...]
        self.margin = safety_margin

    def is_valid(self, x, y):
        if not (0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]):
            return False
        for ox, oy, r in self.obstacles:
            if np.hypot(x - ox, y - oy) <= (r + self.margin):
                # Евклідова відстань від точки (x, y) до центру перешкоди (ox, oy).
                # r - радіус перешкоди; self.margin - захисний буфер (радіус дрона, похибки, інерція).
                # [...]
                return False
        return True

    def plan(self, start, goal):  # метод формування плану
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Приведення неперервних координат до цілих індексів сітки.
        open_set = []
        heapq.heappush(open_set, (0, start))
        # open_set - черга з пріоритетом (мінімальна купа) за f(n) = g(n) + h(n).
        came_from = {}
        # Словник батьківських зв'язків { child_node: parent_node } для відновлення шляху.
        g_score = {start: 0}
        # Найменша відома вартість реального шляху від старту до вузла.
        f_score = {start: np.hypot(start[0] - goal[0], start[1] - goal[1])}
        # Прогнозована загальна вартість f(n) = g(n) + h(n); для старту f = h (евклідова евристика).
        # 8 можливих напрямків руху
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            # Вузол із найменшим f(n).
            if np.hypot(current[0] - goal[0], current[1] - goal[1]) < 1.5:
                # Ціль досягнута з допуском 1.5 клітини (а не точна рівність current == goal).
                path = [current]
                # Відновлення шляху (backtracking) за came_from.
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return np.array(path[::-1], dtype=float)
                # path[::-1] - розворот у порядку "від старту до цілі".

            # Дослідження сусідніх вузлів (node expansion & relaxation).
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue
                # Сусід поза картою або в зоні перешкоди (+ margin) - пропускаємо.
                tentative_g = g_score[current] + np.hypot(dx, dy)
                # Вартість шляху до сусіда через current (1.0 по осях, ~1.414 по діагоналі).
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # Сусід бачимо вперше, або знайдено коротший шлях до нього.
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + np.hypot(neighbor[0] - goal[0], neighbor[1] - goal[1])
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        return np.array([start, goal], dtype=float)


# === 2. КЛАС ДРОНА З ЛОКАЛЬНИМ УНИКНЕННЯМ ===
class Drone:
    def __init__(self, drone_id, start, goal, path):
        self.id = drone_id
        self.pos = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.path = path
        self.target_idx = 0
        self.v_max = 1.2
        self.r_safe = 2.5
        self.history = [self.pos.copy()]

    def update(self, all_drones, dt=0.1):
        if np.linalg.norm(self.pos - self.goal) < 0.5:
            # Допуск прибуття 0.5 м: дрон зупиняється і більше не рахує сил.
            self.history.append(self.pos.copy())
            return
            # Дрон залишається в цілі й слугує статичною перешкодою для інших.

        # Напрямок уздовж планованого A*-шляху
        if self.target_idx < len(self.path):
            target = self.path[self.target_idx]
            if np.linalg.norm(self.pos - target) < 1.0 and self.target_idx < len(self.path) - 1:
                self.target_idx += 1
                target = self.path[self.target_idx]
                # Радіус захоплення точки 1.0 м: перемикання на наступний вейпоінт.
            v_pref = (target - self.pos)
            norm = np.linalg.norm(v_pref)
            if norm > 0:
                v_pref = (v_pref / norm) * self.v_max
                # Одиничний напрямок до вейпоінта, помножений на v_max.
        else:
            v_pref = np.zeros(2)

        # Відштовхування від інших дронів
        f_repel = np.zeros(2)
        for other in all_drones:
            if other.id == self.id:
                continue
            diff = self.pos - other.pos
            dist = np.linalg.norm(diff)
            if 0 < dist < self.r_safe:
                f_repel += (1.0 / dist - 1.0 / self.r_safe) * (diff / (dist**2)) * 3.0
                # Гіперболічне відштовхування; 3.0 - вага (коефіцієнт жорсткості поля).

        v_total = v_pref + f_repel
        speed = np.linalg.norm(v_total)
        if speed > self.v_max:
            v_total = (v_total / speed) * self.v_max
            # Обмеження швидкості (clamping).
        self.pos += v_total * dt
        self.history.append(self.pos.copy())
        # Метод Ейлера: S = V * dt.


# === 3. НАЛАШТУВАННЯ СЦЕНАРІЮ ТА СИМУЛЯЦІЇ ===
grid_size = (50, 50)

# 3 Статичні перешкоди: [(x, y, радіус)]
obstacles = [
    (20, 20, 4.0),
    (30, 28, 5.0),
    (15, 35, 3.5)
]

# Старт та Цілі для 3 дронів
starts = [[5.0, 5.0], [5.0, 40.0], [40.0, 5.0]]
goals = [[45.0, 45.0], [45.0, 10.0], [10.0, 45.0]]

# Планування A*-маршрутів
planner = AStarPlanner(grid_size, obstacles)
drones = []
# Для кожного дрона глобальний маршрут A*, потім створюється Drone з індивідуальними параметрами.
for i in range(3):
    a_star_path = planner.plan(starts[i], goals[i])
    drones.append(Drone(drone_id=i+1, start=starts[i], goal=goals[i], path=a_star_path))

# === 4. СИМУЛЯЦІЯ ТА ВІЗУАЛІЗАЦІЯ ===
dt = 0.2
steps = 250

plt.figure(figsize=(9, 9))
plt.title("Управління 3 дронами через A* та оминанням 3 перешкод", fontsize=11, fontweight='bold')
plt.xlim(0, grid_size[0])
plt.ylim(0, grid_size[1])
plt.grid(True, linestyle='--', alpha=0.5)

# Малювання перешкод
for ox, oy, r in obstacles:
    circle = plt.Circle((ox, oy), r, color='darkred', alpha=0.6)
    plt.gca().add_patch(circle)

# Запуск виконання
for _ in range(steps):
    for d in drones:
        d.update(drones, dt=dt)

# Візуалізація результатів
colors = ['tab:blue', 'tab:green', 'tab:orange']

for i, d in enumerate(drones):
    path_arr = np.array(d.history)
    a_star_arr = np.array(d.path)

    # Запланований A*-маршрут (пунктир)
    plt.plot(a_star_arr[:, 0], a_star_arr[:, 1], '--', color=colors[i], alpha=0.4, label=f'A* План Дрон {d.id}')
    # Фактична траєкторія
    plt.plot(path_arr[:, 0], path_arr[:, 1], color=colors[i], linewidth=2.5, label=f'Траєкторія Дрон {d.id}')
    # Стартові та цільові точки
    plt.plot(d.history[0][0], d.history[0][1], 'o', color=colors[i], markersize=8)
    plt.plot(d.goal[0], d.goal[1], '*', color=colors[i], markersize=12)

plt.xlabel("X (метри)")
plt.ylabel("Y (метри)")
plt.legend(loc='upper right', fontsize=8)
plt.gca().set_aspect('equal')
plt.show()
