# Автор: Осадчук Сергій, аспірант 1 р. н., ЗУНУ (спец. G7), 2026
# Практичне заняття 4: A* + APF для 3 дронів. Варіант 17.
#
# Основа - прототип викладача (Prakt_4_1General+++.pdf, див. original/astar_apf_original.py).
# Класи AStarPlanner і Drone збережено; усі відмінності від прототипу позначено
# коментарями [ВАРІАНТ] (дані варіанта) або [ЗМІНА] (правка коду) і розписано в CHANGES.md.
import heapq
import numpy as np

# === 0. ДАНІ ВАРІАНТА 17 (Prakt_4_2_var21+++.pdf, рядок 17) ===  [ВАРІАНТ]
GRID_SIZE = (50, 50)
OBSTACLES = [(22, 20, 6.0), (27, 30, 5.0), (14, 32, 4.5)]   # (x, y, R)
STARTS = [[5.0, 8.0], [8.0, 45.0], [45.0, 8.0]]
GOALS = [[45.0, 42.0], [42.0, 5.0], [8.0, 45.0]]
R_SAFETY = 1.1          # необхідний запас безпеки від краю перешкоди, м

# === Параметри моделі (значення прототипу збережено) ===
DT = 0.2                # крок за часом, с (як у прототипу)
CAPTURE_RADIUS = 1.0    # радіус захоплення вейпоінта, м (у прототипу вшито в Drone.update)
R_SAFE_DRONES = 2.5     # зона відштовхування між дронами, м (як у прототипу)
K_REP = 3.0             # вага відштовхування (у прототипу вшита числом 3.0)
# [ЗМІНА C2] A* раніше брав margin = r_safety; але через радіус захоплення вейпоінта (1.0 м)
# дрон зрізає кути і просідає до 1 м ближче до перешкоди, ніж проходить сітковий шлях.
PLANNER_MARGIN = R_SAFETY + CAPTURE_RADIUS
MAX_STEPS = 400         # [ЗМІНА C4] було steps = 250 (50 с); ліміт часу, виходимо раніше, коли всі прибули


# === 1. РЕАЛІЗАЦІЯ АЛГОРИТМУ A* ДЛЯ ГРІДУ ===
class AStarPlanner:
    def __init__(self, grid_size, obstacles, safety_margin=1.2):
        self.grid_size = grid_size
        self.obstacles = obstacles  # Список [(x, y, radius), ...]
        self.margin = safety_margin

    def is_valid(self, x, y):
        if not (0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]):
            return False
        for ox, oy, r in self.obstacles:
            if np.hypot(x - ox, y - oy) <= (r + self.margin):
                return False
        return True

    def plan(self, start, goal):
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: np.hypot(start[0] - goal[0], start[1] - goal[1])}
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if np.hypot(current[0] - goal[0], current[1] - goal[1]) < 1.5:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                # [ЗМІНА C3] було: return np.array(path[::-1], dtype=float)
                # Останній вузол A* лежить до 1.5 клітини від цілі; додаємо точну ціль
                # останнім вейпоінтом, інакше дрон ніколи не досягає цілі (див. CHANGES.md, B1).
                return np.array(path[::-1] + [goal], dtype=float)

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue
                tentative_g = g_score[current] + np.hypot(dx, dy)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + np.hypot(neighbor[0] - goal[0], neighbor[1] - goal[1])
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        return np.array([start, goal], dtype=float)


# === 2. КЛАС ДРОНА З ЛОКАЛЬНИМ УНИКНЕННЯМ ===
class Drone:
    # [ЗМІНА C5] r_safe і k_rep стали параметрами конструктора (за замовчуванням - значення прототипу)
    def __init__(self, drone_id, start, goal, path, r_safe=R_SAFE_DRONES, k_rep=K_REP):
        self.id = drone_id
        self.pos = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.path = path
        self.target_idx = 0
        self.v_max = 1.2
        self.r_safe = r_safe
        self.k_rep = k_rep
        self.history = [self.pos.copy()]

    def update(self, all_drones, dt=0.1):
        if np.linalg.norm(self.pos - self.goal) < 0.5:
            self.history.append(self.pos.copy())
            return

        # Напрямок уздовж планованого A*-шляху
        if self.target_idx < len(self.path):
            target = self.path[self.target_idx]
            if np.linalg.norm(self.pos - target) < CAPTURE_RADIUS and self.target_idx < len(self.path) - 1:
                self.target_idx += 1
                target = self.path[self.target_idx]
            v_pref = (target - self.pos)
            norm = np.linalg.norm(v_pref)
            if norm > 0:
                v_pref = (v_pref / norm) * self.v_max
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
                f_repel += (1.0 / dist - 1.0 / self.r_safe) * (diff / (dist**2)) * self.k_rep

        v_total = v_pref + f_repel
        speed = np.linalg.norm(v_total)
        if speed > self.v_max:
            v_total = (v_total / speed) * self.v_max
        self.pos += v_total * dt
        self.history.append(self.pos.copy())


# === 3. СЦЕНАРІЙ, СИМУЛЯЦІЯ, ПОКАЗНИКИ ===
def run_simulation(obstacles=OBSTACLES, starts=STARTS, goals=GOALS, r_safety=R_SAFETY,
                   planner_margin=None, dt=DT, max_steps=MAX_STEPS,
                   r_safe=R_SAFE_DRONES, k_rep=K_REP, grid_size=GRID_SIZE):
    """Повертає словник: дрони, час прибуття кожного, мінімуми відстаней, довжини A*-шляхів."""
    if planner_margin is None:
        planner_margin = r_safety + CAPTURE_RADIUS
    planner = AStarPlanner(grid_size, obstacles, planner_margin)
    drones = []
    for i in range(len(starts)):
        path = planner.plan(starts[i], goals[i])
        drones.append(Drone(i + 1, starts[i], goals[i], path, r_safe=r_safe, k_rep=k_rep))

    t_arrive = [None] * len(drones)
    for step in range(max_steps):
        for d in drones:
            d.update(drones, dt=dt)
        for i, d in enumerate(drones):
            if t_arrive[i] is None and np.linalg.norm(d.pos - d.goal) < 0.5:
                t_arrive[i] = round((step + 1) * dt, 1)
        if all(t is not None for t in t_arrive):   # [ЗМІНА C4] зупинка, коли всі прибули
            break

    H = [np.array(d.history) for d in drones]
    n = min(len(h) for h in H)
    pair = min(float(np.linalg.norm(H[i][:n] - H[j][:n], axis=1).min())
               for i in range(len(H)) for j in range(i + 1, len(H)))
    clearance = min(float(np.hypot(h[:, 0] - ox, h[:, 1] - oy).min() - r)
                    for h in H for ox, oy, r in obstacles)
    astar_len = [float(np.sum(np.linalg.norm(np.diff(d.path, axis=0), axis=1))) for d in drones]
    return {
        "drones": drones, "t_arrive": t_arrive,
        "success": all(t is not None for t in t_arrive) and clearance >= r_safety,
        "makespan": max((t for t in t_arrive if t is not None), default=None),
        "min_pair_m": pair, "min_clearance_m": clearance, "astar_len_m": astar_len,
        "r_safety": r_safety, "obstacles": obstacles,
    }
