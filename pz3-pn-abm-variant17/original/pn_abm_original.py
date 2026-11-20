# ОРИГІНАЛЬНИЙ КОД ПРОТОТИПУ (Prakt_6_1_2General+.pdf, "Практичне заняття 3").
# Перенесено з PDF БЕЗ змін логіки. Єдині правки: (1) розірвані переносом рядки
# коментарів зібрано в один рядок, (2) довгий пояснювальний коментар у update_motion
# скорочено (це лише коментар). Див. ../CHANGES.md, розділ "A".
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# =====================================================================
# 1. КОМПОНЕНТ 1: PN-SUPERVISOR (МЕРЕЖА ПЕТРІ)
# =====================================================================
class PetriNetSupervisor:
    def __init__(self):
        #Позначення:
        #AMR1, AMR2 = мобільні роботи
        #M1,M2 – верстати
        #CZ – критична зона
        # Позиції:
        # 0: p_CZ_free (1 - прохід вільний)
        # 1: p_M1_free (1 - верстат M1 вільний)
        # 2: p_M2_free (1 - верстат M2 вільний)
        # 3: p_AMR1_in_CZ (1 - AMR1 у критичній зоні)
        # 4: p_AMR2_in_CZ (1 - AMR2 у критичній зоні)
        # 5: p_M1_working (1 - M1 обробляє деталь від AMR1)
        # 6: p_M2_working (1 - M2 обробляє деталь від AMR2)
        self.M = np.array([1, 1, 1, 0, 0, 0, 0], dtype=int)
# Початкові умови для допустимості переходів
    def can_trigger(self, transition_id):
        if transition_id == 't_enter_CZ_AMR1': # для переходу робота 1 в критичну зону
            return self.M[0] == 1 and self.M[1] == 1 # потрібні вільні CZ та M1
        elif transition_id == 't_exit_CZ_AMR1': # для виходу робота 1 із критичної зони
            return self.M[3] == 1# потрібно щоб робот 1 був у критичній зоні
        elif transition_id == 't_enter_CZ_AMR2': # для переходу робота 2 в критичну зону
            return self.M[0] == 1 and self.M[2] == 1 # Потрібні вільна CZ та M2
        elif transition_id == 't_exit_CZ_AMR2': # для виходу робота 2 із критичної зони
            return self.M[4] == 1# потрібно щоб робот 2 був у критичній зоні
        return False

    def trigger(self, transition_id):
        if not self.can_trigger(transition_id):#перехід не допустимий – ніяких дій
            return False
        if transition_id == 't_enter_CZ_AMR1': #захід робота 1 в критичну зону
            self.M[0] -= 1 # Займаємо CZ
            self.M[3] += 1 # AMR1 в CZ
        elif transition_id == 't_exit_CZ_AMR1': #вихід робота 1 з критичної зони
            self.M[3] -= 1 # Виходимо з CZ
            self.M[0] += 1 # Звільняємо CZ
            self.M[1] -= 1 # Займаємо M1
            self.M[5] += 1 # M1 працює
        elif transition_id == 't_enter_CZ_AMR2': #захід робота 2 в критичну зону
            self.M[0] -= 1 # Займаємо CZ
            self.M[4] += 1 # AMR2 в CZ
        elif transition_id == 't_exit_CZ_AMR2': #вихід робота 2 з критичної зони
            self.M[4] -= 1 # Виходимо з CZ
            self.M[0] += 1 # Звільняємо CZ
            self.M[2] -= 1 # Займаємо M2
            self.M[6] += 1 # M2 працює
        return True

# =====================================================================
# 2. КОМПОНЕНТ 2: ABM-EXECUTOR (АГЕНТНИЙ РОБОТ)
# =====================================================================
class AMRAgent:
    def __init__(self, agent_id, start_pos, target_m1_m2, color):
        self.id = agent_id
        self.pos = np.array(start_pos, dtype=float)
        self.target = np.array(target_m1_m2, dtype=float)
        self.color = color
        self.v_max = 1.2
        self.radius = 0.6

        # Стани агента: 'APPROACHING_CZ', 'WAITING_PERMIT', 'IN_CZ', 'AT_MACHINE'
        #наближення до критичної зони
        # очікування лозволу
        # в критичній зоні
        # за верстатом
        self.state = 'APPROACHING_CZ'
        self.path_history = [self.pos.copy()]

    def update_motion(self, cz_entry_x, cz_exit_x, supervisor, dt=0.1):
        if self.state == 'AT_MACHINE':
            return
        # Визначення поточного цільового вектору руху
        if self.state in ['APPROACHING_CZ', 'WAITING_PERMIT']:
            # Прямуємо до входу у вузький прохід
            target_pt = np.array([cz_entry_x, self.pos[1]])

            # Перевірка наближення до геозони входу в CZ
            if abs(self.pos[0] - cz_entry_x) < 0.4:
                self.state = 'WAITING_PERMIT'
                t_req = f't_enter_CZ_AMR{self.id}'
                if supervisor.can_trigger(t_req):
                    supervisor.trigger(t_req)
                    self.state = 'IN_CZ'
        elif self.state == 'IN_CZ':
            # Рух через вузький прохід до виходу
            target_pt = np.array([cz_exit_x, self.pos[1]])

            # Перевірка виходу з CZ
            if abs(self.pos[0] - cz_exit_x) < 0.4:
                t_exit = f't_exit_CZ_AMR{self.id}'
                # t_ (Prefix): У теорії та моделюванні мереж Петрі * прийнято
                # чітко розділяти два типи вузлів: Places (позиції, умови, буфери —
                #зазвичай з префіксом p_)
                #та Transitions (переходи, події, дії — зазвичай з префіксом t_).
                #Цей рядок позначає перехід, що відповідає події виходу агента з певної
                #зони.
                #exit (Action): Вказує на напрямок або характер події — завершення
                #перебування у певній ділянці (на відміну від t_enter_...).
                # CZ (Critical Zone / Conflict Zone / Cell Zone): Абревіатура, що означає
                # критичну зону, конфліктну ділянку (вузький коридор, перехрестя,
                # зону завантаження біля верстата) або виробничу комірку (Cell).
                #Це спільний ресурс виробничої системи,
                # доступ до якого суворо регламентований, щоб уникнути колізій.
                # AMR (Entity Type): Вказує на клас сутності, яка виконує операцію —
                # Autonomous Mobile Robot (автономний мобільний робот).
                # {self.id} (Dynamic Instance ID): Інтерполяція у f-рядку Python.
                # f'...' (F-string / Formatted String): Механізм
                # динамічного створення рядків у Python.
                supervisor.trigger(t_exit)
                self.state = 'GOING_TO_MACHINE'
        elif self.state == 'GOING_TO_MACHINE':
            target_pt = self.target
            if np.linalg.norm(self.pos - self.target) < 0.5:
                self.state = 'AT_MACHINE'

        # Якщо чекаємо дозвіл від Мережі Петрі - стоїмо на місці
        if self.state == 'WAITING_PERMIT':
            vel = np.zeros(2)
        else:
            dir_vec = target_pt - self.pos
            norm = np.linalg.norm(dir_vec)
            if norm > 0.05:
                vel = (dir_vec / norm) * self.v_max
            else:
                vel = np.zeros(2)

        self.pos += vel * dt
        self.path_history.append(self.pos.copy())

# =====================================================================
# 3. СИМУЛЯЦІЙНЕ СЕРЕДОВИЩЕ ТА ВІЗУАЛІЗАЦІЯ
# =====================================================================
def run_production_simulation():
    supervisor = PetriNetSupervisor()

    # Геометрія виробничої дільниці (метри)
    # Вузький прохід (Критична зона) від X = 8.0 до X = 16.0
    cz_entry_x = 8.0
    cz_exit_x = 16.0

    # Верстати M1 та M2 розташовані за критичною зоною
    m1_pos = [22.0, 6.0]
    m2_pos = [22.0, 2.0]

    # Створення 2-х роботів (AMR_1 та AMR_2)
    # AMR_1 стартує трохи ближче, AMR_2 - трохи далі
    amr1 = AMRAgent(agent_id=1, start_pos=[1.0, 6.0], target_m1_m2=m1_pos, color='blue')
    amr2 = AMRAgent(agent_id=2, start_pos=[0.0, 2.0], target_m1_m2=m2_pos, color='magenta')

    dt = 0.1
    steps = 220

    # Налаштування візуалізації Matplotlib
    fig, (ax_map, ax_pn) = plt.subplots(2, 1, figsize=(11, 8), gridspec_kw={'height_ratios': [2, 1]})
    plt.ion()

    for step in range(steps):
        # Оновлення агентури
        amr1.update_motion(cz_entry_x, cz_exit_x, supervisor, dt)
        amr2.update_motion(cz_entry_x, cz_exit_x, supervisor, dt)

        if step % 2 == 0:  # Оновлюємо кадр кожні 0.2 сек
            ax_map.clear()
            ax_pn.clear()

            # --- ГРАФІК 1: КАРТА ВИРОБНИЧОЇ ДІЛЬНИЦІ ---
            ax_map.set_xlim(-2, 26)
            ax_map.set_ylim(-1, 9)
            ax_map.set_aspect('equal')
            ax_map.grid(True, linestyle='--', alpha=0.5)
            ax_map.set_title(f"Виробнича дільниця [Крок {step} | t = {step*dt:.1f}s]", fontsize=11, fontweight='bold')

            # Малювання Стін та Вузького Проходу (Критична Зона)
            # Верхня та нижня стіни проходу
            ax_map.add_patch(patches.Rectangle((cz_entry_x, 4.2), cz_exit_x - cz_entry_x, 3.8, color='gray', alpha=0.3))
            ax_map.add_patch(patches.Rectangle((cz_entry_x, 0.0), cz_exit_x - cz_entry_x, 0.8, color='gray', alpha=0.3))

            # Позначення критичної зони
            cz_box = patches.Rectangle((cz_entry_x, 0.8), cz_exit_x - cz_entry_x, 3.4,
                                       linewidth=2, edgecolor='red', facecolor='none', linestyle='--', label='Критична Зона (CZ)')
            ax_map.add_patch(cz_box)

            # Верстати M1 та M2
            ax_map.add_patch(patches.Rectangle((m1_pos[0]-1, m1_pos[1]-1), 2, 2, color='darkgreen', alpha=0.7))
            ax_map.text(m1_pos[0], m1_pos[1], "Верстат M1", color='white', ha='center', va='center', fontweight='bold')

            ax_map.add_patch(patches.Rectangle((m2_pos[0]-1, m2_pos[1]-1), 2, 2, color='darkgreen', alpha=0.7))
            ax_map.text(m2_pos[0], m2_pos[1], "Верстат M2", color='white', ha='center', va='center', fontweight='bold')

            # Малювання траєкторій та роботів
            for amr in [amr1, amr2]:
                pts = np.array(amr.path_history)
                ax_map.plot(pts[:, 0], pts[:, 1], color=amr.color, linestyle='-', alpha=0.6, linewidth=2)

                # Малювання корпуса робота
                circle = plt.Circle(amr.pos, amr.radius, color=amr.color, alpha=0.85)
                ax_map.add_patch(circle)
                ax_map.text(amr.pos[0], amr.pos[1], f"AMR_{amr.id}\n[{amr.state[:4]}]",
                            color='white', fontsize=7, ha='center', va='center', fontweight='bold')

            ax_map.legend(loc='upper left', fontsize=8)

            # --- ГРАФІК 2: МОНІТОР СТАНУ МЕРЕЖІ ПЕТРІ (PN-SUPERVISOR) ---
            ax_pn.set_xlim(0, 10)
            ax_pn.set_ylim(0, 3)
            ax_pn.axis('off')
            ax_pn.set_title("Поточний стан ресурсів PN-Supervisor (Маркіровка M)", fontsize=10, fontweight='bold')

            # Індикатори фішок Мережі Петрі
            tokens_info = [
                (1.5, 1.5, f"p_CZ_free\n[{supervisor.M[0]}]", 'green' if supervisor.M[0] else 'red'),
                (4.0, 2.0, f"p_AMR1_in_CZ\n[{supervisor.M[3]}]", 'blue' if supervisor.M[3] else 'gray'),
                (4.0, 1.0, f"p_AMR2_in_CZ\n[{supervisor.M[4]}]", 'magenta' if supervisor.M[4] else 'gray'),
                (7.5, 2.0, f"p_M1_work\n[{supervisor.M[5]}]", 'darkgreen' if supervisor.M[5] else 'gray'),
                (7.5, 1.0, f"p_M2_work\n[{supervisor.M[6]}]", 'darkgreen' if supervisor.M[6] else 'gray'),
            ]

            for x, y, label, col in tokens_info:
                c = plt.Circle((x, y), 0.45, color=col, alpha=0.8)
                ax_pn.add_patch(c)
                ax_pn.text(x, y, label, color='white', ha='center', va='center', fontsize=8, fontweight='bold')

            plt.pause(0.01)
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_production_simulation()
