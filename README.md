# mmom-labs

Індивідуальні лабораторні/практичні роботи з дисципліни про моделювання автономних систем
(фільтр Калмана, керування групами дронів, мережі Петрі та ін.).

Кожна робота лежить у власній папці та **самодостатня**: має свій `README.md`,
`requirements.txt` і запускається незалежно від інших.

| Робота | Тема | Варіант |
|---|---|---|
| [lab1-variant17](lab1-variant17) | Практичне 4: група дронів, глобальний A\* + локальні потенційні поля (APF) | 17 |
| [pz3-pn-abm-variant17](pz3-pn-abm-variant17) | Практичне 3: мережа Петрі + агенти (PN-Supervisor / ABM-Executors) | 17 |

## Як запустити будь-яку роботу

```bash
git clone https://github.com/serhii-osadchuk/mmom-labs.git
cd mmom-labs/lab1-variant17

python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python run_variant17.py
```

Порядок роботи з завданнями описано в [WORKFLOW.md](WORKFLOW.md).

Клонування забирає лише невеликі файли коду та картинок, тож завантаження швидке.
Кожна наступна робота додається окремою папкою за тим самим шаблоном.
