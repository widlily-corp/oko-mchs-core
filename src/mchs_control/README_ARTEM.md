# БОЕВАЯ ИНСТРУКЦИЯ И ТЗ ДЛЯ АРТЁМА (AUTONOMY & DOCS LEAD)

Привет, Артём! Твоя задача на проекте — **автономия полета БПЛА**, генерация официального протокола инцидента МЧС и техническое описание дрона для жюри.

---

## 1. Где лежат твои файлы:
* Полетный автопилот: `src/mchs_control/scripts/mchs_pilot_node.py`
* Генератор протокола инцидента: `src/mchs_control/scripts/generate_report.py`
* Пояснительная записка: `docs/technical_note_template.md` (или `docs/technical_note.md`)

---

## 2. Твое ТЗ по пунктам (Что нужно сделать)

### Задача 1: Автономный автопилот FSM (`mchs_pilot_node.py`)
Реализовать конечный автомат (FSM) полета дрона:
1. **`TAKEOFF`**:
   - Плавный набор высоты до крейсерского эшелона $Z = 8.0\text{ м}$ (публикация `cmd_vel.linear.z = 1.0`).
2. **`PATROL_GRID`**:
   - Движение по маршрутной сетке («змейка» / галсы патрулирования) со скоростью $2.0\text{ м/с}$.
   - Дрон покрывает площадь сектора поиска.
3. **`HOVER_INSPECTION` (Реакция на ЧС):**
   - Нода слушает топик от Ромы: `/mchs/detection_event`.
   - Как только приходит событие с `"threat_level": "CRITICAL"` или `"WARNING"`, дрон мгновенно сбрасывает скорость до 0 и зависает над точкой на 5–10 секунд для детальной фиксации очага.
   - В этот момент вызывается сохранение протокола инцидента!
4. **`RTL` (Return to Launch):**
   - По завершении облета или по команде дрон разворачивается и возвращается в исходную точку $(0, 0, 8)$.
5. **`LAND`:**
   - Плавный спуск до касания земли ($Z \le 0.2\text{ м}$) и остановка моторов (`cmd_vel = 0`).

### Задача 2: Генератор официального протокола инцидента (`generate_report.py`)
- Скрипт принимает данные детекции и генерирует два файла:
  1. `incident_protocol.json` (машиночитаемый формат для интеграции с ЦУКС МЧС).
  2. `incident_protocol.txt` (официальный печатный бланк оперативного дежурного).
- Обязательные поля:
  * ID инцидента (`INC-2026-XXXX`);
  * Время фиксации (ISO 8601);
  * Тип угрозы (`ПОЖАР / ЗАГРЯЗНЕНИЕ / ЗАТОПЛЕНИЕ`);
  * Координаты полигона (X, Y);
  * Площадь поражения;
  * Рекомендуемые силы реагирования (например: *«Рекомендован наряд: 2 АЦ-40, автолестница АЛ-30, личный состав 8 чел.»*).

### Задача 3: Техническая пояснительная записка (`docs/technical_note.md`)
- Заполнить инженерное описание БПЛА:
  * Рама: легкий ударопрочный карбон (колесная база 650 мм).
  * Силовая установка: 4x мотора T-Motor U8 II 400KV + пропеллеры 15×5.
  * Питание: Smart LiPo 6S 22000 мАч (время автономного полета до 45 мин).
  * Полезная нагрузка: курсовая 4K камера Sony IMX + тепловизор FLIR Boson 640.
  * Бортовой вычислитель: Nvidia Jetson Orin Nano + Ubuntu 22.04 LTS / ROS 2 Humble.

---

## 3. Как запустить и тестировать

Работай в отдельной ветке:
```bash
git checkout -b feature/control-artem
```

Запуск в WSL:
```bash
python3 src/mchs_control/scripts/mchs_pilot_node.py
```
Проверка генератора отчетов (работает и на Windows, и в WSL):
```bash
python3 src/mchs_control/scripts/generate_report.py
```

---

## 4. Статус реализации (ветка `feature/pilot-artem`)

Автопилот и протокол реализованы по ТЗ выше.

### Что сделано
- FSM `TAKEOFF -> PATROL_GRID -> HOVER_INSPECTION -> RTL -> LAND -> FINISHED` в `mchs_pilot_node.py`.
- Подписка на `/mchs/detection_event`: `WARNING` — зависание 5 с, `CRITICAL` — 10 с, `ADVISORY` игнорируется.
- RTL по вектору на базу `(0, 0, 8)`; посадка до `Z <= 0,2 м` и остановка моторов.
- Защита от повторных событий (одна инспекция на очаг, радиус 15 м) и возврат на базу при АКБ < 20 %.
- Протокол `incident_protocol.json` + печатный бланк `incident_protocol.txt` в папке `reports/`.
- Техническая записка: `docs/technical_note.md`.
- Автотесты: `src/mchs_control/test/test_flight_planner.py`.

### Как проверить
```bash
# 1) прогон всего FSM без ROS и Gazebo (виртуальное время)
python3 src/mchs_control/scripts/mchs_pilot_node.py --dry-run --detect-at 8

# 2) другой тип ЧС и крупный очаг
python3 src/mchs_control/scripts/mchs_pilot_node.py --dry-run --hazard FLOOD --area 120

# 3) только протокол (JSON + печатный бланк)
python3 src/mchs_control/scripts/generate_report.py --hazard OIL_SPILL --area 60

# 4) автотесты
python3 src/mchs_control/test/test_flight_planner.py

# 5) реальный запуск в ROS 2
source /opt/ros/humble/setup.bash
python3 src/mchs_control/scripts/mchs_pilot_node.py
```

### Ручная имитация тревоги

Пока нода зрения не публикует события, инспекцию можно вызвать вручную:

```bash
ros2 topic pub --once /mchs/detection_event std_msgs/msg/String \
  "{data: '{\"hazard_type\": \"FIRE\", \"threat_level\": \"CRITICAL\", \"area_sq_m\": 48.5, \"target_geo\": {\"rel_x\": 12.4, \"rel_y\": -3.8}, \"confidence\": 0.94}'}"
```

### Параметры ROS 2

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `takeoff_altitude` | 8,0 м | рабочая высота патруля |
| `cruise_speed` | 2,0 м/с | скорость на галсе |
| `lanes` | 2 | число галсов «змейки» |
| `hover_warning_s` | 5,0 с | зависание при уровне `WARNING` |
| `hover_critical_s` | 10,0 с | зависание при уровне `CRITICAL` |
| `control_rate_hz` | 10,0 Гц | частота такта управления |
| `report_dir` | `reports/` | папка для протоколов |

Переопределение без правки кода:

```bash
python3 src/mchs_control/scripts/mchs_pilot_node.py --ros-args -p cruise_speed:=2.5 -p lanes:=3
```
