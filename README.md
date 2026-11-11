# 🚨 ОКО МЧС — Автономный комплекс БПЛА-мониторинга ЧС

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-3498DB?style=flat&logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo 11](https://img.shields.io/badge/Gazebo-11.10.2-FF6B35?style=flat)](http://gazebosim.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.5.4-5C3EE8?style=flat&logo=opencv)](https://opencv.org/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=flat&logo=python)](https://python.org/)

Проект команды на хакатоне **«Тихий дозор»** по кейсу **«Око МЧС»**.  
Единая система оперативного обнаружения лесных пожаров, задымлений и техногенных ЧС с борта БПЛА в режиме реального времени.

---

## 🎯 Архитектура решения

```text
               ┌──────────────────────────────────────────────┐
               │         3D-Симуляция Gazebo 11               │
               │  • Лесной массив, рельеф, очаг возгорания    │
               │  • Физика БПЛА, сенсоры: камера + барометр   │
               └──────────────────────┬───────────────────────┘
                                      │ /mchs/camera/image_raw
                                      ▼
               ┌──────────────────────────────────────────────┐
               │    Компьютерное зрение (mchs_vision)         │
               │  • Сегментация спектра огня (HSV)            │
               │  • Подавление шума и бликов                  │
               │  • Вычисление координат очага ЧС             │
               └──────────────┬───────────────────────────────┘
                              │
          ┌───────────────────┴───────────────────┐
          │ /mchs/fire_detected                   │
          ▼                                       ▼
┌──────────────────────────────┐        ┌──────────────────────────────┐
│  АРМ Оператора МЧС (mchs_ui) │        │  Автопилот БПЛА              │
│  • Тактическая карта         │        │  (mchs_control)              │
│  • Живое видео + Тепловизор  │        │  • Патрулирование сеткой     │
│  • Сирена тревоги            │        │  • Фиксация очага и RTL      │
│  • Логи телеметрии           │        │  • Генерация incident.json   │
└──────────────────────────────┘        └──────────────────────────────┘
```

---

## 🚀 Быстрый старт (Запуск за 30 секунд)

### 1. Запуск десктопного пульта оператора (Windows):
```bash
cd src/mchs_ui
python app.py
```

### 2. Запуск симуляции и алгоритмов (WSL2 / Ubuntu 22.04):
```bash
# Вход в терминал разработчика
wsl.exe -d Ubuntu-22.04

# Инициализация окружения ROS 2
source /opt/ros/humble/setup.bash
cd /root/oko-mchs-core

# Вариант А: Запуск полного комплекса (Gazebo + Vision + Pilot) в 1 клик
bash scripts/run_all.sh

# Вариант Б: Запуск только 3D-мира Gazebo с кастомным дроном и ареной ЧС
ros2 launch src/mchs_simulation/launch/simulation.launch.py
```

### 3. Динамическое перемещение объектов ЧС для жюри (отдельный терминал):
```bash
# Переместить пожар в заданные координаты прямо во время полета БПЛА:
python3 src/mchs_simulation/scripts/arena_controller.py --hazard fire --x 18.0 --y -6.0

# Переместить нефтяное пятно:
python3 src/mchs_simulation/scripts/arena_controller.py --hazard oil --x 10.0 --y 12.0

# Случайно перетасовать все 4 объекта ЧС:
python3 src/mchs_simulation/scripts/arena_controller.py --random
```

---

## 📁 Структура репозитория

```text
oko-mchs-core/
├── docs/                           # Документация хакатона
│   ├── task.md                     # Исходное ТЗ кейса «Око МЧС»
│   ├── regulation.md               # Регламент хакатона
│   ├── presentation_structure.md   # Структура презентации для Сони
│   └── technical_note_template.md  # Шаблон пояснительной записки для Артёма
├── src/                            # Исходный код пакетов
│   ├── mchs_simulation/            # 3D-мир Gazebo, модели, ланчи
│   ├── mchs_vision/                # Нода компьютерного зрения (Рома)
│   ├── mchs_control/               # Автопилот и отчетность (Артём)
│   └── mchs_ui/                    # АРМ Оператора на CustomTkinter (Соня)
├── scripts/                        # Скрипты автоматизации запуска
│   └── run_all.sh                  # Запуск всех модулей в 1 клик
└── README.md                       # Главная страница проекта
```

---

## 🛠 Технологический стек
* **ОС симуляции:** Ubuntu 22.04 LTS (WSL2)
* **Робототехнический стек:** ROS 2 Humble Hawksbill
* **Симулятор:** Gazebo 11.10.2 (Open Source Robotics Foundation)
* **Компьютерное зрение:** OpenCV 4.5.4, NumPy
* **GUI оператора:** CustomTkinter, Pillow (Python 3)
* **Аппаратное ускорение графики:** WSLg Direct3D 12 (Mesa 23.2)
