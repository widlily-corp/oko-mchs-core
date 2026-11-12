# Аудит «ОКО МЧС» — 2026-11-12

## 1. Резюме

MODE FIX_SAFE, SCOPE ALL, ветка `audit/ai-fixes-20261112` (16 коммитов поверх `8b8fea7`). Внесены исправления обработки конфигурации, потоков UI, геометрии, фото, одометрии и запуска GPU. Строгий контракт аудита выполнен частично: для ряда изменений нет красного прогона до правки, транспорт ROS и полное демо не проверены. Финальный набор: 158 passed (3 прогона: 7.61/8.39/7.99 с), compileall и `git diff --check` чистые. UI переработан под демонстрацию: камера стала главным элементом, тактическая карта уменьшена до боковой панели, детали вынесены в отдельное окно; читаемость проверена матрицей в WSLg (масштабы 1.0/1.25/1.5) и нативным Windows-запуском (DPI 150%, обрезок 0). Офлайн-запуск Gazebo с GPU, полный ROS smoke и несколько гипотез остаются непроверенными; завершённый релизный аудит не заявляется.

## 2. Baseline

- Команды: `git status`, `git log`, 3× полный pytest, `compileall`, `bash -n scripts/run_all.sh`.
- Baseline: 137 passed (5.71/5.51/5.72 с), compileall exit 0. Дерево: `main` = `8b8fea7`, WIP `config/hazards.json` + `mchs_emergency_arena.world` сохранён в `/tmp/oko_mchs_wip.patch` и не коммитился.
- Окружение: WSL Ubuntu 22.04, ROS Humble, Gazebo 11, WSLg GPU D3D12 (AMD Radeon 780M, `glxinfo -B`: accelerated yes). Windows Python 3.14.6 для нативного UI smoke.

## 3. Находки

| ID | Sev | Подсистема | Файл:строка | Проблема | Доказательство | Исправление | Регрессия | Статус |
|---|---|---|---|---|---|---|---|---|
| A01 | P1 | UI/hazards | `src/mchs_ui/app.py::save_hazards` | Прямая запись + скрытый сбой replace, потеря неизвестных полей | E1: 3 failed до фикса | Атомарная запись через уникальный temp, сохранение лишних полей | `test_audit_hazards.py` | fixed (f88b2f6) |
| A02 | P1 | UI | `app.py` VideoClient/StateClient/RecordingWriter | `_stop` перекрывал `Thread._stop`, stop/join падали | E1: traceback до фикса | `_stop_event` (Event) | `test_audit_threads.py` | fixed (1cd0646) |
| A03 | P1 | vision | `events.py::save_victim_evidence_snapshot` | imwrite=False возвращал успех, файл не создан | E1: mocked failure red | Возврат `""` при отказе записи | `test_victim_detection.py` | fixed (1321b29) |
| A04 | P1 | vision | `geometry.py::pixel_to_ground` | NaN/Inf проходили сравнения | E1: NaN-тест red | Отклонение non-finite входов | `test_geometry.py` | fixed (f3cbdfd) |
| A05 | P1 | vision | `fire_area.py::estimate` | Ресайз кадра терял mount_right_m/min_range_m | E1: red-тест калибровки | Калибровка сохраняется при ресайзе | `test_fire_area.py` | fixed (12aa0fe) |
| A06 | P2 | vision | `vision_params.yaml` | enabled_classes без VICTIM | E2: профиль не включал детектор | VICTIM добавлен | прогон | fixed (f521cd6) |
| A07 | P1 | simulation | `run_all.sh` GPU-ветка | GPU-детект до сброса унаследованных software-флагов | E2: порядок строк 53-60 | unset до glxinfo-пробы | bash -n + ручной прогон | fixed (61b9299) |
| A08 | P1 | control | `mchs_pilot_route.py::_on_odom`, `mchs_pilot_node.py::on_odom` | NaN/Inf и нулевой кватернион принимались, last_odom_time обновлялся мусором | E1: 8 параметризованных red-кейсов на исходниках HEAD | Валидация finite + норма кватерниона, отказ без обновления freshness | `tests/test_ros_callback_validation.py` | fixed (33fd8f7) |
| A09 | P1 | vision | `mchs_vision_node.py::_save_incident_photo` | Пустой путь после отказа записи публиковался как `reports/incidents/`, счётчик рос | E1: red-тест consumer | Проверка пустого результата | `tests/test_ros_callback_validation.py` | fixed (1bb359a) |
| A10 | P2 | docs/tests | `run_all.sh:61,68,71`, `test_victim_detection.py:184` | Кириллица заменена на `????` при правках | E1: grep | Восстановлены UTF-8 строки | grep | fixed (f388329) |
| A11 | P1 | vision | `web_stream.py` | Отсутствует timeout клиентского сокета; daemon_threads уже включён стандартным ThreadingHTTPServer | E2: отсутствие timeout; E1: HTTP 503 и stop | socket timeout 10 с; тест не проверяет истечение timeout | `test_streamer_socket_timeout_is_configured` | implemented, verification incomplete (ba49fa9) |
| A12 | P1 | simulation | `arena_controller.py::update_hazards_config` | Общий `hazards.json.tmp`: параллельные записи ломали друг друга (replace No such file) | E1: барьер-тест red (False/True) | Уникальные NamedTemporaryFile + cleanup | `test_arena_hazards_sync.py` | fixed (5dd117f) |
| A13 | P2 | UI | `app.py` (перегруз главного экрана) | Карта/журнал/экспорт сжимали видео; мелкие шрифты при DPI 150% | E1: readability-matrix-before.txt | Камера — центр, карта 324×213 сбоку, детали в Toplevel, wraplength по pixel width | readability-matrix-final.txt (9 сценариев, 0 clipped/overlaps) + Windows smoke PROBLEMS 0 | fixed (c51f6a7, 0c84235) |

## 4. Коммиты

| Хеш | Сообщение | Что исправлено |
|---|---|---|
| f88b2f6 | fix(ui): preserve hazard configuration on concurrent writes and replace failures | A01 |
| 1cd0646 | fix(ui): keep worker threads joinable during shutdown | A02 |
| 1321b29 | fix(vision): report failed victim evidence writes | A03 |
| f3cbdfd | fix(vision): reject non-finite ground projection inputs | A04 |
| 12aa0fe | fix(vision): preserve camera calibration when resizing fire frames | A05 |
| f521cd6 | fix(vision): enable victim detection in ROS parameter profile | A06 |
| 38fee2e | feat(simulation): prefer WSL GPU rendering with software fallback | GPU-режим запуска (запрос пользователя) |
| c51f6a7 | feat(ui): focus operator view on camera and simplify demo controls | A13 |
| 0c84235 | fix(ui): keep labels readable at Windows display scaling | A13 |
| 2fbc14b | test(ui): cover atomic replacement failure for hazards | тесты A01 |
| 33fd8f7 | fix(control): reject invalid odometry callbacks | A08 |
| 1bb359a | fix(vision): do not report failed victim photo as saved | A09 |
| f388329 | chore: restore garbled audit strings in launcher and tests | A10 |
| 61b9299 | fix(simulation): clear inherited software GL flags before GPU probe | A07 |
| ba49fa9 | fix(vision): bound HTTP stream worker sockets | A11 |
| 5dd117f | fix(simulation): isolate temporary hazard files for concurrent writers | A12 |

## 5. Верификация (до/после)

| Проверка | Команда | Ожидание | Факт |
|---|---|---|---|
| Полный набор ×3 | pytest tests + 4 каталога | 0 failed, ≥137 | 158 passed (7.61/8.39/7.99 с) |
| Compile | `python3 -m compileall -q src scripts tests` | 0 | exit 0 |
| Shell | `bash -n scripts/run_all.sh` | 0 | exit 0 |
| Git | `git diff --check` | чисто | чисто |
| Мир идемпотентен | двойная генерация в изолированной копии HEAD | одинаковый SHA-256 | a5f49e1630888780 ×2 |
| CLI | `estimate_fire_area.py --help`, `diagnose_logs.py --summary` | код 0 | estimate 0; diagnose **1** (исторические crash-дампы 06:50, не регресс) |
| Imports | `import mchs_core, mchs_vision, mchs_control` | без ошибок | ok |
| UI WSLg | readability_matrix.py, масштабы 1.0/1.25/1.5 | 0 clipped/overlaps | 0 (readability-matrix-final.txt) |
| UI Windows | нативный запуск offline, DPI 150% | окно строится, 0 обрезок | 2100×1350 px, PROBLEMS 0 |
| GPU Gazebo | живой запуск gzserver | офлайн, без Fuel | **не проверено** (см. раздел 6) |

## 6. Не проверено / ограничения

- **Офлайн-запуск Gazebo с GPU не подтверждён.** Прошлый smoke обращался к `models.gazebosim.org`; живой прогон с `GAZEBO_MODEL_PATH` и проверкой отсутствия сетевых запросов не выполнен. GPU-режим включается корректно (unset до пробы, E2), но FPS-эффект не измерен.
- Живой ROS-транспорт (QoS, `ros2 topic info -v`, SIGINT-поведение нод) не проверялся — только статика и AST-исполнение callback.
- `diagnose_logs.py --summary` exit 1 из-за исторических crash-дампов GCS_UI 06:50–06:51 (ImportError ImageTk в системной PIL WSL). Это данные прошлых запусков, не регресс аудита; для чистого smoke нужен прогон на свежих логах.
- Визуальная оценка скриншотов (WSLg и Windows) человеком не выполнялась — только автоматические метрики обрезки/перекрытия.
- Потерянные обновления `hazards.json` между процессами (UI и arena_controller одновременно редактируют разные очаги) остаются last-writer-wins — архитектурный риск, требует согласования (файловый lock или merge).
- FireAreaEstimator: fallback на надирный GSD при невозможной проекции остаётся E2-подозрением, не доказано как дефект.
- `_last`-словари троттлинга не чистятся (рост памяти за часы) — E2, не измерено.
- H12 (расхождение доков: 127/71/137 тестов, README без mchs_core) подтверждено статически, правки не вносились — вне скоупа FIX_SAFE на этот прогон.
- H13 (нет CI/LICENSE/.gitattributes) зафиксировано, не исправлялось.

## 7. Топ-5 рисков для владельца

1. Офлайн-Gazebo с GPU не проверен живым прогоном — перед защитой сделать полный `run_all.sh` с логом и убедиться в отсутствии обращений к сети.
2. `hazards.json` last-writer-wins между UI и arena_controller — при одновременном перетаскивании разных очагов одна из правок теряется молча.
3. `diagnose_logs.py` на текущих логах всегда красный из-за старых crash-дампов — сохранить исторические дампы и проверить свежие логи отдельно; старые ошибки не доказывают актуальный сбой.
4. Троттлинг-словари растут без чистки — при многочасовом полёте возможен рост памяти (не измерено).
5. Доки расходятся с кодом (число тестов, отсутствие mchs_core в README) — жюри может заметить.

## 8. Что делать дальше (P2/P3)

- CI (pytest + compileall на push), `.gitattributes` (`*.md text eol=lf`), LICENSE — по согласованию.
- Рефакторинг `app.py` (2900+ строк) и дублирования pilot node/route — только после фиксации контрактов тестами.
- Чистка `_last`-словарей троттлинга по TTL.
- Синхронизация README/docs: 158 тестов, упоминание mchs_core, актуальные скриншоты UI.

## 9. Definition of Done

- [x] Полный pytest 0 failed, 3 прогона стабильны (158 ≥ 137 baseline)
- [x] compileall чисто
- [ ] Закрытие всех P0/P1 не доказано: last-writer-wins и непроверенные гипотезы остаются
- [ ] Строгий test-first на каждый фикс не выполнен; часть воспроизведений сделана ретроспективно на исходниках HEAD
- [x] Публичные контракты не изменены
- [x] Smoke: estimate CLI, imports, bash -n, headless UI
- [ ] Demo smoke run_all.sh с живым Gazebo — не выполнен (см. раздел 6)
- [ ] Полная синхронизация документации не закончена; README_UI обновлён
- [x] Исходный WIP оставлен вне индекса; дополнительно обнаружен untracked docs/presentation, не изменялся аудитом
- [x] Отчёт воспроизводим: команды, до/после, уровни доказательств

## Дополнение при передаче

Замечание пользователя о направлении камеры/карты: карта использует X вправо, Y вверх (`app.py::_world_to_canvas`), изображение — перспективу бортовой камеры с наклоном 45°. Горизонтальный проход карты может соответствовать движению в глубину кадра; ошибка поворота пока не воспроизведена на живом потоке. Статус E3/open, правка ориентации без подтверждения не внесена.

Артефакты E1 находятся в `/tmp/oko_mchs_audit_20261112`, они не включены в Git и могут исчезнуть при очистке /tmp. Некоторые проверки выполнялись повторно из-за особенностей передачи команд PowerShell→WSL. Windows smoke-инструментарий сам создал два crash-дампа (неверный master и приватный scaling API); эти сбои не принадлежат продуктовым сценариям.

Отчёт и журнал добавлены в Git при передаче по последней команде пользователя «заверши и пуш», чтобы результаты аудита были доступны в удалённой ветке. Runtime-логи и бинарники не добавлены.


## Исправление направления карты и камеры (после передачи)

По запросу пользователя устранён рассинхрон корпуса и курса движения.
E1: до правок 3 теста _steer падали: angular.z оставался 0 при поперечном курсе; отдельный тест UI показывал 0° вместо body_yaw_deg=90°.
- a4f5e36: маршрутный пилот перед движением поворачивает корпус/камеру к следующей точке, кратчайшая ошибка угла через atan2(sin, cos); скорость поворота ограничена 1 рад/с; движение удерживается, пока ошибка больше 15°. Применяется на маршруте и RTL через _steer; инспекция сохраняет свой контроллер наведения.
- 2b24826: карта использует body_yaw_deg, совместимый fallback yaw_deg для старых серверов. JSON и топики не менялись.
- Регрессии tests/test_ros_callback_validation.py: повороты в обе стороны, переход ±π, согласованный курс, приоритет body yaw и fallback. Полный набор 164 passed in 7.77s; compileall/diff check чистые.
- Живой полёт в Gazebo после изменения не проверен. Карта остаётся видом сверху, камера перспективной: одинаковый курс не означает одинаковое движение объектов в экранных координатах.

## Финальная передача в main

После доработок читаемости отчётов и увеличения модели человека: 168 passed in 7.70s, compileall и diff check чистые. Сохранены текущие координаты hazards.json и соответствующий мир по прямому запросу владельца. Исторические результаты выше относятся к соответствующим этапам аудита.
