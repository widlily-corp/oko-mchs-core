# Рабочий журнал аудита — 2026-11-12

Начало: 11:28 +05:00. MODE FIX_SAFE, ATOMIC, ветка audit/ai-fixes-20261112.
Исходный HEAD 8b8fea7. WIP: config/hazards.json, mchs_emergency_arena.world; untracked .codex/, docs/PROMPT_PROJECT_AUDIT_V2.md.
Снимки: /tmp/oko_mchs_wip.patch, /tmp/oko_mchs_wip_status.txt, /tmp/oko_mchs_index.patch. WIP не включать в коммиты.
Отчёт и журнал оставляем локальными по правилу «не коммить отчёты»; runtime evidence — /tmp/oko_mchs_audit_20261112.

## Карта
Gazebo -> image_raw + odom -> VisionPipeline -> detection_event -> pilot -> incident_protocol.json.
Vision -> HTTP snapshot/state -> Windows UI. UI -> mission.cmd -> route pilot -> cmd_vel.
hazards.json -> world generator / arena controller / UI / vision matching; mission.json -> pilot / UI.
Ядро CV отделено от ROS; logger/blackbox/video_reporter — общие утилиты.

## Baseline
Первый проход: 137 passed in 6.49s (wall 6.91s). ROS Humble и gzserver доступны.
Попытки bash через PowerShell -c столкнулись с кавычками, поэтому многострочные команды передаются в stdin bash; ошибки инфраструктуры не выдаются за результаты проверок.

## Чтение / гипотезы
Прочитаны README, package READMEs, phase2 plan, mission/hazards, run_all.sh; затем geometry, fire_area estimate/render, web_stream, evidence snapshot.
H1-H14 open до доказательств. H2/H3/H5/H8/H10 — приоритет.
E2: events.py save_victim_evidence_snapshot использует имя с точностью до секунды, cv2.imwrite return не проверяется, запись не атомарная. Проверить коллизии и отказ записи (H10).
E2: geometry.pixel_to_ground не отклоняет NaN, comparisons пропускают его. Проследить вход odom и выход events (H3/H5).
E2: fire_area.estimate при смене размера теряет mount_right_m/min_range_m; при невозможной проекции рассчитывает площадь по надирному GSD. Нужны независимые воспроизведения.
E2: HTTP handlers — whitelist paths, файловые пути не обслуживаются. Нет socket timeout. Bind 0.0.0.0 — оценить WSL consumer перед изменением.

## 11:48 ? A01 H2 configuration persistence
E1: hazards-before.txt: 3 failed (replace failure hidden, unknown fields dropped, concurrent reader sees empty JSON).
Fix UI save_hazards: unique temp files, no direct-write fallback, preserve extra fields. hazards-after: 5 passed; hazards-full: 140 passed in 6.03s. compileall/diff check clean.
Baseline archived: baseline-1/2/3 = 137 passed (5.71/5.51/5.72s); compile exit 0.
Scope note: concurrent edits of different objects still use last-writer-wins; this is not a cross-process transaction. Arena controller needs its own writer regression.


## 12:00 — Запрос по интерфейсу
Перестроен первый экран АРМ: видео стало основным, карта уменьшена до боковой панели, оставлена одна понятная кнопка запуска; журнал, телеметрия и экспорт раскрываются отдельно. Проверен headless GUI под WSLg в разрешениях 1400x900 и 1100x720; тревога в плане дополнительного теста.
GPU в WSL подтверждён через glxinfo: D3D12 (AMD Radeon 780M), Accelerated yes. В run_all.sh добавлен auto GPU с software fallback. Ограниченный Gazebo smoke запустил сервер, но загрузка моделей попыталась обратиться к models.gazebosim.org; не считать offline Gazebo подтверждённым, inspect лог отдельно.
После audit commits pytest: 147 passed in 7.14s, дополнительный прогон перед UI: 147 passed in 7.44s. UI edit пока локальный, не включать до обновления README и финальных проверок.

UI final: details moved to Toplevel to preserve camera size on 1100x720. Manual real Tk smoke: overview, details, small, alert, small_details; ERRORS []. Camera 980x598 / 680x418; map 324x213. Evidence /tmp/oko_mchs_audit_20261112/ui-smoke.txt and ui_*.png. No report generation side effects in alert smoke.

## Readability pass (user steering)
E1 matrix /tmp/oko_mchs_audit_20261112/readability_matrix.py: 1100x720, CTk widget scaling 1.0/1.25/1.5; idle, alert and details. Before: clipped labels at 125-150%, overlapping hazard captions on map. Fix: compact dynamic captions, separate camera controls row, scrollable incident card, CTk logical wrap lengths, collision-aware map caption placement. Real Windows DPI rendering still needs direct Windows smoke; CTk scale emulation is not equivalent. Matrix evidence in readability-matrix-final.txt.

## Продолжение: callback и фото
- E1 callback-before.txt: 9 failed на исходниках HEAD до фикса в изолированном /tmp/before_callbacks; после 9 passed. Исполняются реальные AST-тела callback без ROS, проверяется сохранение последнего валидного состояния.
- 33fd8f7: оба пилота отвергают NaN/Inf и нулевой кватернион. Маршрутный пилот не освежает last_odom_time ошибочными сообщениями. Проверка реального ROS транспорта этим не покрыта.
- Фото: consumer _save_incident_photo ошибочно увеличивал счётчик и возвращал reports/incidents/ после пустого результата helper. Добавлена проверка пустого результата.
- Отклонение от протокола: фиксы callback были внесены до регрессий; красный прогон выполнен на сохранённых исходниках HEAD отдельно. Коммит 33fd8f7 создан до полного прогона (целевые 8 passed); это не выдаётся за строгий test-first.
- Git выполнять в WSL: Windows git отказал dubious ownership; глобальное Windows исключение не добавлялось.

## 12:52 — Windows smoke
- E1: нативный запуск АРМ на Windows Python 3.14 (offline): окно 2100x1350 физических px (1400x900 логических, DPI 150%), PROBLEMS 0 по замеру обрезки Label. Скриншот C:/Users/Widlily/AppData/Local/Temp/oko_win_ui_overview.png, логи в /tmp/oko_mchs_audit_20261112. Визуальная оценка скриншота человеком не выполнялась.
- Побочный дефект инструментария: обращение к приватному _get_widget_scaling на CTk-окне вызывает AttributeError (name-mangling), не является багом продукта; crash-дамп от этого теста в logs/crashes от 12:51:59 — артефакт аудита, в индекс не попадает.
- Финальные прогоны: final-1/2/3 = 158 passed (7.61/8.39/7.99s). compileall, bash -n, git diff --check чисто.
- estimate_fire_area --help exit 0; diagnose_logs --summary exit 1 из-за исторических crash-дампов GCS_UI 06:50-06:51 (ImportError ImageTk в системной PIL WSL) — это данные прошлых запусков, не новый регресс.
- Гипотеза H9 частично: двойная генерация мира в изолированной копии HEAD → идентичный SHA-256 a5f49e1630888780. Офлайн-запуск Gazebo с GPU не подтверждён (прошлый smoke обращался к сети за моделями); помечено как не проверено.
- H2: воспроизведена и закрыта гонка arena_controller на общем hazards.json.tmp (синхронизированный барьером тест, до фикса False/True, после True/True, коммит 5dd117f). Потерянные обновления между процессами остаются архитектурным риском (last-writer-wins).

Передача: пользователь запросил завершение и push. Полный DoD не заявляется; в отчёте исправлены чрезмерные утверждения об HTTP, P0/P1 и test-first. Замечание ориентации камеры остаётся E3/open. Исходные WIP не включаются.


## Исправление направления карты и камеры (после передачи)

По запросу пользователя устранён рассинхрон корпуса и курса движения.
E1: до правок 3 теста _steer падали: angular.z оставался 0 при поперечном курсе; отдельный тест UI показывал 0° вместо body_yaw_deg=90°.
- a4f5e36: маршрутный пилот перед движением поворачивает корпус/камеру к следующей точке, кратчайшая ошибка угла через atan2(sin, cos); скорость поворота ограничена 1 рад/с; движение удерживается, пока ошибка больше 15°. Применяется на маршруте и RTL через _steer; инспекция сохраняет свой контроллер наведения.
- 2b24826: карта использует body_yaw_deg, совместимый fallback yaw_deg для старых серверов. JSON и топики не менялись.
- Регрессии tests/test_ros_callback_validation.py: повороты в обе стороны, переход ±π, согласованный курс, приоритет body yaw и fallback. Полный набор 164 passed in 7.77s; compileall/diff check чистые.
- Живой полёт в Gazebo после изменения не проверен. Карта остаётся видом сверху, камера перспективной: одинаковый курс не означает одинаковое движение объектов в экранных координатах.
