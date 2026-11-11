#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Генератор официального протокола инцидента МЧС (кейс «Око МЧС»).

Модуль формирует два документа, как требует ТЗ:

* ``incident_protocol.json`` — машиночитаемый протокол для ЦУКС и пульта
  оператора (схема совместима с тем, что выгружает АРМ оператора);
* ``incident_protocol.txt`` — печатный бланк оперативного дежурного.

Используется в двух ролях:

1. Библиотека: автопилот (``mchs_pilot_node.py``) вызывает
   ``generate_incident_protocol(...)`` в момент зависания над очагом.
2. Утилита: ручная проверка и демонстрация без дрона::

       python3 src/mchs_control/scripts/generate_report.py
       python3 src/mchs_control/scripts/generate_report.py --hazard FLOOD \
               --area 120 --x 40.5 --y -12.0 --confidence 0.91

Классификация угрозы повторяет контракт модуля зрения (``mchs_vision``):
до 15 м² — ``ADVISORY``, 15–50 м² — ``WARNING``, свыше 50 м² — ``CRITICAL``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from datetime import datetime, timedelta, timezone

# ---------------------------------------------------------------------------
# Константы
# ---------------------------------------------------------------------------

MSK = timezone(timedelta(hours=3))

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
DEFAULT_OUTPUT_DIR = os.path.join(REPO_ROOT, "reports")

STATION = "ЦУКС ГУ МЧС России"
PROTOCOL_VERSION = "3.0"

#: Порог площади (м²) -> уровень угрозы. Порядок важен: от высокого к низкому.
THREAT_THRESHOLDS = (
    (50.0, "CRITICAL"),
    (15.0, "WARNING"),
    (0.0, "ADVISORY"),
)

#: Человекочитаемые подписи уровней угрозы.
THREAT_LABELS = {
    "CRITICAL": "CRITICAL — критический",
    "WARNING": "WARNING — предупреждение",
    "ADVISORY": "ADVISORY — внимание",
}

#: Код ЧС -> (короткое название, расшифровка для бланка).
HAZARD_TYPES = {
    "FIRE": ("ПОЖАР", "очаг лесного пожара"),
    "SMOKE": ("ЗАДЫМЛЕНИЕ", "очаг задымления"),
    "OIL_SPILL": ("РАЗЛИВ НЕФТЕПРОДУКТОВ", "разлив нефтепродуктов на грунт"),
    "FLOOD": ("ЗАТОПЛЕНИЕ", "зона затопления / паводка"),
}

#: Рекомендуемый наряд по (код ЧС, уровень угрозы).
RESPONSE_PLANS = {
    ("FIRE", "CRITICAL"): "Рекомендован наряд: 2 АЦ-40, автолестница АЛ-30, личный состав 8 чел.",
    ("FIRE", "WARNING"): "Рекомендован наряд: 1 АЦ-40, личный состав 4 чел.",
    ("FIRE", "ADVISORY"): "Направить наблюдателя, повторный облёт через 15 минут.",
    ("SMOKE", "CRITICAL"): "Рекомендован наряд: 1 АЦ-40, разведгруппа 4 чел., проверка очага.",
    ("SMOKE", "WARNING"): "Направить разведывательную группу 2 чел., повторный облёт.",
    ("SMOKE", "ADVISORY"): "Взять зону под наблюдение, повторный облёт через 20 минут.",
    ("OIL_SPILL", "CRITICAL"): "Рекомендован наряд: 1 АЦ-40, скиммер, боновое ограждение, 6 чел.",
    ("OIL_SPILL", "WARNING"): "Направить бригаду 4 чел. с сорбентом, локализовать пятно.",
    ("OIL_SPILL", "ADVISORY"): "Зафиксировать пятно, уведомить природоохранную службу.",
    ("FLOOD", "CRITICAL"): "Рекомендован наряд: 1 АСМ, 2 плавсредства, личный состав 6 чел.",
    ("FLOOD", "WARNING"): "Направить спасательный расчёт 4 чел., оценить зону подтопления.",
    ("FLOOD", "ADVISORY"): "Вести наблюдение за уровнем воды, повторный облёт через 30 минут.",
}

#: Базовые действия, которые добавляются к рекомендации.
BASE_ACTIONS = (
    "Передать координаты очага в ЦУКС и ЕДДС муниципального района",
    "Оповестить оперативного дежурного пожарно-спасательной части",
    "Сохранить кадр с очагом как приложение к протоколу",
)

#: Дополнительные действия по уровню угрозы.
EXTRA_ACTIONS = {
    "CRITICAL": (
        "Запросить авиационное тушение (Ми-8 с ВСУ-5) при площади свыше 5000 м²",
        "Ограничить доступ людей в радиусе 2 км от очага",
    ),
    "WARNING": ("Назначить повторный облёт через 15 минут для контроля динамики",),
    "ADVISORY": ("Продолжить патрулирование сектора",),
}


# ---------------------------------------------------------------------------
# Классификация и тексты
# ---------------------------------------------------------------------------


def classify_threat(area_sq_m: float) -> str:
    """Возвращает уровень угрозы по площади поражения (контракт зрения)."""
    for threshold, level in THREAT_THRESHOLDS:
        if area_sq_m >= threshold:
            return level
    return "ADVISORY"


def hazard_info(hazard_code: str) -> tuple[str, str]:
    """Возвращает (короткое название, расшифровку) для кода ЧС."""
    return HAZARD_TYPES.get((hazard_code or "").upper(), ("НЕ ОПРЕДЕЛЁН", "тип уточняется"))


def recommended_response(hazard_code: str, threat_level: str) -> str:
    """Подбирает рекомендуемый наряд спасательных сил."""
    key = ((hazard_code or "").upper(), (threat_level or "").upper())
    if key in RESPONSE_PLANS:
        return RESPONSE_PLANS[key]
    return RESPONSE_PLANS[("FIRE", key[1])] if ("FIRE", key[1]) in RESPONSE_PLANS else RESPONSE_PLANS[("FIRE", "WARNING")]


def build_actions(hazard_code: str, threat_level: str) -> list[str]:
    """Формирует перечень действий: базовые + по уровню угрозы."""
    actions = list(BASE_ACTIONS)
    if hazard_code == "OIL_SPILL":
        actions.append("Уведомить Росприроднадзор и собственника участка")
    if hazard_code == "FLOOD":
        actions.append("Проверить подтопленные объекты инфраструктуры")
    actions.extend(EXTRA_ACTIONS.get(threat_level, ()))
    return actions


def make_incident_id(now: datetime) -> str:
    """ID инцидента в формате ТЗ: ``INC-2026-ЧЧММ``."""
    return f"INC-{now:%Y}-{now:%H%M}"


def local_offset_to_gps(
    x_m: float,
    y_m: float,
    lat0: float = 53.4129,
    lon0: float = 58.9814,
) -> tuple[float, float]:
    """Переводит локальные метры полигона (x — восток, y — север) в широту/долготу.

    Нужно для того, чтобы координаты очага в протоколе были понятны
    диспетчеру: полигон задан в метрах, а не в градусах. Для масштабов
    патрулирования плоской аппроксимации достаточно.
    """
    lat = lat0 + y_m / 111_320.0
    lon = lon0 + x_m / (111_320.0 * math.cos(math.radians(lat0)))
    return round(lat, 7), round(lon, 7)


# ---------------------------------------------------------------------------
# Сборка протокола
# ---------------------------------------------------------------------------


def build_incident_protocol(
    hazard_code: str = "FIRE",
    threat_level: str | None = None,
    area_sq_m: float = 35.2,
    target_geo: dict | None = None,
    confidence: float = 0.95,
    incident_id: str | None = None,
    hazard_label_ru: str | None = None,
    track_id: str = "",
    distance_m: float | None = None,
    bearing_deg: float | None = None,
    drone_telemetry: dict | None = None,
    flight_summary: dict | None = None,
    drone_id: str = "MCHS-UAV-04",
    station: str = STATION,
) -> dict:
    """Собирает словарь протокола (без записи на диск).

    Аргументы повторяют поля события ``/mchs/detection_event``, которое
    публикует модуль зрения, поэтому автопилоту достаточно передать событие
    и свою телеметрию.
    """
    now = datetime.now().astimezone()
    haze_code = (hazard_code or "FIRE").upper()
    threat = (threat_level or classify_threat(area_sq_m)).upper()
    geo = dict(target_geo or {})
    telemetry = dict(drone_telemetry or {})

    abs_x = float(geo.get("abs_x", telemetry.get("grid_x_m", 0.0)))
    abs_y = float(geo.get("abs_y", telemetry.get("grid_y_m", 0.0)))
    rel_x = float(geo.get("rel_x", 0.0))
    rel_y = float(geo.get("rel_y", 0.0))

    short_name, description = hazard_info(haze_code)
    # Зрение может передать готовые координаты WGS-84 — доверяем им.
    event_lat = geo.get("lat", geo.get("latitude"))
    event_lon = geo.get("lon", geo.get("longitude"))
    if event_lat is not None and event_lon is not None:
        latitude, longitude = round(float(event_lat), 7), round(float(event_lon), 7)
    else:
        latitude, longitude = local_offset_to_gps(abs_x, abs_y)

    return {
        "incident_id": incident_id or make_incident_id(now),
        "timestamp": now.isoformat(timespec="seconds"),
        "timestamp_msk": now.astimezone(MSK).strftime("%d.%m.%Y %H:%M:%S"),
        "station": station,
        "hazard_type": short_name,
        "hazard_code": haze_code,
        "hazard_description": description,
        "hazard_label_ru": hazard_label_ru or description,
        "track_id": track_id,
        "threat_level": threat,
        "estimated_area_sq_m": round(float(area_sq_m), 1),
        "target_geo": {
            "abs_x": round(abs_x, 2),
            "abs_y": round(abs_y, 2),
            "rel_x": round(rel_x, 2),
            "rel_y": round(rel_y, 2),
            "latitude": latitude,
            "longitude": longitude,
            "distance_m": round(float(distance_m), 2) if distance_m is not None else None,
            "bearing_deg": round(float(bearing_deg), 1) if bearing_deg is not None else None,
        },
        "confidence": round(float(confidence), 3),
        "drone_telemetry": {
            "drone_id": drone_id,
            "altitude_m": round(float(telemetry.get("altitude_m", 8.0)), 1),
            "battery_percent": round(float(telemetry.get("battery_percent", 100.0)), 0),
            "grid_x_m": round(abs_x, 2),
            "grid_y_m": round(abs_y, 2),
        },
        "recommended_response": recommended_response(haze_code, threat),
        "recommended_actions": build_actions(haze_code, threat),
        "flight_summary": dict(flight_summary or {}),
        "protocol_version": PROTOCOL_VERSION,
    }


def render_protocol_text(protocol: dict) -> str:
    """Формирует печатный бланк оперативного дежурного (``incident_protocol.txt``)."""
    geo = protocol["target_geo"]
    telemetry = protocol["drone_telemetry"]
    threat = protocol["threat_level"]
    lines = [
        "=" * 72,
        "ПРОТОКОЛ ИНЦИДЕНТА № {}".format(protocol["incident_id"]).center(72),
        protocol["station"].center(72),
        "=" * 72,
        f"Дата и время фиксации ....... {protocol['timestamp_msk']} (МСК)",
        f"Тип чрезвычайной ситуации ... {protocol['hazard_type']} ({protocol['hazard_description']})",
        f"Уровень угрозы .............. {THREAT_LABELS.get(threat, threat)}",
        f"Координаты очага (X, Y) ..... {geo['abs_x']:.1f} м, {geo['abs_y']:.1f} м",
        f"Координаты WGS-84 ........... {geo['latitude']:.5f} с.ш., {geo['longitude']:.5f} в.д.",
        f"Площадь поражения ........... {protocol['estimated_area_sq_m']:.1f} м²",
        f"Уверенность детектора ....... {protocol['confidence'] * 100:.0f} %",
        f"Борт ........................ {telemetry['drone_id']}",
        f"Высота / заряд АКБ .......... {telemetry['altitude_m']:.1f} м / {telemetry['battery_percent']:.0f} %",
        "-" * 72,
        "Рекомендуемый наряд спасательных сил:",
        "   " + protocol["recommended_response"],
        "-" * 72,
        "Рекомендуемые действия:",
    ]
    for index, action in enumerate(protocol["recommended_actions"], start=1):
        lines.append(f"   {index}. {action}")

    summary = protocol.get("flight_summary") or {}
    if summary:
        lines.append("-" * 72)
        lines.append("Сводка полёта БПЛА (на момент фиксации):")
        lines.append(
            "   время {:.1f} с, дистанция {:.0f} м, зависаний {}, фаз: {}".format(
                summary.get("elapsed_s", 0.0),
                summary.get("distance_flown_m", 0.0),
                summary.get("inspections", 0),
                len(summary.get("phases", [])),
            )
        )

    lines.extend(
        [
            "=" * 72,
            "Оперативный дежурный ЦУКС: ______________ / ____________________ /",
            "Документ сформирован автоматически бортовым комплексом «Око МЧС»",
            "=" * 72,
            "",
        ]
    )
    return "\n".join(lines)


def write_protocol_files(protocol: dict, out_dir: str | None) -> tuple[str | None, str | None]:
    """Пишет JSON и TXT в ``out_dir``. ``None`` — работаем без записи на диск."""
    if not out_dir:
        return None, None
    os.makedirs(out_dir, exist_ok=True)
    json_path = os.path.join(out_dir, "incident_protocol.json")
    txt_path = os.path.join(out_dir, "incident_protocol.txt")
    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(protocol, handle, ensure_ascii=False, indent=2)
    with open(txt_path, "w", encoding="utf-8") as handle:
        handle.write(render_protocol_text(protocol))
    return json_path, txt_path


def generate_incident_protocol(
    hazard_code: str = "FIRE",
    threat_level: str | None = None,
    area_sq_m: float = 35.2,
    target_geo: dict | None = None,
    confidence: float = 0.95,
    incident_id: str | None = None,
    hazard_label_ru: str | None = None,
    track_id: str = "",
    distance_m: float | None = None,
    bearing_deg: float | None = None,
    drone_telemetry: dict | None = None,
    flight_summary: dict | None = None,
    drone_id: str = "MCHS-UAV-04",
    out_dir: str | None = DEFAULT_OUTPUT_DIR,
    verbose: bool = True,
) -> dict:
    """Собирает протокол, сохраняет оба файла и возвращает словарь протокола."""
    protocol = build_incident_protocol(
        hazard_code=hazard_code,
        threat_level=threat_level,
        area_sq_m=area_sq_m,
        target_geo=target_geo,
        confidence=confidence,
        incident_id=incident_id,
        hazard_label_ru=hazard_label_ru,
        track_id=track_id,
        distance_m=distance_m,
        bearing_deg=bearing_deg,
        drone_telemetry=drone_telemetry,
        flight_summary=flight_summary,
        drone_id=drone_id,
    )
    json_path, txt_path = write_protocol_files(protocol, out_dir)
    protocol["archive"] = {"json": json_path, "txt": txt_path}
    if verbose:
        print_console_summary(protocol)
    return protocol


#: Совместимость с прежним названием функции (её вызывал старый автопилот).
generate_incident_report = generate_incident_protocol


def print_console_summary(protocol: dict) -> None:
    """Печатает краткую сводку в консоль (для логов ноды и защиты)."""
    geo = protocol["target_geo"]
    archive = protocol.get("archive") or {}
    print()
    print("=" * 64)
    print("   ПРОТОКОЛ ИНЦИДЕНТА " + protocol["incident_id"])
    print("=" * 64)
    print(f"  Время            : {protocol['timestamp_msk']} (МСК)")
    print(f"  Тип ЧС           : {protocol['hazard_type']} ({protocol['hazard_code']})")
    print(f"  Уровень угрозы   : {protocol['threat_level']}")
    print(f"  Координаты (X, Y): {geo['abs_x']:.1f} м, {geo['abs_y']:.1f} м")
    print(f"  Площадь          : {protocol['estimated_area_sq_m']:.1f} м²")
    print(f"  Уверенность      : {protocol['confidence'] * 100:.0f} %")
    print(f"  Наряд            : {protocol['recommended_response']}")
    if archive.get("json"):
        print(f"  JSON             : {archive['json']}")
    if archive.get("txt"):
        print(f"  Бланк (TXT)      : {archive['txt']}")
    print("=" * 64)
    print()


# ---------------------------------------------------------------------------
# Командная строка
# ---------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Разбирает аргументы командной строки."""
    parser = argparse.ArgumentParser(
        description="Формирование протокола инцидента МЧС «Око МЧС» (JSON + печатный бланк).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Примеры:\n"
            "  python3 generate_report.py\n"
            "  python3 generate_report.py --hazard OIL_SPILL --area 60 --x 30 --y -10\n"
            "  python3 generate_report.py --hazard FLOOD --area 120 --x 40.5 --y -12 --out-dir .\n"
        ),
    )
    parser.add_argument("--hazard", default="FIRE", choices=sorted(HAZARD_TYPES), help="код ЧС")
    parser.add_argument("--threat", default=None, choices=sorted(THREAT_LABELS), help="уровень угрозы (по умолчанию — по площади)")
    parser.add_argument("--area", type=float, default=35.2, help="площадь поражения, м²")
    parser.add_argument("--x", type=float, default=14.8, help="координата X очага на полигоне, м")
    parser.add_argument("--y", type=float, default=-5.2, help="координата Y очага на полигоне, м")
    parser.add_argument("--rel-x", type=float, default=4.8, help="смещение очага от дрона по X, м")
    parser.add_argument("--rel-y", type=float, default=-1.2, help="смещение очага от дрона по Y, м")
    parser.add_argument("--confidence", type=float, default=0.95, help="уверенность детектора, 0..1")
    parser.add_argument("--altitude", type=float, default=8.0, help="высота дрона, м")
    parser.add_argument("--battery", type=float, default=96.0, help="заряд АКБ, %%")
    parser.add_argument("--drone-id", default="MCHS-UAV-04", help="бортовой номер БПЛА")
    parser.add_argument("--out-dir", default=DEFAULT_OUTPUT_DIR, help="папка для протокола")
    parser.add_argument("--quiet", action="store_true", help="не печатать сводку в консоль")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Точка входа утилиты. Возвращает код возврата процесса."""
    args = parse_args(argv)
    try:
        generate_incident_protocol(
            hazard_code=args.hazard,
            threat_level=args.threat,
            area_sq_m=args.area,
            target_geo={"abs_x": args.x, "abs_y": args.y, "rel_x": args.rel_x, "rel_y": args.rel_y},
            confidence=args.confidence,
            drone_telemetry={"altitude_m": args.altitude, "battery_percent": args.battery},
            drone_id=args.drone_id,
            out_dir=args.out_dir,
            verbose=not args.quiet,
        )
    except OSError as error:  # нет прав на запись в папку протоколов
        print(f"[ОШИБКА] Не удалось сохранить протокол: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
