#!/usr/bin/env python3
"""CLI утилита калиброванного расчёта площади пожара по аэрофотосъёмке.

Пример использования:
    python scripts/estimate_fire_area.py --input photo.jpg --altitude 22.0 --output annotated.jpg --json result.json
"""

import argparse
import json
import os
import sys

import cv2

# Добавляем путь к пакетам
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VISION_PATH = os.path.join(REPO_ROOT, "src", "mchs_vision")
if VISION_PATH not in sys.path:
    sys.path.insert(0, VISION_PATH)
SRC_PATH = os.path.join(REPO_ROOT, "src")
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)

from mchs_vision.config import CameraConfig
from mchs_vision.fire_area import FireAreaEstimator
from mchs_vision.geometry import Pose


def main():
    parser = argparse.ArgumentParser(description="Расчёт площади пожара по фото БПЛА")
    parser.add_argument("--input", "-i", required=True, help="Путь к входному изображению (JPG/PNG)")
    parser.add_argument("--altitude", "-a", type=float, default=22.0, help="Высота полёта БПЛА в метрах (по умолчанию: 22.0)")
    parser.add_argument("--pitch", "-p", type=float, default=45.0, help="Угол наклона камеры в градусах (45 - камера largedrone, 90 - надир)")
    parser.add_argument("--hfov", type=float, default=80.0, help="Горизонтальный угол обзора камеры в градусах")
    parser.add_argument("--output", "-o", default=None, help="Путь для сохранения размеченного изображения")
    parser.add_argument("--json", "-j", default=None, help="Путь для сохранения JSON отчёта с метриками")

    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"[ОШИБКА] Файл не найден: {args.input}", file=sys.stderr)
        sys.exit(1)

    img = cv2.imread(args.input)
    if img is None:
        print(f"[ОШИБКА] Не удалось прочитать изображение: {args.input}", file=sys.stderr)
        sys.exit(1)

    h, w = img.shape[:2]
    camera = CameraConfig(
        width=w,
        height=h,
        hfov_deg=args.hfov,
        pitch_down_deg=args.pitch,
    )
    estimator = FireAreaEstimator(camera=camera)
    pose = Pose(x_m=0.0, y_m=0.0, z_m=args.altitude, yaw_rad=0.0)

    result = estimator.estimate(img, pose=pose, annotate=True)

    data = {
        "input_image": args.input,
        "altitude_m": args.altitude,
        "pitch_deg": args.pitch,
        "has_fire": result.has_fire,
        "has_smoke": result.has_smoke,
        "flame_area_px": result.flame_area_px,
        "flame_area_m2": result.flame_area_m2,
        "flame_area_ha": result.flame_area_ha,
        "smoke_area_px": result.smoke_area_px,
        "smoke_area_m2": result.smoke_area_m2,
        "smoke_area_ha": result.smoke_area_ha,
        "total_area_m2": result.total_area_m2,
        "total_area_ha": result.total_area_ha,
        "perimeter_m": result.perimeter_m,
        "gsd_m_per_px": result.gsd_m_per_px,
        "scale_bar_m": result.scale_bar_m,
        "centroid_px": list(result.centroid_px),
    }

    if args.output and result.annotated_image is not None:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        cv2.imwrite(args.output, result.annotated_image)
        print(f"Размеченное изображение сохранено: {args.output}")

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"Метрики сохранены: {args.json}")

    print("\n--- РЕЗУЛЬТАТЫ РАСЧЁТА ПЛОЩАДИ ПОЖАРА ---")
    print(f"Обнаружен огонь:     {'ДА' if result.has_fire else 'НЕТ'}")
    print(f"Площадь пламени:     {result.flame_area_m2:.2f} м² ({result.flame_area_ha:.4f} га)")
    print(f"Площадь задымления:  {result.smoke_area_m2:.2f} м² ({result.smoke_area_ha:.4f} га)")
    print(f"Суммарная площадь:   {result.total_area_m2:.2f} м² ({result.total_area_ha:.4f} га)")
    print(f"Периметр очага:      {result.perimeter_m:.2f} м")
    print(f"Разрешение GSD:      {result.gsd_m_per_px*100:.2f} см/пикс")


if __name__ == "__main__":
    main()
