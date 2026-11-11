#!/usr/bin/env python3
"""Сгенерировать синтетический датасет ЧС для демонстрации и проверки.

Пример:
    python3 scripts/make_test_scenes.py --output /tmp/oko_mchs_dataset

Результат:
    <output>/manifest.json        — позы БПЛА и наземная истина
    <output>/fire_critical.png    — кадры 640×480
    …
    <output>/world_map.png        — общий ортофотополигон (если не отключён)
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime
from typing import Dict

import cv2

# Пакет лежит на уровень выше scripts/.
PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from mchs_vision.simulated_scene import (FRAME_HEIGHT, FRAME_WIDTH,
                                         NADIR_ALTITUDE_M, SCALE_M_PER_PX,
                                         SCENE_YAW_DEG, SCENES, SyntheticArena)


def build_manifest(arena: SyntheticArena, output_dir: str,
                   image_files: Dict[str, str]) -> dict:
    scenes = []
    for name, spec in SCENES.items():
        pose = arena.pose_for(spec.center_px)
        scenes.append({
            "name": name,
            "title_ru": spec.title_ru,
            "image": image_files[name],
            "pose": {
                "x_m": round(pose.x_m, 3),
                "y_m": round(pose.y_m, 3),
                "z_m": round(pose.z_m, 3),
                "yaw_deg": round(pose.yaw_deg, 2),
            },
            "ground_truth": arena.ground_truth(spec.center_px),
        })

    return {
        "schema": "oko-mchs-synthetic-dataset/1.0",
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "seed": arena.seed,
        "scale_m_per_px": SCALE_M_PER_PX,
        "image_size": [FRAME_WIDTH, FRAME_HEIGHT],
        "camera": {
            "width": FRAME_WIDTH,
            "height": FRAME_HEIGHT,
            "hfov_deg": 80.0,
            "pitch_down_deg": 90.0,
            "mount_forward_m": 0.0,
            "mount_up_m": 0.0,
        },
        "uav_altitude_m": NADIR_ALTITUDE_M,
        "scene_yaw_deg": SCENE_YAW_DEG,
        "scenes": scenes,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="/tmp/oko_mchs_vision_dataset",
                        help="каталог датасета")
    parser.add_argument("--seed", type=int, default=7,
                        help="seed процедурной генерации")
    parser.add_argument("--no-world-map", action="store_true",
                        help="не сохранять общий ортофотополигон")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    arena = SyntheticArena(seed=args.seed)
    world = arena.render()

    image_files: Dict[str, str] = {}
    for name, spec in SCENES.items():
        frame = arena.crop(spec.center_px)
        filename = f"{name}.png"
        path = os.path.join(args.output, filename)
        if not cv2.imwrite(path, frame):
            print(f"[ERROR] не удалось сохранить {path}", file=sys.stderr)
            return 1
        image_files[name] = filename

    manifest = build_manifest(arena, args.output, image_files)
    manifest_path = os.path.join(args.output, "manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)

    if not args.no_world_map:
        cv2.imwrite(os.path.join(args.output, "world_map.png"), world)

    print(f"[OK] Датасет: {args.output}")
    print(f"     сцен: {len(SCENES)}, разрешение {FRAME_WIDTH}x{FRAME_HEIGHT}, "
          f"масштаб {SCALE_M_PER_PX} м/px, высота {NADIR_ALTITUDE_M} м")
    for scene in manifest["scenes"]:
        hazards = ", ".join(
            f"{gt['hazard_type']} {gt['area_m2']} м² ({gt['threat_level']})"
            for gt in scene["ground_truth"]
        ) or "нет очагов"
        print(f"     - {scene['name']:<16} {hazards}")
    print(f"[OK] Манифест: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
