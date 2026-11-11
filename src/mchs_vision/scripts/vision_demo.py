#!/usr/bin/env python3
"""Офлайн-демонстрация бортового зрения «Око МЧС» без ROS и Gazebo.

Примеры:
    # Датасет, созданный make_test_scenes.py
    python3 scripts/vision_demo.py --input /tmp/oko_mchs_vision_dataset \
        --output /tmp/oko_mchs_vision_demo --json --csv

    # Одно изображение с параметрами камеры по умолчанию
    python3 scripts/vision_demo.py --input frame.png --altitude 40 --yaw 90

    # Видео: аннотированный MP4 + журнал событий
    python3 scripts/vision_demo.py --input flight.mp4 --output /tmp/out --json

Скрипт не требует ROS: используется тот же :class:`VisionPipeline`, что и в
бортовой ноде, поэтому результаты демонстрации соответствуют полёту.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from mchs_vision.config import CameraConfig, DetectorConfig
from mchs_vision.geometry import Pose
from mchs_vision.pipeline import PipelineResult, VisionPipeline

IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
VIDEO_EXTENSIONS = {".mp4", ".avi", ".mov", ".mkv", ".webm"}


def load_manifest(path: str) -> Optional[dict]:
    manifest_path = path
    if os.path.isdir(path):
        manifest_path = os.path.join(path, "manifest.json")
    if not os.path.isfile(manifest_path):
        return None
    with open(manifest_path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def build_camera(manifest: Optional[dict], args, frame_shape: Tuple[int, int]
                 ) -> CameraConfig:
    height, width = frame_shape
    if manifest and "camera" in manifest:
        camera = manifest["camera"]
        return CameraConfig(
            width=int(camera.get("width", width)),
            height=int(camera.get("height", height)),
            hfov_deg=float(camera.get("hfov_deg", args.hfov)),
            pitch_down_deg=float(camera.get("pitch_down_deg", args.pitch)),
        )
    return CameraConfig(
        width=width,
        height=height,
        hfov_deg=args.hfov,
        pitch_down_deg=args.pitch,
    )


def build_pipeline(camera: CameraConfig, min_hits: int,
                   title: str = "MCHS OPTICAL RECON // OFFLINE DEMO"
                   ) -> VisionPipeline:
    config = DetectorConfig(track_min_hits=max(1, min_hits))
    return VisionPipeline(camera=camera, detector_config=config, title=title)


def pose_for_scene(scene: Optional[dict], args) -> Pose:
    if scene and "pose" in scene:
        pose = scene["pose"]
        return Pose.from_deg(
            x_m=float(pose.get("x_m", 0.0)),
            y_m=float(pose.get("y_m", 0.0)),
            z_m=float(pose.get("z_m", args.altitude)),
            yaw_deg=float(pose.get("yaw_deg", args.yaw)),
        )
    return Pose.from_deg(args.x, args.y, args.altitude, args.yaw)


def evaluate_scene(scene_name: str, ground_truth: Sequence[dict],
                   result: PipelineResult, match_radius_m: float) -> List[dict]:
    """Сопоставить детекции с эталоном и вернуть строки метрик."""
    records = list(result.records)
    used = set()
    rows: List[dict] = []

    for gt in ground_truth:
        gt_type = gt["hazard_type"]
        gt_xy = gt["center_xy_m"]
        best_index = None
        best_distance = float("inf")

        for index, record in enumerate(records):
            if index in used or record.hazard_type != gt_type:
                continue
            distance = float(np.hypot(record.abs_xy[0] - gt_xy[0],
                                      record.abs_xy[1] - gt_xy[1]))
            if distance < best_distance:
                best_distance = distance
                best_index = index

        if best_index is None or best_distance > match_radius_m:
            rows.append({
                "scene": scene_name,
                "status": "FN",
                "gt_type": gt_type,
                "gt_area_m2": round(float(gt["area_m2"]), 2),
                "gt_threat": gt["threat_level"],
                "det_type": "",
                "det_area_m2": 0.0,
                "det_threat": "",
                "center_error_m": 0.0,
                "area_error_pct": 0.0,
                "confidence": 0.0,
                "track_id": "",
            })
            continue

        used.add(best_index)
        record = records[best_index]
        area_error = abs(record.smoothed_area_m2 - float(gt["area_m2"])) \
            / max(float(gt["area_m2"]), 0.01) * 100.0
        rows.append({
            "scene": scene_name,
            "status": "TP",
            "gt_type": gt_type,
            "gt_area_m2": round(float(gt["area_m2"]), 2),
            "gt_threat": gt["threat_level"],
            "det_type": record.hazard_type,
            "det_area_m2": round(record.smoothed_area_m2, 2),
            "det_threat": record.threat_level,
            "center_error_m": round(best_distance, 3),
            "area_error_pct": round(area_error, 2),
            "confidence": round(record.confidence, 3),
            "track_id": record.track_id,
        })

    for index, record in enumerate(records):
        if index in used:
            continue
        rows.append({
            "scene": scene_name,
            "status": "FP",
            "gt_type": "",
            "gt_area_m2": 0.0,
            "gt_threat": "",
            "det_type": record.hazard_type,
            "det_area_m2": round(record.smoothed_area_m2, 2),
            "det_threat": record.threat_level,
            "center_error_m": 0.0,
            "area_error_pct": 0.0,
            "confidence": round(record.confidence, 3),
            "track_id": record.track_id,
        })

    return rows


def summarize(rows: Sequence[dict]) -> dict:
    tp = [row for row in rows if row["status"] == "TP"]
    fp = [row for row in rows if row["status"] == "FP"]
    fn = [row for row in rows if row["status"] == "FN"]

    precision = len(tp) / (len(tp) + len(fp)) if (tp or fp) else 0.0
    recall = len(tp) / (len(tp) + len(fn)) if (tp or fn) else 0.0
    threat_ok = sum(1 for row in tp if row["gt_threat"] == row["det_threat"])
    center_errors = [row["center_error_m"] for row in tp]
    area_errors = [row["area_error_pct"] for row in tp]

    return {
        "ground_truth": len(tp) + len(fn),
        "detections": len(tp) + len(fp),
        "tp": len(tp),
        "fp": len(fp),
        "fn": len(fn),
        "precision": round(precision, 3),
        "recall": round(recall, 3),
        "mean_center_error_m": round(float(np.mean(center_errors)), 3)
        if center_errors else 0.0,
        "max_center_error_m": round(float(np.max(center_errors)), 3)
        if center_errors else 0.0,
        "mean_abs_area_error_pct": round(float(np.mean(area_errors)), 2)
        if area_errors else 0.0,
        "threat_accuracy": round(threat_ok / len(tp), 3) if tp else 0.0,
    }


def print_rows(rows: Sequence[dict]) -> None:
    header = (f"{'SCENE':<16} {'GT':<18} {'DET':<18} {'AREA ERR':>9} "
              f"{'POS ERR':>8} {'THREAT':<10} STATUS")
    print(header)
    print("-" * len(header))
    for row in rows:
        gt = (f"{row['gt_type']} {row['gt_area_m2']:.1f}"
              if row["gt_type"] else "-")
        det = (f"{row['det_type']} {row['det_area_m2']:.1f}"
               if row["det_type"] else "-")
        threat = (f"{row['gt_threat']}/{row['det_threat']}"
                  if row["gt_threat"] and row["det_threat"] else "-")
        print(f"{row['scene']:<16} {gt:<18} {det:<18} "
              f"{row['area_error_pct']:>8.1f}% {row['center_error_m']:>7.2f}m "
              f"{threat:<10} {row['status']}")


def save_metrics(rows: Sequence[dict], summary: dict, output_dir: str) -> None:
    csv_path = os.path.join(output_dir, "metrics.csv")
    fieldnames = ["scene", "status", "gt_type", "gt_area_m2", "gt_threat",
                  "det_type", "det_area_m2", "det_threat", "center_error_m",
                  "area_error_pct", "confidence", "track_id"]
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    with open(os.path.join(output_dir, "metrics.json"), "w",
              encoding="utf-8") as handle:
        json.dump({"summary": summary, "rows": rows}, handle,
                  ensure_ascii=False, indent=2)


def process_images(paths: List[str], scenes: Dict[str, dict], args,
                   output_dir: str) -> Tuple[List[dict], List[dict], int]:
    annotated_dir = os.path.join(output_dir, "annotated")
    os.makedirs(annotated_dir, exist_ok=True)

    all_events: List[dict] = []
    all_rows: List[dict] = []
    processed = 0
    pipeline: Optional[VisionPipeline] = None
    camera_signature = None

    for index, path in enumerate(paths):
        frame = cv2.imread(path, cv2.IMREAD_COLOR)
        if frame is None:
            print(f"[WARN] не удалось прочитать {path}", file=sys.stderr)
            continue

        scene = scenes.get(os.path.basename(path))
        camera = build_camera(args.manifest, args, frame.shape[:2])
        signature = (camera.width, camera.height, camera.hfov_deg,
                     camera.pitch_down_deg, args.min_hits)
        if pipeline is None or signature != camera_signature:
            pipeline = build_pipeline(camera, args.min_hits)
            camera_signature = signature
        elif not args.no_reset:
            # Каждое изображение датасета — независимая сцена. Без сброса
            # трекер склеит очаги из разных кадров, ведь оба центрированы.
            pipeline.reset_tracking()

        pose = pose_for_scene(scene, args)
        timestamp = datetime.now().astimezone() + timedelta(seconds=index * 3)
        result = pipeline.process(frame, pose, timestamp, draw=not args.no_hud)

        scene_name = scene["name"] if scene else os.path.splitext(
            os.path.basename(path))[0]

        for event in result.events:
            event = dict(event)
            event["scene"] = scene_name
            event["source_image"] = os.path.basename(path)
            all_events.append(event)

        if not args.no_hud:
            out_path = os.path.join(annotated_dir, f"{scene_name}_annotated.png")
            cv2.imwrite(out_path, result.annotated)

        ground_truth = scene.get("ground_truth", []) if scene else []
        rows = evaluate_scene(scene_name, ground_truth, result,
                              args.match_radius)
        all_rows.extend(rows)

        if index == 0 and args.debug_masks:
            for class_name, mask in pipeline.detector.class_masks(frame).items():
                cv2.imwrite(os.path.join(output_dir, f"mask_{class_name}.png"),
                            mask)

        processed += 1
        if not ground_truth:
            for record in result.records:
                print(f"  {scene_name}: {record.hazard_type} "
                      f"{record.smoothed_area_m2:.1f} м² "
                      f"{record.threat_level} conf={record.confidence:.2f} "
                      f"@ ({record.abs_xy[0]:.1f}, {record.abs_xy[1]:.1f})")
        if args.limit and processed >= args.limit:
            break

    return all_events, all_rows, processed


def process_video(path: str, args, output_dir: str
                  ) -> Tuple[List[dict], int]:
    capture = cv2.VideoCapture(path)
    if not capture.isOpened():
        raise RuntimeError(f"Не удалось открыть видео: {path}")

    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

    camera = build_camera(args.manifest, args, (height, width))
    pipeline = build_pipeline(camera, args.min_hits)

    scale = args.scale
    out_width, out_height = int(width * scale), int(height * scale)
    out_path = os.path.join(output_dir,
                            f"{os.path.splitext(os.path.basename(path))[0]}"
                            f"_annotated.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps,
                             (out_width, out_height))
    if not writer.isOpened():
        capture.release()
        raise RuntimeError("Не удалось открыть VideoWriter (кодек mp4v)")

    events: List[dict] = []
    index = 0
    start = datetime.now().astimezone()
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        pose = Pose.from_deg(args.x, args.y, args.altitude, args.yaw)
        timestamp = start + timedelta(seconds=index / fps)
        result = pipeline.process(frame, pose, timestamp, draw=not args.no_hud)

        for event in result.events:
            event = dict(event)
            event["frame_index"] = index
            event["video_time_s"] = round(index / fps, 2)
            events.append(event)

        output = result.annotated
        if scale != 1.0:
            output = cv2.resize(output, (out_width, out_height),
                                interpolation=cv2.INTER_CUBIC)
        writer.write(output)

        index += 1
        if index % 60 == 0:
            percent = (index / total * 100.0) if total else 0.0
            print(f"  кадр {index}/{total or '?'} ({percent:.0f}%), "
                  f"событий: {len(events)}")
        if args.limit and index >= args.limit:
            break

    writer.release()
    capture.release()
    print(f"[OK] Видео: {out_path} ({index} кадров, {fps:.1f} FPS)")
    return events, index


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", required=True,
                        help="изображение, каталог или видео")
    parser.add_argument("--output", default="/tmp/oko_mchs_vision_demo",
                        help="каталог результатов (по умолчанию вне репозитория)")
    parser.add_argument("--manifest", default=None,
                        help="путь к manifest.json (иначе ищется рядом)")
    parser.add_argument("--altitude", type=float, default=38.14,
                        help="высота полёта, м")
    parser.add_argument("--x", type=float, default=0.0, help="позиция X, м")
    parser.add_argument("--y", type=float, default=0.0, help="позиция Y, м")
    parser.add_argument("--yaw", type=float, default=90.0,
                        help="рыскание, град (90 = север-вверх)")
    parser.add_argument("--pitch", type=float, default=90.0,
                        help="наклон камеры вниз, град (90 = надир)")
    parser.add_argument("--hfov", type=float, default=80.0,
                        help="горизонтальный FOV, град")
    parser.add_argument("--min-hits", type=int, default=1,
                        help="кадров подряд для подтверждения детекции")
    parser.add_argument("--match-radius", type=float, default=15.0,
                        help="радиус сопоставления с эталоном, м")
    parser.add_argument("--scale", type=float, default=1.5,
                        help="масштаб выходного видео")
    parser.add_argument("--limit", type=int, default=0,
                        help="ограничить число изображений/кадров")
    parser.add_argument("--no-hud", action="store_true",
                        help="не рисовать HUD и не писать аннотированные кадры")
    parser.add_argument("--no-reset", action="store_true",
                        help="не сбрасывать трекер между изображениями "
                             "(если это последовательные кадры одного полёта)")
    parser.add_argument("--debug-masks", action="store_true",
                        help="сохранить HSV-маски классов для первого кадра")
    parser.add_argument("--json", action="store_true",
                        help="сохранить события в events.json")
    parser.add_argument("--csv", action="store_true",
                        help="сохранить метрики в metrics.csv")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    args.manifest = args.manifest or load_manifest(args.input)

    is_video = os.path.isfile(args.input) \
        and os.path.splitext(args.input)[1].lower() in VIDEO_EXTENSIONS
    start = datetime.now()

    if is_video:
        events, processed = process_video(args.input, args, args.output)
        summary = {"video_frames": processed, "events": len(events)}
        if args.json:
            with open(os.path.join(args.output, "events.json"), "w",
                      encoding="utf-8") as handle:
                json.dump(events, handle, ensure_ascii=False, indent=2)
        with open(os.path.join(args.output, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)
        print(f"[OK] События: {len(events)}, результаты: {args.output}")
        return 0

    # Изображения
    if os.path.isdir(args.input):
        manifest = args.manifest
        scenes: Dict[str, dict] = {}
        paths: List[str] = []
        if manifest:
            for scene in manifest.get("scenes", []):
                image_name = scene.get("image")
                image_path = os.path.join(args.input, image_name) \
                    if image_name else None
                if image_path and os.path.isfile(image_path):
                    paths.append(image_path)
                    scenes[os.path.basename(image_path)] = scene
        if not paths:
            for name in sorted(os.listdir(args.input)):
                if os.path.splitext(name)[1].lower() in IMAGE_EXTENSIONS \
                        and name != "world_map.png":
                    paths.append(os.path.join(args.input, name))
    else:
        paths = [args.input]
        scenes = {}

    if not paths:
        print("[ERROR] не найдено ни одного изображения", file=sys.stderr)
        return 1

    events, rows, processed = process_images(paths, scenes, args, args.output)
    summary = summarize(rows)

    if rows:
        print_rows(rows)
        print("-" * 80)
        print(f"GT: {summary['ground_truth']}  детекций: {summary['detections']}  "
              f"TP/FP/FN: {summary['tp']}/{summary['fp']}/{summary['fn']}  "
              f"precision={summary['precision']:.3f} recall={summary['recall']:.3f}")
        print(f"Средняя ошибка координат: {summary['mean_center_error_m']:.2f} м "
              f"(макс {summary['max_center_error_m']:.2f} м), "
              f"площади: {summary['mean_abs_area_error_pct']:.1f}%, "
              f"уровень опасности: {summary['threat_accuracy'] * 100:.0f}%")

    if args.csv and rows:
        save_metrics(rows, summary, args.output)
    if args.json:
        with open(os.path.join(args.output, "events.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(events, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(args.output, "summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump({"summary": summary,
                   "elapsed_s": round((datetime.now() - start).total_seconds(), 2),
                   "processed_images": processed},
                  handle, ensure_ascii=False, indent=2)

    print(f"[OK] Обработано изображений: {processed}; события: {len(events)}; "
          f"результаты: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
