#!/usr/bin/env python3
"""Сгенерировать демонстрационное видео полёта БПЛА над полигоном ЧС.

Дрон проходит по маршруту над пожаром, разливом нефти, зоной затопления
и задымлением. Каждый кадр обрабатывается тем же :class:`VisionPipeline`,
что и в бортовой ноде: HUD, рамки, координаты, площадь, уровень угрозы.

Пример:
    python3 scripts/make_demo_video.py --output /tmp/oko_mchs_vision_demo
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timedelta
from typing import List, Tuple

import cv2

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from mchs_vision.config import CameraConfig, DetectorConfig
from mchs_vision.pipeline import VisionPipeline
from mchs_vision.simulated_scene import (FRAME_HEIGHT, FRAME_WIDTH,
                                         SyntheticArena)

# (время, x, y) в пикселях ортофотополигона. Одинаковые точки = зависание.
KEYFRAMES: List[Tuple[float, int, int]] = [
    (0.0, 400, 200),
    (2.0, 420, 320),
    (4.5, 420, 320),
    (7.0, 600, 450),
    (9.5, 790, 620),
    (11.5, 790, 620),
    (14.0, 1050, 520),
    (16.5, 1350, 380),
    (19.0, 1350, 380),
    (22.0, 900, 700),
    (25.0, 480, 1030),
    (27.5, 480, 1030),
    (30.0, 850, 1060),
    (33.0, 1300, 980),
    (35.5, 1300, 980),
]


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def position_at(time_s: float) -> Tuple[int, int]:
    if time_s <= KEYFRAMES[0][0]:
        return KEYFRAMES[0][1], KEYFRAMES[0][2]
    for (t0, x0, y0), (t1, x1, y1) in zip(KEYFRAMES, KEYFRAMES[1:]):
        if t0 <= time_s <= t1:
            span = max(1e-6, t1 - t0)
            ratio = smoothstep((time_s - t0) / span)
            return int(round(x0 + (x1 - x0) * ratio)), \
                int(round(y0 + (y1 - y0) * ratio))
    return KEYFRAMES[-1][1], KEYFRAMES[-1][2]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", default="/tmp/oko_mchs_vision_demo",
                        help="каталог результатов")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--scale", type=float, default=1.5,
                        help="масштаб выходного видео")
    parser.add_argument("--min-hits", type=int, default=2)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    arena = SyntheticArena()
    arena.render()

    camera = CameraConfig(width=FRAME_WIDTH, height=FRAME_HEIGHT,
                          hfov_deg=80.0, pitch_down_deg=90.0)
    pipeline = VisionPipeline(
        camera=camera,
        detector_config=DetectorConfig(track_min_hits=args.min_hits),
        title="MCHS OPTICAL RECON // SYNTHETIC ARENA",
    )

    duration = KEYFRAMES[-1][0]
    total_frames = int(round(duration * args.fps))
    if args.limit:
        total_frames = min(total_frames, args.limit)

    out_w = int(FRAME_WIDTH * args.scale)
    out_h = int(FRAME_HEIGHT * args.scale)
    video_path = os.path.join(args.output, "oko_mchs_vision_demo.mp4")
    writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (out_w, out_h))
    if not writer.isOpened():
        print("[ERROR] не удалось открыть VideoWriter", file=sys.stderr)
        return 1

    start = datetime.now().astimezone()
    events = []
    poster_written = False

    for index in range(total_frames):
        time_s = index / float(args.fps)
        center = position_at(time_s)
        frame = arena.crop(center)
        pose = arena.pose_for(center)
        timestamp = start + timedelta(seconds=time_s)

        result = pipeline.process(frame, pose, timestamp, draw=True)
        for event in result.events:
            event = dict(event)
            event["video_time_s"] = round(time_s, 2)
            event["frame_index"] = index
            events.append(event)

        output = result.annotated
        if args.scale != 1.0:
            output = cv2.resize(output, (out_w, out_h),
                                interpolation=cv2.INTER_CUBIC)
        writer.write(output)

        if not poster_written and result.records and time_s >= 2.2:
            cv2.imwrite(os.path.join(args.output, "poster_demo.png"), output)
            poster_written = True

        if index % 60 == 0:
            print(f"  кадр {index}/{total_frames} ({time_s:.1f} c), "
                  f"событий: {len(events)}, треков: {len(result.records)}")

    writer.release()

    with open(os.path.join(args.output, "events.json"), "w",
              encoding="utf-8") as handle:
        json.dump(events, handle, ensure_ascii=False, indent=2)

    summary = {
        "video": video_path,
        "duration_s": duration,
        "fps": args.fps,
        "frames": total_frames,
        "events": len(events),
        "resolution": [out_w, out_h],
    }
    with open(os.path.join(args.output, "video_summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)

    print(f"[OK] Видео: {video_path} ({total_frames} кадров, {duration:.1f} c)")
    print(f"[OK] Событий: {len(events)}; JSON: events.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
