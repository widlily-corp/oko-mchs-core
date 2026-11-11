# Advanced Emergency Features Implementation Plan: Fire Area, Human Search & Video Reports

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement calibrated fire area estimation from aerial photos, autonomous victim/human search with coordinate-tagged photographic evidence, 3D human model integration in Gazebo simulation, and real-time UI highlighting with automated tactical video report generation.

**Architecture:** Distributed modular architecture adhering to Senior ROS 2 / CV standards: `mchs_vision` provides calibrated ray-cast area calculation and multi-stage victim detection; `mchs_simulation` incorporates standard Gazebo 3D human models with dynamic arena relocation; `mchs_core` provides an offline-safe video report generator (`VideoWriter`); and `mchs_ui` (customtkinter) displays pulsating radar targets, photo cards, and video report exports.

**Tech Stack:** ROS 2 Humble, Python 3.10+, OpenCV 4 (cv2), Gazebo 11 (SDF 1.5/1.6, ODE physics), customtkinter / Tkinter, NumPy, PIL.

**Spec:** User Requirements Phase 2 ("Площадь возгорания по фото, Поиск человека и фото с координатами, Интеграция 3D моделей человека в Gazebo, Подсветка в реальном времени и видеоотчёт").

---

## Global Constraints

- **Zero Regression:** All existing 71+ unit and regression tests must remain 100% green at all times.
- **Offline Self-Sufficiency:** The system must function completely without internet access during jury defense (include offline fallback 3D models and local CV algorithms without cloud API dependencies).
- **Zero GC-Alloc / Low CPU Overhead:** Computer vision pipelines must sustain $\ge 25$ FPS on CPU (Jetson Orin Nano target budget $\le 15$W).
- **Single Source of Truth:** Coordinates and hazard configurations remain strictly synchronized across `config/hazards.json`, Gazebo world generator, and ARM UI.
- **Aviation-Grade Logging:** All new events (`HUMAN_DETECTED`, `FIRE_AREA_CALCULATED`, `VIDEO_REPORT_GENERATED`) must pass through `mchs_core.log_event` with structured payloads.

---

## Review Focus (Failure Modes & Edge Cases)

1. **Altitude & Perspective Skew in Area Calculation:** At different altitudes (e.g. 15 m vs 25 m vs 40 m) or camera tilts (45° pitch vs 90° nadir), simple pixel counting yields orders-of-magnitude errors. *Mitigation:* Explicit ground-projected polygon area using camera intrinsics ($FOV = 80^\circ$, $1280 \times 720$) and drone altitude $Z$ via `GroundProjector`.
2. **Missing Internet Access for 3D Models in Gazebo:** If Gazebo tries to download models from Fuel during defense without internet, gzserver hangs for 30 seconds or crashes. *Mitigation:* Local offline storage in `src/mchs_simulation/models/person_standing` with self-contained meshes/materials + procedural fallback.
3. **Low Contrast / Small Human Silhouette at Altitude:** A human seen from 22 m is only ~20–50 pixels. Pure HOG on whole $1280 \times 720$ frame can be slow (~5 FPS) or miss lying figures. *Mitigation:* Hybrid pipeline — color saliency ROI filtering (emergency vest / thermal signature / contrast) + targeted HOG verification on candidate ROIs.
4. **Race Condition in Incident Photo Saving:** Rapid successive detections could overwrite photo files or cause file lock errors on Windows UNC share. *Mitigation:* Nanosecond timestamping + unique incident IDs + atomic write to temp file then rename.
5. **Video Encoding Codec Compatibility:** OpenCV `VideoWriter` with `mp4v` or `avc1` may fail if ffmpeg / H.264 libraries are missing in the OS. *Mitigation:* Fallback codec cascade (`mp4v` -> `XVID` -> `MJPG` with `.avi` fallback) ensuring zero crash guarantee.

---

## Tasks

### Task 1: Calibrated Fire Area Estimation from Aerial Photo (`mchs_vision.fire_area`)
**Lead Subagent:** Vision Lead (Рома)

**Files:**
- Create: `src/mchs_vision/mchs_vision/fire_area.py`
- Create: `scripts/estimate_fire_area.py`
- Create: `src/mchs_vision/tests/test_fire_area.py`
- Modify: `src/mchs_vision/mchs_vision/__init__.py`
- Modify: `src/mchs_vision/mchs_vision/pipeline.py`

**Steps:**
- [ ] Write failing unit test `src/mchs_vision/tests/test_fire_area.py` covering:
  - Exact area calculation for known synthetic geometric shapes at $Z = 22.0$ m and $Z = 35.0$ m.
  - Flame vs. smoke segmentation separation.
  - Tactical annotation rendering (contour outline, area text, tactical 10 m / 50 m scale ruler).
  - CLI execution with `--input`, `--altitude`, `--output`, `--json`.
- [ ] Run test to verify failure (`python3 -m pytest src/mchs_vision/tests/test_fire_area.py`).
- [ ] Implement `src/mchs_vision/mchs_vision/fire_area.py`:
  - `FireAreaEstimator`: Ground Sampling Distance ($GSD = 2 \cdot Z \cdot \tan(FOV/2) / W$), perspective projection, contour moments, $m^2$ and hectare conversion.
  - Flame and smoke masks extraction using calibrated HSV boundaries.
  - Overlay renderer with translucent fill, bounding box, centroid marker, and visual scale bar.
- [ ] Implement standalone CLI tool `scripts/estimate_fire_area.py` for operator & batch processing.
- [ ] Expose `FireAreaEstimator` in `mchs_vision/__init__.py` and hook into `pipeline.py` for live HUD area reporting.
- [ ] Run tests and verify 100% pass (`python3 -m pytest src/mchs_vision/tests/test_fire_area.py`).
- [ ] Commit: `git commit -m "feat(vision): add calibrated fire area estimation from aerial photo with scale bar"`

---

### Task 2: 3D Human Models Integration in Gazebo Simulation Arena
**Lead Subagent:** Simulation & Arena Lead (Руслан)

**Files:**
- Create: `src/mchs_simulation/scripts/download_human_models.py`
- Create: `src/mchs_simulation/models/person_standing/model.config`
- Create: `src/mchs_simulation/models/person_standing/model.sdf`
- Modify: `src/mchs_simulation/scripts/generate_arena_world.py`
- Modify: `src/mchs_simulation/scripts/arena_controller.py`
- Modify: `config/hazards.json`
- Create: `tests/test_arena_human_models.py`

**Steps:**
- [ ] Write failing test `tests/test_arena_human_models.py` verifying:
  - Model directory structure and SDF syntax validity.
  - `generate_arena_world.py` produces world containing human entity when configured in `hazards.json`.
  - `arena_controller.py` recognizes `--hazard human` / `--hazard victim`.
- [ ] Run test to verify failure.
- [ ] Implement `download_human_models.py` with offline fallback:
  - Downloads official `person_standing` and `casual_female` meshes from Gazebo models repo.
  - If network is unreachable, automatically creates self-contained geometric SDF model (anatomical proportions, high-vis rescue vest #FF5722) so simulation never breaks offline.
- [ ] Update `config/hazards.json` to include `HUMAN` / `VICTIM` target:
  - `"HUMAN": {"title": "Пострадавший (человек)", "short": "ЧЕЛОВЕК", "threat": "CRITICAL", "color": "#E040FB", "x": 15.0, "y": -22.0, "model": "person_standing", "detect_classes": ["VICTIM"]}`.
- [ ] Update `generate_arena_world.py` to spawn the human model into the arena at specified coordinates.
- [ ] Update `arena_controller.py` to support dynamic relocation of the human model via ROS 2 `/gazebo/set_entity_state` or `gz service`.
- [ ] Run tests and verify 100% pass (`python3 -m pytest tests/test_arena_human_models.py`).
- [ ] Commit: `git commit -m "feat(simulation): integrate 3D human models into Gazebo arena and arena controller"`

---

### Task 3: Autonomous Human Detection Pipeline with Coordinates & Snapshot Evidence
**Lead Subagent:** Vision Lead (Рома)

**Files:**
- Modify: `src/mchs_vision/mchs_vision/config.py`
- Modify: `src/mchs_vision/mchs_vision/detector.py`
- Modify: `src/mchs_vision/mchs_vision/events.py`
- Modify: `src/mchs_vision/scripts/mchs_vision_node.py`
- Create: `src/mchs_vision/tests/test_victim_detection.py`

**Steps:**
- [ ] Write failing test `src/mchs_vision/tests/test_victim_detection.py` verifying:
  - Detection of human silhouette / high-vis clothing in synthetic and Gazebo frames.
  - Coordinate triangulation via `GroundProjector` yielding accurate $(X, Y)$ and GPS (lat, lon).
  - Snapshot evidence creation in `reports/incidents/VICTIM_<id>_<time>.jpg`.
  - Event payload containing `photo_path`, `target_geo`, and `threat_level: CRITICAL`.
- [ ] Run test to verify failure.
- [ ] Modify `mchs_vision/config.py` to add `VICTIM` class specifications (high-vis emergency orange/cyan + aspect ratio filters).
- [ ] Modify `mchs_vision/detector.py` with multi-cue human detector (HSV color saliency + contour aspect ratio $[1.5, 4.0]$ + candidate ROI verification).
- [ ] Modify `mchs_vision/events.py` to record victim snapshot paths and WGS-84 coordinates in `IncidentRegistry`.
- [ ] Modify `mchs_vision_node.py`:
  - On `VICTIM` confirmation, immediately save high-res cropped and full-frame snapshot into `reports/incidents/`.
  - Transmit `photo_path` in `/mchs/detection_event` and update `/state.json`.
- [ ] Run tests and verify 100% pass (`python3 -m pytest src/mchs_vision/tests/test_victim_detection.py`).
- [ ] Commit: `git commit -m "feat(vision): add autonomous human detection with coordinate-tagged photographic evidence"`

---

### Task 4: Real-Time UI Highlighting & Mission Video Report Generator
**Lead Subagent:** UI & Autonomy Lead (Артём)

**Files:**
- Create: `src/mchs_core/video_reporter.py`
- Create: `src/mchs_ui/tests/test_video_reporter.py`
- Modify: `src/mchs_ui/app.py`
- Modify: `src/mchs_core/__init__.py`

**Steps:**
- [ ] Write failing test `src/mchs_ui/tests/test_video_reporter.py` verifying:
  - Video clip generation (`compile_mission_video_report`) from images and telemetry log.
  - Codec fallback cascade (`mp4v` -> `XVID` -> `MJPG`).
  - Overlay title card, telemetry graphs, and incident callouts.
- [ ] Run test to verify failure.
- [ ] Implement `src/mchs_core/video_reporter.py`:
  - Mission video report compiler: concatenates flight camera frames, adds title slide ("ОКО МЧС — РАПОРТ ПАТРУЛИРОВАНИЯ"), incident highlight cards with photo snapshots, coordinates, and fire area, plus summary statistics outro.
- [ ] Update `src/mchs_ui/app.py`:
  - Real-time pulsating radar highlight (animated concentric ripples around detected victim / active hazard).
  - Prominent alert banner: "🚨 ОБНАРУЖЕН ЧЕЛОВЕК! КООРДИНАТЫ [X, Y] ПЕРЕДАНЫ В ЦУКС".
  - Incident Photo Card in GCS UI: displays snapshot thumbnail with click-to-enlarge.
  - "🎬 ВИДЕОРАПОРТ" button: triggers tactical video generation and opens output MP4 in default player.
- [ ] Run tests and verify 100% pass (`python3 -m pytest src/mchs_ui/tests/test_video_reporter.py`).
- [ ] Commit: `git commit -m "feat(ui): add real-time pulsating target highlighting and tactical video report compilation"`

---

### Task 5: End-to-End System Integration, Full Regression Suite & Live Defense Verification
**Lead Subagent:** Principal Architect & Reviewer (Руслан)

**Files:**
- Modify: `scripts/run_all.sh` & `run_all.bat` (update environment variables and model paths).
- Modify: `docs/LIVE_SCAN_DEMO.md` (add human search scenario runbook).
- Modify: `docs/BIG_ARENA.md` (document victim hazard entity and coordinates).
- Test: Full regression suite (`80+ tests`).

**Steps:**
- [ ] Update `scripts/run_all.sh` to ensure `GAZEBO_MODEL_PATH` exports `src/mchs_simulation/models`.
- [ ] Run full project test suite in WSL:
  `PYTHONPATH=src:src/mchs_vision:src/mchs_core:src/mchs_control:src/mchs_ui:scripts python3 -m pytest tests src/mchs_vision/tests src/mchs_core/tests src/mchs_control/test src/mchs_ui/tests -q`
  Confirm all tests green.
- [ ] Perform live verification dry run:
  1. Generate updated world: `python3 src/mchs_simulation/scripts/generate_arena_world.py`.
  2. Test `estimate_fire_area.py` on a sample test frame, verify annotated JPEG and area JSON output.
  3. Relocate human using `arena_controller.py --hazard human --x 20.0 --y -15.0`.
  4. Verify incident protocol JSON contains both fire area metrics and victim photo path.
- [ ] Update documentation (`docs/LIVE_SCAN_DEMO.md`) with the new 4-stage jury presentation flow.
- [ ] Final commit: `git commit -m "chore(release): integrate phase 2 advanced disaster monitoring features and verify green tests"`
