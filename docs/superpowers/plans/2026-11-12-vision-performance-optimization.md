# Vision Performance Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Accelerate the onboard vision pipeline by eliminating full-frame Laplacian computation and synchronous JPEG encoding, doubling CPU frame rate without affecting hazard detection accuracy.

**Architecture:** Refactor `HazardDetector` to calculate texture metrics strictly inside candidate ROIs, and update `LatestFrameStreamer` to encode JPEGs on demand with monotonic frame caching.

**Tech Stack:** Python 3.10, OpenCV (cv2), NumPy, ROS 2 Humble, pytest.

**Spec:** `C:\Users\Widlily\.gemini\antigravity\brain\b8962da0-bbe8-480c-a0f4-3e35d019b1e5\VISION_PERFORMANCE_OPTIMIZATION_SPEC.md`

## Global Constraints
- Python 3.10 compatible, strict type annotations.
- Zero regression: all 121 existing unit/integration tests must pass.
- Mathematical equivalence: texture std inside candidate ROI must match within 1e-4 tolerance.
- Full resolution preserved for HUMAN/VICTIM reflective stripe inspection.

## Tasks

- [x] **Task 1: Lazy ROI Laplacian in `HazardDetector`**
  - **Files:** `src/mchs_vision/mchs_vision/detector.py`, `src/mchs_vision/tests/test_detector_performance.py`
  - **Details:**
    - Replace full-frame GaussianBlur and Laplacian in `HazardDetector.detect()` with lazy ROI helper `_roi_texture_std(gray, bbox, mask_roi, ksize)`.
    - Update `_detect_class` and `_merge_candidates` to call `_roi_texture_std` only when candidate passes geometric filter.
    - Add tests asserting identical detection results and benchmark timing.
  - **Verification:** Run `pytest src/mchs_vision/tests/test_detector.py src/mchs_vision/tests/test_detector_performance.py`.

- [x] **Task 2: On-Demand Cached JPEG Encoding in `LatestFrameStreamer`**
  - **Files:** `src/mchs_vision/mchs_vision/web_stream.py`, `src/mchs_vision/tests/test_web_stream_performance.py`
  - **Details:**
    - In `update(frame_bgr)`, store raw frame pointer and increment `_frame_id` under lock; do NOT call `cv2.imencode`.
    - In `snapshot()`, if cached `_cached_frame_id != _frame_id`, encode JPEG and cache bytes.
    - Ensure thread-safety for multi-threaded HTTP server.
  - **Verification:** Run `pytest src/mchs_vision/tests/test_web_stream_performance.py`.

- [x] **Task 3: Full Test Suite Verification and FPS Benchmark**
  - **Files:** `scripts/benchmark_vision_fps.py`
  - **Details:**
    - Create lightweight benchmark script measuring FPS on test frames.
    - Run all 131 tests in WSL (100% green).
  - **Verification:** Run `PYTHONPATH=src:... python3 -m pytest tests src/mchs_vision/tests src/mchs_core/tests src/mchs_control/test src/mchs_ui/tests -v`.
