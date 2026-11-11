"""
MCHS BlackBox Flight Recorder.

Provides real-time, non-blocking, thread-safe onboard telemetry and event
logging to telemetry_blackbox.csv and telemetry_blackbox.jsonl for post-flight
incident analysis, flight playback, and emergency investigation.
"""

from __future__ import annotations

import atexit
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Union

from .logger import get_default_log_dir


CSV_HEADER = [
    "timestamp",
    "epoch_s",
    "x",
    "y",
    "z",
    "yaw_deg",
    "vx",
    "vy",
    "vz",
    "battery_pct",
    "state",
    "hazard_alert",
    "message",
]


@dataclass
class TelemetryRecord:
    """Represents a single telemetry snapshot from the flight controller / sensors."""

    timestamp: str
    epoch_s: float
    x: float
    y: float
    z: float
    yaw_deg: float
    vx: float
    vy: float
    vz: float
    battery_pct: float
    state: str
    hazard_alert: bool
    message: str
    extra: Optional[Dict[str, Any]] = None

    def to_csv_row(self) -> List[str]:
        return [
            self.timestamp,
            f"{self.epoch_s:.4f}",
            f"{self.x:.3f}",
            f"{self.y:.3f}",
            f"{self.z:.3f}",
            f"{self.yaw_deg:.2f}",
            f"{self.vx:.3f}",
            f"{self.vy:.3f}",
            f"{self.vz:.3f}",
            f"{self.battery_pct:.1f}",
            str(self.state),
            str(bool(self.hazard_alert)),
            str(self.message),
        ]

    def to_dict(self) -> Dict[str, Any]:
        data = {
            "timestamp": self.timestamp,
            "epoch_s": self.epoch_s,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw_deg": self.yaw_deg,
            "vx": self.vx,
            "vy": self.vy,
            "vz": self.vz,
            "battery_pct": self.battery_pct,
            "state": self.state,
            "hazard_alert": bool(self.hazard_alert),
            "message": self.message,
        }
        if self.extra:
            data["extra"] = self.extra
        return data


class BlackBoxRecorder:
    """
    Onboard telemetry blackbox flight recorder.

    Records flight telemetry and significant mission events to CSV and JSONL
    in a dedicated background thread without blocking high-rate control loops.
    """

    def __init__(
        self,
        log_dir: Optional[Union[str, Path]] = None,
        flush_interval_s: float = 0.2,
        max_queue_size: int = 50000,
        csv_filename: str = "telemetry_blackbox.csv",
        jsonl_filename: str = "telemetry_blackbox.jsonl",
    ) -> None:
        self._log_dir = Path(log_dir) if log_dir else get_default_log_dir()
        self._log_dir.mkdir(parents=True, exist_ok=True)

        self._csv_path = self._log_dir / csv_filename
        self._jsonl_path = self._log_dir / jsonl_filename

        self._flush_interval_s = max(0.01, flush_interval_s)
        self._queue: queue.Queue[Optional[TelemetryRecord]] = queue.Queue(maxsize=max_queue_size)

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._closed = False
        self._records_written = 0

        # File handles for worker thread
        self._csv_file = None
        self._csv_writer = None
        self._jsonl_file = None

        self._init_files()

        # Start background writer thread
        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            name="MchsBlackBoxWorker",
            daemon=True,
        )
        self._worker_thread.start()

        # Clean shutdown on exit
        atexit.register(self._cleanup_atexit)

    def _init_files(self) -> None:
        """Initialize files and write CSV header if new or empty."""
        need_header = not self._csv_path.exists() or self._csv_path.stat().st_size == 0

        self._csv_file = open(self._csv_path, "a", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        if need_header:
            self._csv_writer.writerow(CSV_HEADER)
            self._csv_file.flush()

        self._jsonl_file = open(self._jsonl_path, "a", encoding="utf-8")

    @property
    def csv_path(self) -> Path:
        return self._csv_path

    @property
    def jsonl_path(self) -> Path:
        return self._jsonl_path

    @property
    def records_written(self) -> int:
        return self._records_written

    def record_telemetry(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw_deg: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        battery_pct: float = 100.0,
        state: str = "IDLE",
        hazard_alert: bool = False,
        message: str = "",
        timestamp: Optional[str] = None,
        epoch_s: Optional[float] = None,
        extra: Optional[Dict[str, Any]] = None,
        **extra_kwargs,
    ) -> TelemetryRecord:
        """
        Record a telemetry frame asynchronously without blocking the caller.

        Returns:
            The created TelemetryRecord.
        """
        if self._closed:
            raise RuntimeError("Cannot record telemetry to a closed BlackBoxRecorder")

        curr_epoch = round(time.time(), 4) if epoch_s is None else float(epoch_s)
        if timestamp is None:
            now_dt = datetime.fromtimestamp(curr_epoch, tz=timezone.utc)
            curr_timestamp = now_dt.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        else:
            curr_timestamp = str(timestamp)

        combined_extra = {}
        if extra:
            combined_extra.update(extra)
        if extra_kwargs:
            combined_extra.update(extra_kwargs)

        record = TelemetryRecord(
            timestamp=curr_timestamp,
            epoch_s=curr_epoch,
            x=float(x),
            y=float(y),
            z=float(z),
            yaw_deg=float(yaw_deg),
            vx=float(vx),
            vy=float(vy),
            vz=float(vz),
            battery_pct=float(battery_pct),
            state=str(state),
            hazard_alert=bool(hazard_alert),
            message=str(message),
            extra=combined_extra if combined_extra else None,
        )

        try:
            self._queue.put_nowait(record)
        except queue.Full:
            # Overload protection: do not crash flight loop, drop oldest or report
            try:
                _ = self._queue.get_nowait()
                self._queue.task_done()
                self._queue.put_nowait(record)
            except Exception:
                pass

        return record

    def record_event(
        self,
        event_type: str,
        message: str = "",
        state: str = "EVENT",
        hazard_alert: bool = False,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw_deg: float = 0.0,
        battery_pct: float = 100.0,
        extra: Optional[Dict[str, Any]] = None,
        **extra_kwargs,
    ) -> TelemetryRecord:
        """
        Record an operational or safety event into the telemetry stream.
        """
        combined_msg = f"[{event_type}] {message}".strip()
        merged_extra = {"event_type": event_type}
        if extra:
            merged_extra.update(extra)
        if extra_kwargs:
            merged_extra.update(extra_kwargs)

        return self.record_telemetry(
            x=x,
            y=y,
            z=z,
            yaw_deg=yaw_deg,
            battery_pct=battery_pct,
            state=state,
            hazard_alert=hazard_alert,
            message=combined_msg,
            extra=merged_extra,
        )

    def _worker_loop(self) -> None:
        """Background worker consuming queue and flushing batches to disk."""
        batch: List[TelemetryRecord] = []

        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                # Wait for next item
                item = self._queue.get(timeout=self._flush_interval_s)
                if item is not None:
                    batch.append(item)

                # Drain remaining immediately available items
                while len(batch) < 1000:
                    try:
                        next_item = self._queue.get_nowait()
                        if next_item is not None:
                            batch.append(next_item)
                        else:
                            self._queue.task_done()
                    except queue.Empty:
                        break

            except queue.Empty:
                pass

            if batch:
                self._flush_batch(batch)
                for _ in range(len(batch)):
                    self._queue.task_done()
                batch.clear()

    def _flush_batch(self, batch: List[TelemetryRecord]) -> None:
        """Write batch of records to CSV and JSONL files."""
        if not batch:
            return

        with self._lock:
            try:
                if self._csv_file and not self._csv_file.closed:
                    for rec in batch:
                        self._csv_writer.writerow(rec.to_csv_row())
                    self._csv_file.flush()

                if self._jsonl_file and not self._jsonl_file.closed:
                    for rec in batch:
                        line = json.dumps(rec.to_dict(), ensure_ascii=False) + "\n"
                        self._jsonl_file.write(line)
                    self._jsonl_file.flush()

                self._records_written += len(batch)
            except Exception:
                pass

    def flush(self, timeout: float = 3.0) -> None:
        """
        Wait for all queued telemetry records to be written to disk and flushed.
        """
        if self._closed:
            return

        # Wait for queue items to be consumed by worker
        try:
            self._queue.join()
        except Exception:
            pass

        with self._lock:
            if self._csv_file and not self._csv_file.closed:
                try:
                    self._csv_file.flush()
                    os.fsync(self._csv_file.fileno())
                except Exception:
                    pass

            if self._jsonl_file and not self._jsonl_file.closed:
                try:
                    self._jsonl_file.flush()
                    os.fsync(self._jsonl_file.fileno())
                except Exception:
                    pass

    def close(self) -> None:
        """
        Gracefully stop worker thread and close file descriptors.
        """
        with self._lock:
            if self._closed:
                return
            self._closed = True

        self._stop_event.set()
        # Wake up worker
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass

        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=3.0)

        # Final flush & close
        with self._lock:
            try:
                if self._csv_file and not self._csv_file.closed:
                    self._csv_file.flush()
                    self._csv_file.close()
            except Exception:
                pass

            try:
                if self._jsonl_file and not self._jsonl_file.closed:
                    self._jsonl_file.flush()
                    self._jsonl_file.close()
            except Exception:
                pass

    def _cleanup_atexit(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self) -> BlackBoxRecorder:
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
