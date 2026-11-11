"""
MCHS Core Logging Subsystem.

Provides structured, thread-safe, colored console and rotating file logging,
JSONL event streaming, and unhandled exception / crash logging for the
"Oko MCHS" emergency monitoring complex.
"""

from __future__ import annotations

import inspect
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import platform
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Optional, Union


def get_default_log_dir() -> Path:
    """Return default repository logs directory (<repo_root>/logs)."""
    # logger.py is at <repo_root>/src/mchs_core/logger.py
    # parents[2] points to <repo_root>
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "logs"


class SubsystemFilter(logging.Filter):
    """Ensures each LogRecord has a subsystem attribute (defaults to CORE)."""

    def __init__(self, subsystem: str = "CORE") -> None:
        super().__init__()
        self.subsystem = subsystem.upper()

    def filter(self, record: logging.LogRecord) -> bool:
        if not hasattr(record, "subsystem"):
            record.subsystem = self.subsystem
        return True


class MchsConsoleFormatter(logging.Formatter):
    """
    Colorized console formatter:
    [YYYY-MM-DD HH:MM:SS.mmm] [SUBSYSTEM] [LEVEL] Message
    """

    COLOR_MAP = {
        logging.DEBUG: "\033[36m",      # Cyan
        logging.INFO: "\033[32m",       # Green
        logging.WARNING: "\033[33m",    # Yellow
        logging.ERROR: "\033[31m",      # Red
        logging.CRITICAL: "\033[1;31m", # Bold Red
    }
    RESET = "\033[0m"
    SUBSYSTEM_COLOR = "\033[1;34m"    # Bold Blue
    TIME_COLOR = "\033[90m"          # Bright Black (Gray)

    def __init__(self, use_color: bool = True) -> None:
        super().__init__()
        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        dt = datetime.fromtimestamp(record.created)
        timestamp = dt.strftime("%Y-%m-%d %H:%M:%S") + f".{int(record.msecs):03d}"
        subsystem = getattr(record, "subsystem", "CORE")
        levelname = record.levelname
        message = record.getMessage()

        if record.exc_info:
            if not record.exc_text:
                record.exc_text = self.formatException(record.exc_info)
            if record.exc_text:
                message = f"{message}\n{record.exc_text}"

        if self.use_color:
            level_color = self.COLOR_MAP.get(record.levelno, "")
            return (
                f"{self.TIME_COLOR}[{timestamp}]{self.RESET} "
                f"{self.SUBSYSTEM_COLOR}[{subsystem}]{self.RESET} "
                f"{level_color}[{levelname}]{self.RESET} "
                f"{message}"
            )
        return f"[{timestamp}] [{subsystem}] [{levelname}] {message}"


class MchsFileFormatter(logging.Formatter):
    """
    Standard file formatter without ANSI escapes:
    [YYYY-MM-DD HH:MM:SS.mmm] [SUBSYSTEM] [LEVEL] Message
    """

    def format(self, record: logging.LogRecord) -> str:
        dt = datetime.fromtimestamp(record.created)
        timestamp = dt.strftime("%Y-%m-%d %H:%M:%S") + f".{int(record.msecs):03d}"
        subsystem = getattr(record, "subsystem", "CORE")
        levelname = record.levelname
        message = record.getMessage()

        if record.exc_info:
            if not record.exc_text:
                record.exc_text = self.formatException(record.exc_info)
            if record.exc_text:
                message = f"{message}\n{record.exc_text}"

        return f"[{timestamp}] [{subsystem}] [{levelname}] {message}"


class MchsLogger:
    """
    Centralized logging manager for 'Oko MCHS'.

    Features:
    - Thread-safe singleton with get_logger() factory.
    - Automatic logs/ directory creation.
    - Console stream handler with ANSI colors.
    - Rotating file handler for logs/system.log (10MB, 5 backups, utf-8).
    - Rotating file handler for logs/<subsystem>.log (pilot.log, vision.log, etc.).
    - Machine-readable structured events logging to logs/events.jsonl.
    - Crash dump generation on unhandled exceptions.
    """

    _instance: Optional[MchsLogger] = None
    _lock = threading.RLock()

    def __new__(cls, log_dir: Optional[Union[str, Path]] = None, use_color: bool = True) -> MchsLogger:
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self, log_dir: Optional[Union[str, Path]] = None, use_color: bool = True) -> None:
        with self._lock:
            if getattr(self, "_initialized", False) and log_dir is None:
                return

            self._log_dir = Path(log_dir) if log_dir else get_default_log_dir()
            self._log_dir.mkdir(parents=True, exist_ok=True)
            self._use_color = use_color

            # Shared system file handler (system.log)
            self._system_log_path = self._log_dir / "system.log"
            self._events_jsonl_path = self._log_dir / "events.jsonl"
            self._crashes_dir = self._log_dir / "crashes"
            self._crashes_dir.mkdir(parents=True, exist_ok=True)

            self._file_formatter = MchsFileFormatter()
            self._console_formatter = MchsConsoleFormatter(use_color=self._use_color)

            # Reusable system handler
            self._system_handler: Optional[RotatingFileHandler] = None
            self._subsystem_handlers: Dict[str, RotatingFileHandler] = {}
            self._console_handler: Optional[logging.Handler] = None

            # JSONL lock
            self._events_lock = threading.Lock()

            # Registry of created loggers: name -> Logger
            self._configured_loggers: Dict[str, logging.Logger] = {}

            self._init_core_handlers()
            self._initialized = True

    def _init_core_handlers(self) -> None:
        """Initialize console and system.log handlers."""
        # Console handler
        console = logging.StreamHandler(sys.stdout)
        console.setFormatter(self._console_formatter)
        console.setLevel(logging.DEBUG)
        self._console_handler = console

        # System file handler
        system_handler = RotatingFileHandler(
            self._system_log_path,
            maxBytes=10 * 1024 * 1024,
            backupCount=5,
            encoding="utf-8",
        )
        system_handler.setFormatter(self._file_formatter)
        system_handler.setLevel(logging.DEBUG)
        self._system_handler = system_handler

    def _get_subsystem_handler(self, subsystem: str) -> RotatingFileHandler:
        """Get or create rotating handler for logs/<subsystem>.log."""
        subsystem_clean = subsystem.strip().lower()
        if subsystem_clean not in self._subsystem_handlers:
            sub_log_path = self._log_dir / f"{subsystem_clean}.log"
            handler = RotatingFileHandler(
                sub_log_path,
                maxBytes=10 * 1024 * 1024,
                backupCount=5,
                encoding="utf-8",
            )
            handler.setFormatter(self._file_formatter)
            handler.setLevel(logging.DEBUG)
            self._subsystem_handlers[subsystem_clean] = handler
        return self._subsystem_handlers[subsystem_clean]

    @property
    def log_dir(self) -> Path:
        return self._log_dir

    @classmethod
    def get_instance(cls, log_dir: Optional[Union[str, Path]] = None, use_color: bool = True) -> MchsLogger:
        """Access singleton instance, optionally reconfiguring log_dir."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = MchsLogger(log_dir=log_dir, use_color=use_color)
            elif log_dir is not None and cls._instance._log_dir != Path(log_dir):
                cls.reset()
                cls._instance = MchsLogger(log_dir=log_dir, use_color=use_color)
            return cls._instance

    @classmethod
    def reset(cls) -> None:
        """Reset singleton and flush/close existing handlers (useful for tests)."""
        with cls._lock:
            if cls._instance is not None:
                # Close handlers
                for h in list(cls._instance._subsystem_handlers.values()):
                    try:
                        h.close()
                    except Exception:
                        pass
                if cls._instance._system_handler:
                    try:
                        cls._instance._system_handler.close()
                    except Exception:
                        pass
                if cls._instance._console_handler:
                    try:
                        cls._instance._console_handler.close()
                    except Exception:
                        pass
                cls._instance._subsystem_handlers.clear()
                cls._instance._configured_loggers.clear()
                cls._instance = None

    @classmethod
    def get_logger(cls, name: str, subsystem: str = "CORE") -> logging.Logger:
        """
        Factory to obtain a configured logger for a given subsystem.

        Args:
            name: Logger name (e.g. 'mchs.vision.detector').
            subsystem: Subsystem identifier (e.g. 'PILOT', 'VISION', 'UI', 'CORE').
        """
        instance = cls.get_instance()
        return instance._configure_logger(name, subsystem)

    def _configure_logger(self, name: str, subsystem: str = "CORE") -> logging.Logger:
        subsystem_upper = subsystem.strip().upper()
        cache_key = f"{name}::{subsystem_upper}"

        with self._lock:
            if cache_key in self._configured_loggers:
                return self._configured_loggers[cache_key]

            logger = logging.getLogger(f"mchs.{subsystem_upper}.{name}")
            logger.setLevel(logging.DEBUG)
            logger.propagate = False

            # Clear existing handlers to prevent duplicate lines
            logger.handlers.clear()
            logger.filters.clear()

            # Subsystem filter
            sub_filter = SubsystemFilter(subsystem_upper)
            logger.addFilter(sub_filter)

            # Handlers
            if self._console_handler is not None:
                logger.addHandler(self._console_handler)
            if self._system_handler is not None:
                logger.addHandler(self._system_handler)

            sub_handler = self._get_subsystem_handler(subsystem_upper)
            logger.addHandler(sub_handler)

            self._configured_loggers[cache_key] = logger
            return logger

    @classmethod
    def log_event(
        cls,
        subsystem: str,
        event_type: str,
        data: Dict[str, Any],
        level: str = "INFO",
    ) -> Dict[str, Any]:
        """
        Append structured event to logs/events.jsonl and log via subsystem logger.

        Args:
            subsystem: Subsystem name ('PILOT', 'VISION', etc.).
            event_type: Identifier of the event ('TAKEOFF', 'HAZARD_DETECTED', etc.).
            data: Arbitrary dictionary with event payload.
            level: Logging level name ('INFO', 'WARNING', etc.).
        """
        instance = cls.get_instance()
        subsystem_upper = subsystem.strip().upper()
        now = datetime.now(timezone.utc)
        record = {
            "timestamp": now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
            "epoch_s": round(time.time(), 4),
            "subsystem": subsystem_upper,
            "event_type": event_type,
            "level": level.upper(),
            "data": data,
        }

        # Write to JSONL thread-safely
        line = json.dumps(record, ensure_ascii=False) + "\n"
        with instance._events_lock:
            with open(instance._events_jsonl_path, "a", encoding="utf-8") as f:
                f.write(line)
                f.flush()

        # Also send to subsystem logger
        logger = instance._configure_logger(f"events.{subsystem_upper.lower()}", subsystem_upper)
        lvl_no = getattr(logging, level.upper(), logging.INFO)
        logger.log(lvl_no, f"[{event_type}] {json.dumps(data, ensure_ascii=False)}")

        return record

    @classmethod
    def install_crash_handler(
        cls,
        subsystem: str = "CORE",
        chain: bool = True,
    ) -> Callable[..., Any]:
        """
        Install global exception hook to capture unhandled exceptions,
        writing full traceback, environment info, and frame variables snapshot to
        logs/crashes/crash_<subsystem>_<timestamp>.log.
        """
        instance = cls.get_instance()
        subsystem_clean = subsystem.strip().lower()
        crashes_dir = instance._log_dir / "crashes"
        crashes_dir.mkdir(parents=True, exist_ok=True)

        prev_excepthook = sys.excepthook

        def _safe_repr(val: Any) -> str:
            try:
                r = repr(val)
                if len(r) > 400:
                    return r[:397] + "..."
                return r
            except Exception:
                return "<unprintable object>"

        def crash_handler(exc_type: type, exc_value: BaseException, exc_tb: Any) -> None:
            now_dt = datetime.now()
            time_str = now_dt.strftime("%Y%m%d_%H%M%S_%f")[:19]
            crash_filename = f"crash_{subsystem_clean}_{time_str}.log"
            crash_file = crashes_dir / crash_filename

            import traceback

            tb_lines = traceback.format_exception(exc_type, exc_value, exc_tb)
            tb_str = "".join(tb_lines)

            # Inspect frames
            frame_dump = []
            curr_tb = exc_tb
            depth = 0
            while curr_tb is not None:
                frame = curr_tb.tb_frame
                code = frame.f_code
                frame_info = (
                    f"Frame {depth}: {code.co_name} in {code.co_filename}:{curr_tb.tb_lineno}\n"
                )
                local_vars = []
                for var_name, var_val in frame.f_locals.items():
                    if var_name.startswith("__") and var_name.endswith("__"):
                        continue
                    local_vars.append(f"    {var_name} = {_safe_repr(var_val)}")
                if local_vars:
                    frame_info += "\n".join(local_vars) + "\n"
                else:
                    frame_info += "    <no local variables>\n"
                frame_dump.append(frame_info)
                curr_tb = curr_tb.tb_next
                depth += 1

            crash_content = (
                "======================================================================\n"
                f"OKO MCHS EMERGENCY COMPLEX - CRASH REPORT [{subsystem.upper()}]\n"
                "======================================================================\n"
                f"Crash Time   : {now_dt.isoformat()}\n"
                f"Subsystem    : {subsystem.upper()}\n"
                f"Process ID   : {os.getpid()}\n"
                f"Thread ID    : {threading.get_ident()} ({threading.current_thread().name})\n"
                f"Python Ver   : {sys.version}\n"
                f"Platform     : {platform.platform()}\n"
                "----------------------------------------------------------------------\n"
                f"Exception    : {exc_type.__name__}: {exc_value}\n"
                "----------------------------------------------------------------------\n"
                "TRACEBACK:\n"
                f"{tb_str}\n"
                "----------------------------------------------------------------------\n"
                "FRAME VARIABLES SNAPSHOT:\n"
                + "\n".join(frame_dump)
                + "======================================================================\n"
            )

            try:
                with open(crash_file, "w", encoding="utf-8") as f:
                    f.write(crash_content)
                    f.flush()
            except Exception as write_err:
                sys.stderr.write(f"Failed to write crash dump: {write_err}\n")

            # Also log to system and subsystem logs
            try:
                logger = instance._configure_logger(f"crash.{subsystem_clean}", subsystem.upper())
                logger.critical(
                    f"CRASH in subsystem {subsystem.upper()}: {exc_type.__name__}: {exc_value}\n"
                    f"Saved dump to {crash_file}\n{tb_str}"
                )
            except Exception:
                pass

            if chain and prev_excepthook and prev_excepthook is not crash_handler:
                prev_excepthook(exc_type, exc_value, exc_tb)

        sys.excepthook = crash_handler

        # Also hook threading.excepthook if available (Python 3.8+)
        if hasattr(threading, "excepthook"):
            prev_thread_excepthook = threading.excepthook

            def thread_crash_handler(args: Any) -> None:
                crash_handler(args.exc_type, args.exc_value, args.exc_traceback)
                if chain and prev_thread_excepthook and prev_thread_excepthook is not thread_crash_handler:
                    prev_thread_excepthook(args)

            threading.excepthook = thread_crash_handler

        return crash_handler


# Module-level convenience functions
def get_logger(name: str, subsystem: str = "CORE") -> logging.Logger:
    """Convenience alias for MchsLogger.get_logger()."""
    return MchsLogger.get_logger(name=name, subsystem=subsystem)


def log_event(
    subsystem: str,
    event_type: str,
    data: Dict[str, Any],
    level: str = "INFO",
) -> Dict[str, Any]:
    """Convenience alias for MchsLogger.log_event()."""
    return MchsLogger.log_event(subsystem=subsystem, event_type=event_type, data=data, level=level)


def install_crash_handler(subsystem: str = "CORE", chain: bool = True) -> Callable[..., Any]:
    """Convenience alias for MchsLogger.install_crash_handler()."""
    return MchsLogger.install_crash_handler(subsystem=subsystem, chain=chain)


def set_log_dir(path: Union[str, Path]) -> None:
    """Reconfigure default logging directory."""
    MchsLogger.get_instance(log_dir=path)


def reset_logging() -> None:
    """Reset logger state (used between tests)."""
    MchsLogger.reset()
