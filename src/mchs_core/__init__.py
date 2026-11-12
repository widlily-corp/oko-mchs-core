"""
Centralized Core Infrastructure for the 'Oko MCHS' Emergency Monitoring UAV Complex.

Provides:
- MchsLogger: Unified subsystem-tagged logging with rotation and colored terminal output.
- log_event: Machine-readable JSONL event journal.
- install_crash_handler: Post-mortem crash dump generator with variable inspection.
- BlackBoxRecorder: Non-blocking onboard flight telemetry recorder (CSV/JSONL).
- MissionVideoReporter: Offline tactical video report compiler.
"""

from .blackbox import (
    CSV_HEADER,
    BlackBoxRecorder,
    TelemetryRecord,
)
from .logger import (
    MchsConsoleFormatter,
    MchsFileFormatter,
    MchsLogger,
    SubsystemFilter,
    get_default_log_dir,
    get_logger,
    install_crash_handler,
    log_event,
    reset_logging,
    set_log_dir,
)


def __getattr__(name: str):
    """Ленивый импорт видеорепортёра: бортовые ноды не должны требовать Pillow.

    ``video_reporter`` тянет Pillow ради кириллицы в видеорапорте и раньше
    импортировался сразу при ``import mchs_core``, из-за чего зрение, автопилот
    и arena_controller падали на чистой WSL без Pillow. Теперь модуль
    подгружается только при фактическом обращении к этим именам.
    """
    if name in ("MissionVideoReporter", "VideoReportConfig"):
        from . import video_reporter

        return getattr(video_reporter, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__version__ = "1.0.0"

__all__ = [
    "MchsLogger",
    "get_logger",
    "log_event",
    "install_crash_handler",
    "set_log_dir",
    "reset_logging",
    "get_default_log_dir",
    "SubsystemFilter",
    "MchsConsoleFormatter",
    "MchsFileFormatter",
    "BlackBoxRecorder",
    "TelemetryRecord",
    "CSV_HEADER",
    "MissionVideoReporter",
    "VideoReportConfig",
    "__version__",
]
