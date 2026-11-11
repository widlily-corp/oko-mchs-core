"""
Centralized Core Infrastructure for the 'Oko MCHS' Emergency Monitoring UAV Complex.

Provides:
- MchsLogger: Unified subsystem-tagged logging with rotation and colored terminal output.
- log_event: Machine-readable JSONL event journal.
- install_crash_handler: Post-mortem crash dump generator with variable inspection.
- BlackBoxRecorder: Non-blocking onboard flight telemetry recorder (CSV/JSONL).
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
    "__version__",
]
