"""Exercise production callback bodies without installing ROS or running executors."""
import ast
import math
import os
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def callback(path, name, **globals_):
    tree = ast.parse((ROOT / path).read_text(encoding="utf-8"))
    method = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(math=math, time=time, os=os, datetime=datetime, **globals_)
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[
        ast.alias(name="annotations")], level=0), method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    return namespace[name]


@pytest.mark.parametrize("field,value", [
    ("x", float("nan")), ("y", float("inf")), ("qx", float("-inf")),
    ("w", 0.0),
])
@pytest.mark.parametrize("route", [True, False])
def test_invalid_odometry_keeps_last_valid_state(field, value, route):
    pos = NS(x=1.0, y=2.0, z=22.0)
    ori = NS(x=0.0, y=0.0, z=0.0, w=1.0)
    if field == "qx":
        ori.x = value
    elif field == "w":
        ori.w = value
    else:
        setattr(pos, field, value)
    message = NS(pose=NS(pose=NS(position=pos, orientation=ori)),
                 twist=NS(twist=NS(linear=NS(x=0.0, y=0.0, z=0.0))))
    pilot = NS(have_odom=True, last_odom_time=123.0, odom_x=9.0,
               odom_y=8.0, est_x=9.0, est_y=8.0, odom_yaw=0.0,
               odom_position=(9.0, 8.0), odom_altitude=22.0,
               odom_vx=0.0, odom_vy=0.0, odom_vz=0.0, odom_yaw_deg=0.0,
               _warn=lambda text: None,
               get_logger=lambda: NS(warning=lambda text: None))
    before = vars(pilot).copy()
    path = "src/mchs_control/scripts/mchs_pilot_" + ("route.py" if route else "node.py")
    callback(path, "_on_odom" if route else "on_odom")(pilot, message)
    assert vars(pilot) == before


def test_failed_victim_photo_does_not_publish_path_or_increment_count(tmp_path):
    node = NS(save_incident_photos=True, _last_photo_time=0.0,
              _photo_count=7, _last_photo="previous.jpg", _pose=None,
              incidents_dir=str(tmp_path), logger=None)
    save = callback("src/mchs_vision/scripts/mchs_vision_node.py",
                    "_save_incident_photo", save_victim_evidence_snapshot=lambda **kw: "")
    assert save(node, {"hazard_type": "VICTIM"}, np.zeros((100, 100, 3))) == ""
    assert node._photo_count == 7
    assert node._last_photo == "previous.jpg"
    assert node._last_photo_time == 0.0

def test_streamer_socket_timeout_is_configured():
    import socket
    import threading
    from mchs_vision.web_stream import LatestFrameStreamer

    streamer = LatestFrameStreamer(host="127.0.0.1", port=0, start_server=True)
    try:
        assert streamer._server is not None
        assert streamer._server.daemon_threads is True
        client = socket.create_connection(streamer._server.server_address, timeout=2.0)
        client.settimeout(2.0)
        client.sendall(b"GET /snapshot.jpg HTTP/1.0\r\n\r\n")
        chunks = []
        try:
            while True:
                data = client.recv(65536)
                if not data:
                    break
                chunks.append(data)
        except socket.timeout:
            pass
        client.close()
        assert b"503" in b"".join(chunks)
    finally:
        streamer.stop()
