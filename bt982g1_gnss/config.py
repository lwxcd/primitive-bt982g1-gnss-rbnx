# SPDX-License-Identifier: MulanPSL-2.0
"""Configuration parsing and serial port resolution for the BT-982G1 primitive."""
from __future__ import annotations

import math
from pathlib import Path
import re

TOPICS = {
    "fix": "/gps/fix",
    "rtk_status": "/gps/rtk_status",
}
NTRIP_DEFAULT_ENV = "~/.config/rtk_imu/ntrip.env"
PL2303G_VID = 0x067B
PL2303G_PID = 0x23A3
TOPIC_PATTERN = r"/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*"


def parse_config(cfg):
    if not isinstance(cfg, dict):
        raise ValueError("Configuration must be an object")
    unknown = set(cfg) - {"serial", "ntrip", "frame_id", "status_rate_hz",
                          "sentinel_timeout_s", "topics"}
    if unknown:
        raise ValueError(f"Unknown config keys: {sorted(unknown)}")
    serial = _serial(cfg.get("serial", {}))
    ntrip = _ntrip(cfg.get("ntrip", {}))
    frame = cfg.get("frame_id", "gnss_link")
    if not isinstance(frame, str) or not frame or any(c.isspace() or c == "\0" for c in frame):
        raise ValueError("frame_id must be a nonempty string without whitespace")
    rate = cfg.get("status_rate_hz", 1.0)
    if type(rate) not in (int, float) or not math.isfinite(rate) or not 0 < rate <= 20:
        raise ValueError("status_rate_hz must be in (0,20] hertz")
    timeout = cfg.get("sentinel_timeout_s", 30.0)
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 120:
        raise ValueError("sentinel_timeout_s must be in (0,120] seconds")
    topics = cfg.get("topics", {})
    if not isinstance(topics, dict) or set(topics) - set(TOPICS):
        raise ValueError("Unknown topics key")
    topics = {**TOPICS, **topics}
    if any(not isinstance(topic, str) or re.fullmatch(TOPIC_PATTERN, topic) is None
           for topic in topics.values()):
        raise ValueError("Topics must be absolute ROS names without substitutions")
    if len(set(topics.values())) != len(topics):
        raise ValueError("Each stream must use a distinct topic")
    return {"serial": serial, "ntrip": ntrip, "frame_id": frame,
            "status_rate_hz": float(rate), "sentinel_timeout_s": float(timeout),
            "topics": topics}


def _serial(cfg):
    if not isinstance(cfg, dict):
        raise ValueError("serial must be an object")
    unknown = set(cfg) - {"port", "baud", "usb_vid", "usb_pid", "init_commands"}
    if unknown:
        raise ValueError(f"Unknown serial keys: {sorted(unknown)}")
    port = cfg.get("port", "auto")
    if not isinstance(port, str) or (port != "auto" and not Path(port).is_absolute()) \
            or any(c in port for c in "\r\n\0"):
        raise ValueError("serial.port must be auto or an absolute device path")
    baud = cfg.get("baud", 115200)
    if type(baud) is not int or baud <= 0:
        raise ValueError("serial.baud must be a positive integer")
    vid = cfg.get("usb_vid", PL2303G_VID)
    pid = cfg.get("usb_pid", PL2303G_PID)
    for key, value in (("usb_vid", vid), ("usb_pid", pid)):
        if type(value) is not int or not 0 <= value <= 65535:
            raise ValueError(f"serial.{key} must be an integer in [0,65535]")
    if ("usb_vid" in cfg) != ("usb_pid" in cfg):
        raise ValueError("serial.usb_vid and usb_pid must be specified together")
    if port != "auto" and ("usb_vid" in cfg or "usb_pid" in cfg):
        raise ValueError("serial.usb_vid/usb_pid are only used when serial.port is auto")
    commands = cfg.get("init_commands", ["MODE ROVER"])
    if not isinstance(commands, list):
        raise ValueError("serial.init_commands must be a list of strings")
    for command in commands:
        if not isinstance(command, str) or not command or len(command) > 128 \
                or any(ord(c) < 0x20 or ord(c) > 0x7E for c in command):
            raise ValueError("serial.init_commands entries must be nonempty printable ASCII without control characters")
    return {"port": port, "baud": baud, "usb_vid": vid, "usb_pid": pid,
            "init_commands": list(commands)}


def _ntrip(cfg):
    if not isinstance(cfg, dict):
        raise ValueError("ntrip must be an object")
    unknown = set(cfg) - {"enabled", "env_file", "reconnect_s", "gga_interval_s"}
    if unknown:
        raise ValueError(f"Unknown ntrip keys: {sorted(unknown)}")
    enabled = cfg.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError("ntrip.enabled must be boolean")
    env_file = cfg.get("env_file", NTRIP_DEFAULT_ENV)
    if not isinstance(env_file, str) or not env_file or any(c in env_file for c in "\r\n\0"):
        raise ValueError("ntrip.env_file must be a nonempty path")
    reconnect = cfg.get("reconnect_s", 10.0)
    if type(reconnect) not in (int, float) or not math.isfinite(reconnect) or not 0 < reconnect <= 300:
        raise ValueError("ntrip.reconnect_s must be in (0,300] seconds")
    gga_interval = cfg.get("gga_interval_s", 10.0)
    if type(gga_interval) not in (int, float) or not math.isfinite(gga_interval) \
            or not 0 < gga_interval <= 120:
        raise ValueError("ntrip.gga_interval_s must be in (0,120] seconds")
    return {"enabled": enabled, "env_file": env_file,
            "reconnect_s": float(reconnect), "gga_interval_s": float(gga_interval)}


def resolve_port(serial):
    if serial["port"] != "auto":
        return serial["port"]
    candidates = []
    aliases = Path("/dev/rtk")
    if aliases.exists():
        candidates.append(aliases)
    if not candidates:
        from serial.tools import list_ports
        candidates = [Path(info.device) for info in list_ports.comports()
                      if info.vid == serial["usb_vid"] and info.pid == serial["usb_pid"]]
    unique = {path.resolve(): path for path in candidates}
    if len(unique) != 1:
        raise ValueError("Cannot uniquely identify the BT-982G1 receiver; "
                         "set serial.port to /dev/rtk or its stable by-id path")
    return str(next(iter(unique.values())))
