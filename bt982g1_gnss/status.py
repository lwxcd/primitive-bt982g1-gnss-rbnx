# SPDX-License-Identifier: MulanPSL-2.0
"""rtk_status JSON payload; kept free of ROS imports so it can be tested offline."""
from __future__ import annotations

import math

from .nmea import FIX_NAMES


def _round(value, digits):
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return round(value, digits) if math.isfinite(value) else None


def build_status(snapshot: dict, ntrip: dict, frame_id: str) -> dict:
    quality = snapshot["quality"]
    return {
        "timestamp_source": "HOST_SERIAL_RECEIVE",
        "serial_connected": snapshot["serial_connected"],
        "serial_status": snapshot["serial_status"],
        "port": snapshot["port"],
        "baud": snapshot["baud"],
        "frame_id": frame_id,
        "receiver_quality": quality,
        "receiver_quality_name": FIX_NAMES.get(quality) if quality is not None else None,
        "fix_age_s": _round(snapshot["fix_age_s"], 3),
        "measurement_age_s": _round(snapshot["measurement_age_s"], 3),
        "valid": snapshot["valid"],
        "lat": _round(snapshot["lat"], 9),
        "lon": _round(snapshot["lon"], 9),
        "alt_m": _round(snapshot["alt_m"], 3),
        "sats": snapshot["sats"],
        "hdop": _round(snapshot["hdop"], 2),
        "diff_age_s": _round(snapshot["diff_age_s"], 2),
        "speed_mps": _round(snapshot["speed_mps"], 3),
        "course_deg": _round(snapshot["course_deg"], 2),
        "heading_deg": _round(snapshot["heading_deg"], 2),
        "heading_pitch_deg": _round(snapshot["heading_pitch_deg"], 2),
        "heading_source": snapshot["heading_source"],
        "counts": dict(snapshot["counts"]),
        "checksum_errors": snapshot["checksum_errors"],
        "ntrip": dict(ntrip),
    }
