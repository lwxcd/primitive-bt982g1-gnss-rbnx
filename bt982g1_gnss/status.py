# SPDX-License-Identifier: MulanPSL-2.0
"""rtk_status JSON payload; kept free of ROS imports so it can be tested offline."""
from __future__ import annotations

import math

from .nmea import FIX_NAMES, STATE_NAMES

FIX_FRESH_S = 2.0


def _round(value, digits):
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return round(value, digits) if math.isfinite(value) else None


def _stamp(value):
    """ROS stamp seconds as JSON number; never rounded (consumers key on it)."""
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def position_covariance(sigma) -> tuple[float, float, float] | None:
    """Map verified GST sigmas to NavSatFix diagonal variances (lon, lat, alt)."""
    if not isinstance(sigma, dict):
        return None
    try:
        values = [float(sigma[key]) for key in
                  ("longitude_sigma_m", "latitude_sigma_m", "altitude_sigma_m")]
    except (KeyError, TypeError, ValueError):
        return None
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        return None
    return tuple(value * value for value in values)


def build_status(snapshot: dict, ntrip: dict, frame_id: str) -> dict:
    quality = snapshot["quality"]
    fix_age = snapshot["fix_age_s"]
    gga_age = snapshot.get("gga_age_s", fix_age)
    if quality is None:
        state = "NO_DATA"
    elif (not snapshot["serial_connected"] or gga_age is None
          or not math.isfinite(float(gga_age)) or gga_age > FIX_FRESH_S):
        state = "STALE"
    else:
        state = STATE_NAMES.get(quality, "UNKNOWN_QUALITY")
    velocity = snapshot.get("velocity") or {}
    sigma = snapshot.get("sigma")
    covariance_source = "NMEA_GST" if position_covariance(sigma) is not None else "UNKNOWN"
    return {
        "timestamp_source": "HOST_SERIAL_RECEIVE",
        "serial_connected": snapshot["serial_connected"],
        "serial_status": snapshot["serial_status"],
        "serial_state": "OPEN" if snapshot["serial_connected"] else "DISCONNECTED",
        "serial_error": "" if snapshot["serial_connected"] else snapshot["serial_status"],
        "port": snapshot["port"],
        "baud": snapshot["baud"],
        "frame_id": frame_id,
        "state": state,
        "rtk_fixed": bool(state == "RTK_FIXED" and snapshot["valid"]),
        "receiver_quality": quality,
        "receiver_quality_name": FIX_NAMES.get(quality) if quality is not None else None,
        "fix_age_s": _round(fix_age, 3),
        "gga_age_s": _round(gga_age, 3),
        "measurement_age_s": _round(snapshot["measurement_age_s"], 3),
        "fix_stamp_s": _stamp(snapshot.get("fix_stamp_s")),
        "utc_time": snapshot.get("utc_time"),
        "valid": snapshot["valid"],
        "lat": _round(snapshot["lat"], 9),
        "lon": _round(snapshot["lon"], 9),
        "alt_m": _round(snapshot["alt_m"], 3),
        "sats": snapshot["sats"],
        "satellites": snapshot["sats"],
        "hdop": _round(snapshot["hdop"], 2),
        "diff_age_s": _round(snapshot["diff_age_s"], 2),
        "differential_age_s": _round(snapshot["diff_age_s"], 2),
        "speed_mps": _round(snapshot["speed_mps"], 3),
        "course_deg": _round(snapshot["course_deg"], 2),
        "velocity": {
            "source": velocity.get("source", "NMEA_RMC"),
            "fresh": velocity.get("fresh") is True,
            "age_s": _round(velocity.get("age_s"), 3),
            "utc_time": velocity.get("utc_time"),
            "stamp_s": _stamp(velocity.get("stamp_s")),
            "reference_frame": velocity.get("reference_frame", "ENU"),
            "covariance_known": False,
        },
        "position_covariance_source": covariance_source,
        "heading_deg": _round(snapshot["heading_deg"], 2),
        "heading_pitch_deg": _round(snapshot["heading_pitch_deg"], 2),
        "heading_source": snapshot["heading_source"],
        "counts": dict(snapshot["counts"]),
        "checksum_errors": snapshot["checksum_errors"],
        "bad_nmea": snapshot["checksum_errors"],
        "ntrip": dict(ntrip),
    }
