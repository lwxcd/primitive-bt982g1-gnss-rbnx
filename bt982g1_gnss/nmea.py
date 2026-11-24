# SPDX-License-Identifier: MulanPSL-2.0
"""NMEA 0183 parsing shared by the BT-982G1 receiver and its tests."""
from __future__ import annotations

import math
from typing import Optional

FIX_NAMES = {
    0: "no fix",
    1: "single",
    2: "DGPS",
    3: "PPS",
    4: "RTK fixed",
    5: "RTK float",
    6: "dead reckoning",
}

STATE_NAMES = {
    0: "NO_FIX",
    1: "SINGLE",
    2: "DGPS",
    3: "PPS",
    4: "RTK_FIXED",
    5: "RTK_FLOAT",
    6: "DEAD_RECKONING",
    7: "MANUAL",
    8: "SIMULATION",
}


def _float(text: str) -> Optional[float]:
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _int(text: str) -> Optional[int]:
    try:
        return int(text)
    except (TypeError, ValueError):
        return None


def utc_seconds(text: Optional[str]) -> Optional[float]:
    """NMEA hhmmss.ss to seconds of day; None when malformed."""
    if not isinstance(text, str) or len(text) < 6 or not text[:6].isdigit():
        return None
    try:
        hour, minute, second = int(text[:2]), int(text[2:4]), float(text[4:])
    except ValueError:
        return None
    if not (0 <= hour < 24 and 0 <= minute < 60 and math.isfinite(second)
            and 0 <= second < 61):
        return None
    return hour * 3600 + minute * 60 + second


def checksum_ok(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, checksum = line[1:].rpartition("*")
    if len(checksum) < 2:
        return False
    try:
        expected = int(checksum[:2], 16)
    except ValueError:
        return False
    actual = 0
    for char in body:
        actual ^= ord(char)
    return actual == expected


def sentence_fields(line: str) -> Optional[list[str]]:
    """Return comma-separated NMEA fields without the checksum, or None."""
    body = line.strip()
    if not body.startswith("$"):
        return None
    fields = body[1:].split("*", 1)[0].split(",")
    if not fields or len(fields[0]) < 4:
        return None
    return fields


def sentence_kind(line: str) -> Optional[str]:
    fields = sentence_fields(line)
    return None if fields is None else fields[0][-3:]


def coordinate(value: str, hemisphere: str) -> Optional[float]:
    raw = _float(value)
    if raw is None:
        return None
    degrees = int(raw // 100)
    out = degrees + (raw - degrees * 100) / 60.0
    return -out if hemisphere in ("S", "W") else out


def parse_gga(line: str) -> Optional[dict]:
    fields = sentence_fields(line)
    if fields is None or len(fields) < 10 or fields[0][-3:] != "GGA":
        return None
    utc_time = fields[1] or None
    return {
        "quality": _int(fields[6] or "0") or 0,
        "sats": _int(fields[7] or "0") or 0,
        "hdop": _float(fields[8]),
        "lat": coordinate(fields[2], fields[3]) if fields[2] else None,
        "lon": coordinate(fields[4], fields[5]) if fields[4] else None,
        "alt": _float(fields[9]),
        "diff_age": _float(fields[13]) if len(fields) > 13 else None,
        "utc_time": utc_time,
        "epoch_s": utc_seconds(utc_time),
    }


def parse_rmc(line: str) -> Optional[dict]:
    fields = sentence_fields(line)
    if fields is None or len(fields) < 9 or fields[0][-3:] != "RMC":
        return None
    status = fields[2].strip().upper()
    utc_time = fields[1] or None
    mode = fields[12].strip().upper() if len(fields) > 12 else ""
    if status != "A" or mode == "N":
        return {"status": status, "valid": False, "utc_time": utc_time,
                "epoch_s": None, "speed_mps": None, "course_deg": None,
                "east_mps": None, "north_mps": None}
    knots = _float(fields[7])
    course = _float(fields[8])
    epoch = utc_seconds(utc_time)
    valid = knots is not None and math.isfinite(knots) and knots >= 0 and epoch is not None
    if course is not None and (not math.isfinite(course) or not 0.0 <= course < 360.0):
        course = None
    if valid and course is None and knots != 0:
        valid = False
    speed_mps = None if knots is None else knots * 0.514444
    east = north = None
    if valid:
        angle = math.radians(course or 0.0)
        east = speed_mps * math.sin(angle)
        north = speed_mps * math.cos(angle)
    return {
        "status": status,
        "valid": valid,
        "utc_time": utc_time,
        "epoch_s": epoch,
        "speed_mps": speed_mps,
        "course_deg": course,
        "east_mps": east,
        "north_mps": north,
    }


def parse_gst(line: str) -> Optional[dict]:
    """Receiver-estimated position sigmas; never derived from HDOP."""
    fields = sentence_fields(line)
    if fields is None or len(fields) < 9 or fields[0][-3:] != "GST":
        return None
    utc_time = fields[1] or None
    epoch = utc_seconds(utc_time)
    values = [_float(fields[6]), _float(fields[7]), _float(fields[8])]
    if epoch is None or any(v is None or not math.isfinite(v) or v <= 0.0 for v in values):
        return None
    return {"utc_time": utc_time, "epoch_s": epoch,
            "latitude_sigma_m": values[0], "longitude_sigma_m": values[1],
            "altitude_sigma_m": values[2]}


def parse_vtg(line: str) -> Optional[dict]:
    fields = sentence_fields(line)
    if fields is None or len(fields) < 8 or fields[0][-3:] != "VTG":
        return None
    course = _float(fields[1])
    kmh = _float(fields[7])
    if course is None and kmh is None:
        return None
    return {"speed_mps": None if kmh is None else kmh / 3.6, "course_deg": course}


def parse_heading(line: str) -> Optional[dict]:
    fields = sentence_fields(line)
    if fields is None or len(fields) < 2:
        return None
    kind = fields[0][-3:]
    if kind == "THS" and len(fields) >= 3:
        heading = _float(fields[1])
        if heading is None or fields[2].strip().upper() != "A":
            return None
        return {"source": "THS", "heading_deg": heading % 360.0, "pitch_deg": None}
    if kind == "TRA" and len(fields) >= 2:
        heading = _float(fields[1])
        if heading is None or not 0.0 <= heading <= 360.0:
            return None
        return {"source": "TRA", "heading_deg": heading % 360.0, "pitch_deg": _float(fields[2])}
    return None
