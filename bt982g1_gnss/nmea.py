# SPDX-License-Identifier: MulanPSL-2.0
"""NMEA 0183 parsing shared by the BT-982G1 receiver and its tests."""
from __future__ import annotations

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
    return {
        "quality": _int(fields[6] or "0") or 0,
        "sats": _int(fields[7] or "0") or 0,
        "hdop": _float(fields[8]),
        "lat": coordinate(fields[2], fields[3]) if fields[2] else None,
        "lon": coordinate(fields[4], fields[5]) if fields[4] else None,
        "alt": _float(fields[9]),
        "diff_age": _float(fields[13]) if len(fields) > 13 else None,
    }


def parse_rmc(line: str) -> Optional[dict]:
    fields = sentence_fields(line)
    if fields is None or len(fields) < 9 or fields[0][-3:] != "RMC":
        return None
    status = fields[2].strip().upper()
    if status != "A":
        return {"status": status, "speed_mps": None, "course_deg": None}
    knots = _float(fields[7])
    course = _float(fields[8])
    return {
        "status": status,
        "speed_mps": None if knots is None else knots * 0.514444,
        "course_deg": course,
    }


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
