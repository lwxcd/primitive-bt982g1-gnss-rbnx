# SPDX-License-Identifier: MulanPSL-2.0
"""Serial receiver thread: NMEA ingest, freshness tracking and correction writes.

Adapted from robot-deep_robotics-lite3/scripts/rtk_imu/rover_status.py and
ntrip_client.py, without the dashboard, CSV logging or IMU port ownership.
"""
from __future__ import annotations

from collections import deque
import math
import threading
import time
from typing import Callable, Optional

import serial

from .nmea import (checksum_ok, parse_gga, parse_gst, parse_heading, parse_rmc,
                   parse_vtg, utc_seconds)

FIX_FRESH_S = 2.0
HEADING_FRESH_S = 1.5
GGA_UPLINK_MAX_AGE_S = 10.0
POSITION_DIFF_MIN_BASELINE_S = 0.5
POSITION_DIFF_MAX_BASELINE_S = 1.5
POSITION_DIFF_TARGET_BASELINE_S = 1.0
POSITION_DIFF_MAX_SPEED_MPS = 10.0
POSITION_DIFF_HISTORY_S = 4.0
WGS84_A_M = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


def enu_offset(old: tuple, new: tuple) -> tuple[float, float]:
    """East/north metres from ``old`` to ``new`` on the WGS84 local tangent
    plane at the older sample.  Samples are (mono, utc, lat, lon, quality)."""
    lat0 = math.radians(old[2])
    dlat = math.radians(new[2] - old[2])
    dlon = math.radians(new[3] - old[3])
    sin_lat = math.sin(lat0)
    denom = 1.0 - WGS84_E2 * sin_lat * sin_lat
    prime_vertical = WGS84_A_M / math.sqrt(denom)
    meridian = WGS84_A_M * (1.0 - WGS84_E2) / denom ** 1.5
    return dlon * prime_vertical * math.cos(lat0), dlat * meridian


def position_diff_velocity(samples, newest: tuple) -> Optional[dict]:
    """ENU velocity from two RTK-fixed GGA samples, or None.

    ``samples`` holds (mono, utc, lat, lon, quality) with ``newest`` included.
    Both endpoints must be quality 4 and the baseline 0.5-1.5 s (closest to
    1.0 s); no sample outside that window means no velocity, never a guess.
    """
    if newest[4] != 4:
        return None
    best = None
    for old in samples:
        if old is newest or old[4] != 4:
            continue
        baseline = newest[0] - old[0]
        if not POSITION_DIFF_MIN_BASELINE_S <= baseline <= POSITION_DIFF_MAX_BASELINE_S:
            continue
        score = abs(baseline - POSITION_DIFF_TARGET_BASELINE_S)
        if best is None or score < best[0]:
            best = (score, old, baseline)
    if best is None:
        return None
    _, old, baseline = best
    east_m, north_m = enu_offset(old, newest)
    east, north = east_m / baseline, north_m / baseline
    speed = math.hypot(east, north)
    if not math.isfinite(speed) or speed > POSITION_DIFF_MAX_SPEED_MPS:
        return None
    return {
        "status": "A",
        "valid": True,
        "utc_time": newest[1],
        "epoch_s": utc_seconds(newest[1]),
        "speed_mps": speed,
        "course_deg": math.degrees(math.atan2(east, north)) % 360.0,
        "east_mps": east,
        "north_mps": north,
    }


def _stamp_seconds(stamp: Optional[tuple[int, int]]) -> Optional[float]:
    return None if stamp is None else stamp[0] + stamp[1] / 1e9


class Receiver(threading.Thread):
    def __init__(self, port: str, baud: int, init_commands: list[str],
                 on_fix: Callable[[dict], None], log: Callable[[str], None],
                 on_velocity: Optional[Callable[[dict], None]] = None,
                 on_sigma: Optional[Callable[[dict], None]] = None,
                 velocity_source: str = "rmc") -> None:
        super().__init__(daemon=True, name="bt982g1-receiver")
        self.port = port
        self.baud = baud
        self.init_commands = init_commands
        self.on_fix = on_fix
        self.on_velocity = on_velocity
        self.on_sigma = on_sigma
        self.velocity_source = velocity_source
        self.velocity_kind: Optional[str] = None
        self.gga_history: deque = deque()
        self.log = log
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.serial: Optional[serial.Serial] = None
        self.serial_status = "closed"
        self.counts = {"gga": 0, "rmc": 0, "vtg": 0, "gst": 0,
                       "ths": 0, "tra": 0, "other": 0}
        self.checksum_errors = 0
        self.last_measurement_mono: Optional[float] = None
        self.last_fix_mono: Optional[float] = None
        self.last_gga_line: Optional[bytes] = None
        self.last_gga_mono: Optional[float] = None
        self.last_gga_sample_mono: Optional[float] = None
        self.utc_time: Optional[str] = None
        self.last_fix_info: Optional[dict] = None
        self.fix_stamp: Optional[tuple[int, int]] = None
        self.velocity: Optional[dict] = None
        self.velocity_at: Optional[float] = None
        self.velocity_stamp: Optional[tuple[int, int]] = None
        self.sigma: Optional[dict] = None
        self.sigma_at: Optional[float] = None
        self.lat: Optional[float] = None
        self.lon: Optional[float] = None
        self.alt: Optional[float] = None
        self.quality: Optional[int] = None
        self.sats: Optional[int] = None
        self.hdop: Optional[float] = None
        self.diff_age: Optional[float] = None
        self.speed_mps: Optional[float] = None
        self.course_deg: Optional[float] = None
        self.heading_deg: Optional[float] = None
        self.heading_pitch_deg: Optional[float] = None
        self.heading_source: Optional[str] = None
        self.heading_mono: Optional[float] = None

    def run(self) -> None:
        while not self.stop_event.is_set():
            with self.lock:
                ser = self.serial
            if ser is None:
                try:
                    ser = serial.Serial(self.port, self.baud, timeout=0.2)
                except Exception as exc:  # noqa: BLE001
                    with self.lock:
                        self.serial_status = f"open failed: {exc}"
                    self.log(f"serial: {exc}")
                    self.stop_event.wait(1.0)
                    continue
                with self.lock:
                    self.serial = ser
                    self.serial_status = "open"
                self.log(f"serial open {self.port} @ {self.baud}")
                for command in self.init_commands:
                    try:
                        ser.write(command.encode("ascii") + b"\r\n")
                        self.log(f"init command: {command}")
                    except Exception as exc:  # noqa: BLE001
                        self.log(f"init command failed: {exc}")
            try:
                line = ser.readline()
            except Exception as exc:  # noqa: BLE001
                self.log(f"serial read failed: {exc}")
                self.close()
                self.stop_event.wait(1.0)
                continue
            if line:
                self._handle(line)
        self.close()

    def _handle(self, line: bytes) -> None:
        text = line.decode("ascii", "ignore").strip()
        if not text:
            return
        if text.startswith("$command"):
            self.log(f"receiver: {text}")
            return
        if not text.startswith("$"):
            return
        if not checksum_ok(text):
            with self.lock:
                self.checksum_errors += 1
            return
        kind = text[3:6]
        now = time.monotonic()
        with self.lock:
            self.last_measurement_mono = now
        if kind == "GGA":
            info = parse_gga(text)
            if info is not None:
                self._handle_gga(info, text, now)
        elif kind == "RMC":
            info = parse_rmc(text)
            if info is not None:
                with self.lock:
                    self.counts["rmc"] += 1
                    self.speed_mps = info["speed_mps"]
                    self.course_deg = info["course_deg"]
                    use_rmc = self.velocity_source == "rmc" or (
                        self.velocity_source == "auto" and not (
                            self.quality == 4
                            and self.velocity_kind == "POSITION_DIFF"
                            and self.velocity_at is not None
                            and now - self.velocity_at <= FIX_FRESH_S))
                    if info["valid"] and use_rmc:
                        self.velocity = dict(info)
                        self.velocity_at = now
                        self.velocity_kind = "NMEA_RMC"
                    elif not info["valid"] and use_rmc:
                        self.velocity = None
                        self.velocity_at = None
                        self.velocity_kind = None
                if info["valid"] and use_rmc and self.on_velocity is not None:
                    self.on_velocity(info)
        elif kind == "VTG":
            info = parse_vtg(text)
            if info is not None:
                with self.lock:
                    self.counts["vtg"] += 1
                    if info["speed_mps"] is not None:
                        self.speed_mps = info["speed_mps"]
                    if info["course_deg"] is not None:
                        self.course_deg = info["course_deg"]
        elif kind == "GST":
            info = parse_gst(text)
            if info is not None:
                with self.lock:
                    self.counts["gst"] += 1
                    self.sigma = dict(info)
                    self.sigma_at = now
                if self.on_sigma is not None:
                    self.on_sigma(info)
        elif kind in ("THS", "TRA"):
            info = parse_heading(text)
            if info is not None:
                with self.lock:
                    self.counts[info["source"].lower()] += 1
                    self.heading_deg = info["heading_deg"]
                    self.heading_pitch_deg = info["pitch_deg"]
                    self.heading_source = info["source"]
                    self.heading_mono = now
        else:
            with self.lock:
                self.counts["other"] += 1

    def _handle_gga(self, info: dict, line: str, now: float) -> None:
        has_position = info["lat"] is not None and info["lon"] is not None and info["quality"] > 0
        sample = None
        if info["lat"] is not None and info["lon"] is not None:
            sample = (now, info["utc_time"], info["lat"], info["lon"],
                      info["quality"])
        velocity = None
        with self.lock:
            self.counts["gga"] += 1
            self.quality = info["quality"]
            self.sats = info["sats"]
            self.hdop = info["hdop"]
            self.diff_age = info["diff_age"]
            self.utc_time = info["utc_time"]
            self.last_gga_sample_mono = now
            if info["lat"] is not None and info["lon"] is not None:
                self.lat, self.lon, self.alt = info["lat"], info["lon"], info["alt"]
            if has_position:
                self.last_fix_info = dict(info)
                self.last_fix_mono = now
                self.last_gga_line = (line + "\r\n").encode("ascii", "ignore")
                self.last_gga_mono = now
            if sample is not None:
                self.gga_history.append(sample)
                cutoff = now - POSITION_DIFF_HISTORY_S
                while self.gga_history and self.gga_history[0][0] < cutoff:
                    self.gga_history.popleft()
                if self.velocity_source in ("position_diff", "auto"):
                    velocity = position_diff_velocity(self.gga_history, sample)
                    if velocity is not None:
                        self.velocity = velocity
                        self.velocity_at = now
                        self.velocity_kind = "POSITION_DIFF"
        if velocity is not None and self.on_velocity is not None:
            self.on_velocity(velocity)
        if has_position:
            self.on_fix(info)

    def fresh_gga(self, max_age: float = GGA_UPLINK_MAX_AGE_S) -> Optional[bytes]:
        with self.lock:
            if self.last_gga_line is None or self.last_gga_mono is None \
                    or self.lat is None or self.lon is None:
                return None
            if time.monotonic() - self.last_gga_mono > max_age:
                return None
            return self.last_gga_line

    def record_fix_stamp(self, stamp) -> None:
        with self.lock:
            self.fix_stamp = (int(stamp.sec), int(stamp.nanosec))

    def record_velocity_stamp(self, stamp) -> None:
        with self.lock:
            self.velocity_stamp = (int(stamp.sec), int(stamp.nanosec))

    def latest_fix(self) -> tuple[Optional[dict], Optional[tuple[int, int]]]:
        with self.lock:
            info = dict(self.last_fix_info) if self.last_fix_info is not None else None
            stamp = self.fix_stamp
        return info, stamp

    def sigma_for(self, utc_time: Optional[str],
                  max_age: float = FIX_FRESH_S) -> Optional[dict]:
        now = time.monotonic()
        with self.lock:
            if self.sigma is None or self.sigma_at is None:
                return None
            if not 0.0 <= now - self.sigma_at <= max_age:
                return None
            if utc_time is None or self.sigma["utc_time"] != utc_time:
                return None
            return dict(self.sigma)

    def write(self, data: bytes) -> bool:
        with self.write_lock:
            with self.lock:
                ser = self.serial
            if ser is None:
                return False
            try:
                ser.write(data)
                return True
            except Exception as exc:  # noqa: BLE001
                self.log(f"serial write failed: {exc}")
                self.close()
                return False

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            fix_age = None if self.last_fix_mono is None else max(0.0, now - self.last_fix_mono)
            measurement_age = (None if self.last_measurement_mono is None
                               else max(0.0, now - self.last_measurement_mono))
            heading_age = None if self.heading_mono is None else max(0.0, now - self.heading_mono)
            heading = self.heading_deg if heading_age is not None and heading_age <= HEADING_FRESH_S else None
            gga_age = (None if self.last_gga_sample_mono is None
                       else max(0.0, now - self.last_gga_sample_mono))
            valid = (self.serial is not None and self.lat is not None and self.lon is not None
                     and self.quality is not None and self.quality > 0
                     and fix_age is not None and fix_age <= FIX_FRESH_S)
            velocity_age = (None if self.velocity_at is None
                            else max(0.0, now - self.velocity_at))
            velocity = self.velocity
            source = self.velocity_kind or (
                "POSITION_DIFF" if self.velocity_source == "position_diff"
                else "NMEA_RMC")
            velocity_status = {
                "source": source,
                "fresh": bool(velocity is not None and velocity_age is not None
                              and velocity_age <= FIX_FRESH_S),
                "age_s": velocity_age,
                "utc_time": velocity["utc_time"] if velocity is not None else None,
                "stamp_s": _stamp_seconds(self.velocity_stamp),
                "reference_frame": "ENU",
                "covariance_known": False,
            }
            sigma = None
            if (self.sigma is not None and self.sigma_at is not None
                    and 0.0 <= now - self.sigma_at <= FIX_FRESH_S
                    and self.last_fix_info is not None
                    and self.sigma["utc_time"] == self.last_fix_info.get("utc_time")):
                sigma = dict(self.sigma)
            return {
                "serial_connected": self.serial is not None,
                "serial_status": self.serial_status,
                "port": self.port,
                "baud": self.baud,
                "valid": valid,
                "quality": self.quality,
                "fix_age_s": fix_age,
                "gga_age_s": gga_age,
                "measurement_age_s": measurement_age,
                "lat": self.lat,
                "lon": self.lon,
                "alt_m": self.alt,
                "sats": self.sats,
                "hdop": self.hdop,
                "diff_age_s": self.diff_age,
                "speed_mps": self.speed_mps,
                "course_deg": self.course_deg,
                "utc_time": self.utc_time,
                "fix_stamp_s": _stamp_seconds(self.fix_stamp),
                "velocity": velocity_status,
                "sigma": sigma,
                "heading_deg": heading,
                "heading_pitch_deg": self.heading_pitch_deg if heading is not None else None,
                "heading_source": self.heading_source if heading is not None else None,
                "counts": dict(self.counts),
                "checksum_errors": self.checksum_errors,
            }

    def close(self) -> None:
        with self.lock:
            ser = self.serial
            self.serial = None
            self.serial_status = "closed"
            self.gga_history.clear()
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def stop(self) -> None:
        self.stop_event.set()
        self.close()
