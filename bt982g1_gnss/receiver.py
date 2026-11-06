# SPDX-License-Identifier: MulanPSL-2.0
"""Serial receiver thread: NMEA ingest, freshness tracking and correction writes.

Adapted from robot-deep_robotics-lite3/scripts/rtk_imu/rover_status.py and
ntrip_client.py, without the dashboard, CSV logging or IMU port ownership.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import serial

from .nmea import checksum_ok, parse_gga, parse_heading, parse_rmc, parse_vtg

FIX_FRESH_S = 2.0
HEADING_FRESH_S = 1.5
GGA_UPLINK_MAX_AGE_S = 10.0


class Receiver(threading.Thread):
    def __init__(self, port: str, baud: int, init_commands: list[str],
                 on_fix: Callable[[dict], None], log: Callable[[str], None]) -> None:
        super().__init__(daemon=True, name="bt982g1-receiver")
        self.port = port
        self.baud = baud
        self.init_commands = init_commands
        self.on_fix = on_fix
        self.log = log
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.serial: Optional[serial.Serial] = None
        self.serial_status = "closed"
        self.counts = {"gga": 0, "rmc": 0, "vtg": 0, "ths": 0, "tra": 0, "other": 0}
        self.checksum_errors = 0
        self.last_measurement_mono: Optional[float] = None
        self.last_fix_mono: Optional[float] = None
        self.last_gga_line: Optional[bytes] = None
        self.last_gga_mono: Optional[float] = None
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
        elif kind == "VTG":
            info = parse_vtg(text)
            if info is not None:
                with self.lock:
                    self.counts["vtg"] += 1
                    if info["speed_mps"] is not None:
                        self.speed_mps = info["speed_mps"]
                    if info["course_deg"] is not None:
                        self.course_deg = info["course_deg"]
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
        with self.lock:
            self.counts["gga"] += 1
            self.quality = info["quality"]
            self.sats = info["sats"]
            self.hdop = info["hdop"]
            self.diff_age = info["diff_age"]
            if info["lat"] is not None and info["lon"] is not None:
                self.lat, self.lon, self.alt = info["lat"], info["lon"], info["alt"]
            if has_position:
                self.last_fix_mono = now
                self.last_gga_line = (line + "\r\n").encode("ascii", "ignore")
                self.last_gga_mono = now
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
            valid = (self.serial is not None and self.lat is not None and self.lon is not None
                     and self.quality is not None and self.quality > 0
                     and fix_age is not None and fix_age <= FIX_FRESH_S)
            return {
                "serial_connected": self.serial is not None,
                "serial_status": self.serial_status,
                "port": self.port,
                "baud": self.baud,
                "valid": valid,
                "quality": self.quality,
                "fix_age_s": fix_age,
                "measurement_age_s": measurement_age,
                "lat": self.lat,
                "lon": self.lon,
                "alt_m": self.alt,
                "sats": self.sats,
                "hdop": self.hdop,
                "diff_age_s": self.diff_age,
                "speed_mps": self.speed_mps,
                "course_deg": self.course_deg,
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
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def stop(self) -> None:
        self.stop_event.set()
        self.close()
