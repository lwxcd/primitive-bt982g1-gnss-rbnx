# SPDX-License-Identifier: MulanPSL-2.0
import math
import os
import pty
import time
from types import SimpleNamespace

import bt982g1_gnss.receiver as receiver_mod
from bt982g1_gnss.receiver import Receiver

GGA_FIXED = "$GNGGA,023034.00,3112.38608,N,12127.29727,E,4,28,0.6,12.345,M,-8.0,M,1.2,0000*7E"
RMC = "$GNRMC,023034.00,A,3112.38608,N,12127.29727,E,0.514,45.0,061126,,,A*7F"
VTG = "$GNVTG,45.0,T,,M,1.852,N,3.704,K,A*2C"
THS = "$GPTHS,90.5,A*0B"
GST = "$GNGST,023034.00,0.8,0.5,0.3,32.6,0.6,0.4,0.9*7D"


def nmea_coordinate(value, positive, negative, digits):
    hemisphere = positive if value >= 0 else negative
    value = abs(value)
    minutes = (value - int(value)) * 60.0
    return f"{int(value):0{digits}d}{minutes:08.5f},{hemisphere}"


def gga_line(utc, lat, lon, quality=4):
    body = (f"GNGGA,{utc},{nmea_coordinate(lat, 'N', 'S', 2)},"
            f"{nmea_coordinate(lon, 'E', 'W', 3)},"
            f"{quality},28,0.6,12.345,M,-8.0,M,1.2,0000")
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"${body}*{checksum:02X}"


def gga_sample(mono, lat, lon, quality=4, utc="023034.00"):
    return (mono, utc, lat, lon, quality)


def wait_until(predicate, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_position_diff_velocity_requires_fixed_baseline():
    lat, lon = 31.206434666, 121.4549545
    metre_east = 1.0 / (111320.0 * math.cos(math.radians(lat)))
    old = gga_sample(10.0, lat, lon)
    new = gga_sample(11.0, lat, lon + metre_east, utc="023035.00")
    velocity = receiver_mod.position_diff_velocity([old, new], new)
    assert velocity is not None
    assert abs(velocity["east_mps"] - 1.0) < 0.01
    assert abs(velocity["north_mps"]) < 0.01
    assert velocity["utc_time"] == "023035.00"

    floating = gga_sample(11.0, lat, lon + metre_east, quality=5)
    assert receiver_mod.position_diff_velocity([old, floating], floating) is None
    too_short = gga_sample(10.2, lat, lon + metre_east * 0.2)
    assert receiver_mod.position_diff_velocity([old, too_short], too_short) is None
    too_long = gga_sample(12.0, lat, lon + metre_east * 2.0)
    assert receiver_mod.position_diff_velocity([old, too_long], too_long) is None
    too_fast = gga_sample(11.0, lat, lon + metre_east * 20.0)
    assert receiver_mod.position_diff_velocity([old, too_fast], too_fast) is None


def test_position_diff_velocity_picks_closest_baseline():
    lat, lon = 31.206434666, 121.4549545
    metre_east = 1.0 / (111320.0 * math.cos(math.radians(lat)))
    near = gga_sample(10.0, lat, lon)
    far = gga_sample(9.2, lat, lon)
    new = gga_sample(11.0, lat, lon + metre_east)
    velocity = receiver_mod.position_diff_velocity([far, near, new], new)
    assert velocity is not None
    # 1.0 s baseline (near) wins over 1.8 s, which is outside the window anyway.
    assert abs(velocity["east_mps"] - 1.0) < 0.01


def test_position_diff_mode_never_falls_back_to_rmc():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    velocities = []
    receiver = Receiver(path, 115200, [], lambda info: None, lambda message: None,
                        on_velocity=velocities.append, velocity_source="position_diff")
    receiver.start()
    try:
        assert wait_until(lambda: receiver.snapshot()["serial_connected"])
        os.write(master, (GGA_FIXED + "\r\n").encode())
        assert wait_until(lambda: receiver.snapshot()["quality"] == 4)
        os.write(master, (RMC + "\r\n").encode())
        time.sleep(0.3)
        assert velocities == []
        snapshot = receiver.snapshot()
        assert snapshot["velocity"]["source"] == "POSITION_DIFF"
        assert snapshot["velocity"]["fresh"] is False
        assert snapshot["velocity"]["utc_time"] is None
        assert snapshot["speed_mps"] is not None
    finally:
        receiver.stop()
        receiver.join(timeout=2)
        os.close(master)
        os.close(slave)


def test_auto_position_diff_velocity_over_pty():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    velocities = []
    receiver = Receiver(path, 115200, [], lambda info: None, lambda message: None,
                        on_velocity=velocities.append, velocity_source="auto")
    receiver.start()
    try:
        assert wait_until(lambda: receiver.snapshot()["serial_connected"])
        lat, lon = 31.206434666, 121.4549545
        os.write(master, (gga_line("023034.00", lat, lon) + "\r\n").encode())
        assert wait_until(lambda: receiver.snapshot()["quality"] == 4)
        time.sleep(0.7)
        metre_east = 0.7 / (111320.0 * math.cos(math.radians(lat)))
        os.write(master, (gga_line("023035.00", lat, lon + metre_east) + "\r\n").encode())
        assert wait_until(lambda: any(v.get("east_mps") is not None
                                      for v in velocities))
        velocity = velocities[-1]
        assert abs(velocity["east_mps"] - 1.0) < 0.15
        assert abs(velocity["north_mps"]) < 0.1
        snapshot = receiver.snapshot()
        assert snapshot["velocity"]["source"] == "POSITION_DIFF"
        assert snapshot["velocity"]["fresh"] is True
        assert snapshot["velocity"]["utc_time"] == "023035.00"
        assert snapshot["velocity"]["reference_frame"] == "ENU"
        assert snapshot["velocity"]["covariance_known"] is False
    finally:
        receiver.stop()
        receiver.join(timeout=2)
        os.close(master)
        os.close(slave)


def test_receiver_over_pty():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    fixes = []
    velocities = []
    receiver = Receiver(path, 115200, [], fixes.append, lambda message: None,
                        on_velocity=velocities.append)
    receiver.start()
    try:
        assert wait_until(lambda: receiver.snapshot()["serial_connected"])
        os.write(master, (GGA_FIXED + "\r\n").encode())
        assert wait_until(lambda: bool(fixes))
        snapshot = receiver.snapshot()
        assert snapshot["valid"] is True
        assert snapshot["quality"] == 4
        assert snapshot["sats"] == 28
        assert snapshot["utc_time"] == "023034.00"
        assert snapshot["fix_age_s"] is not None and snapshot["fix_age_s"] < 2.0
        assert snapshot["gga_age_s"] is not None and snapshot["gga_age_s"] < 2.0
        assert snapshot["measurement_age_s"] is not None
        assert receiver.fresh_gga() is not None

        os.write(master, (THS + "\r\n").encode())
        os.write(master, (RMC + "\r\n").encode())
        os.write(master, (VTG + "\r\n").encode())
        assert wait_until(lambda: receiver.snapshot()["heading_deg"] is not None)
        assert wait_until(lambda: bool(velocities))
        time.sleep(0.2)
        snapshot = receiver.snapshot()
        assert snapshot["heading_source"] == "THS"
        assert abs(snapshot["heading_deg"] - 90.5) < 1e-6
        assert abs(snapshot["speed_mps"] - 3.704 / 3.6) < 1e-3
        assert abs(snapshot["course_deg"] - 45.0) < 1e-6
        assert snapshot["velocity"]["source"] == "NMEA_RMC"
        assert snapshot["velocity"]["fresh"] is True
        assert snapshot["velocity"]["utc_time"] == "023034.00"
        assert snapshot["velocity"]["reference_frame"] == "ENU"
        assert snapshot["velocity"]["covariance_known"] is False
        speed = 0.514 * 0.514444
        assert abs(velocities[-1]["east_mps"] - speed * math.sin(math.radians(45.0))) < 1e-9
        assert abs(velocities[-1]["north_mps"] - speed * math.cos(math.radians(45.0))) < 1e-9

        receiver.record_fix_stamp(SimpleNamespace(sec=100, nanosec=500000000))
        receiver.record_velocity_stamp(SimpleNamespace(sec=100, nanosec=600000000))
        snapshot = receiver.snapshot()
        assert abs(snapshot["fix_stamp_s"] - 100.5) < 1e-9
        assert abs(snapshot["velocity"]["stamp_s"] - 100.6) < 1e-9

        os.write(master, (GST + "\r\n").encode())
        assert wait_until(lambda: receiver.sigma_for("023034.00") is not None)
        snapshot = receiver.snapshot()
        assert snapshot["sigma"]["longitude_sigma_m"] == 0.4
        assert snapshot["counts"]["gst"] == 1

        before = snapshot["checksum_errors"]
        os.write(master, b"$GNGGA,023036.00,3112.38608,N,12127.29727,E,4,28,0.6*00\r\n")
        assert wait_until(lambda: receiver.snapshot()["checksum_errors"] > before)
    finally:
        receiver.stop()
        receiver.join(timeout=2)
        os.close(master)
        os.close(slave)


def test_receiver_reports_missing_port_without_dying():
    receiver = Receiver("/dev/definitely-not-a-gnss-port", 115200, [], lambda info: None,
                        lambda message: None)
    receiver.start()
    try:
        assert wait_until(lambda: "open failed" in receiver.snapshot()["serial_status"])
        assert receiver.snapshot()["serial_connected"] is False
        assert receiver.is_alive()
    finally:
        receiver.stop()
        receiver.join(timeout=2)
