# SPDX-License-Identifier: MulanPSL-2.0
import os
import pty
import time

from bt982g1_gnss.receiver import Receiver

GGA_FIXED = "$GNGGA,023034.00,3112.38608,N,12127.29727,E,4,28,0.6,12.345,M,-8.0,M,1.2,0000*7E"
RMC = "$GNRMC,023034.00,A,3112.38608,N,12127.29727,E,0.514,45.0,061126,,,A*7F"
VTG = "$GNVTG,45.0,T,,M,1.852,N,3.704,K,A*2C"
THS = "$GPTHS,90.5,A*0B"


def wait_until(predicate, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_receiver_over_pty():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    fixes = []
    receiver = Receiver(path, 115200, [], fixes.append, lambda message: None)
    receiver.start()
    try:
        assert wait_until(lambda: receiver.snapshot()["serial_connected"])
        os.write(master, (GGA_FIXED + "\r\n").encode())
        assert wait_until(lambda: bool(fixes))
        snapshot = receiver.snapshot()
        assert snapshot["valid"] is True
        assert snapshot["quality"] == 4
        assert snapshot["sats"] == 28
        assert snapshot["fix_age_s"] is not None and snapshot["fix_age_s"] < 2.0
        assert snapshot["measurement_age_s"] is not None
        assert receiver.fresh_gga() is not None

        os.write(master, (THS + "\r\n").encode())
        os.write(master, (RMC + "\r\n").encode())
        os.write(master, (VTG + "\r\n").encode())
        assert wait_until(lambda: receiver.snapshot()["heading_deg"] is not None)
        time.sleep(0.2)
        snapshot = receiver.snapshot()
        assert snapshot["heading_source"] == "THS"
        assert abs(snapshot["heading_deg"] - 90.5) < 1e-6
        assert abs(snapshot["speed_mps"] - 3.704 / 3.6) < 1e-3
        assert abs(snapshot["course_deg"] - 45.0) < 1e-6

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
