# SPDX-License-Identifier: MulanPSL-2.0
import math

from bt982g1_gnss import nmea

GGA_FIXED = "$GNGGA,023034.00,3112.38608,N,12127.29727,E,4,28,0.6,12.345,M,-8.0,M,1.2,0000*7E"
GGA_SINGLE = "$GNGGA,023035.00,3112.38608,N,12127.29727,E,1,18,1.1,12.345,M,-8.0,M,,*52"
RMC = "$GNRMC,023034.00,A,3112.38608,N,12127.29727,E,0.514,45.0,061126,,,A*7F"
VTG = "$GNVTG,45.0,T,,M,1.852,N,3.704,K,A*2C"
THS = "$GPTHS,90.5,A*0B"
TRA = "$GPTRA,91.2,1.5,0.2,*42"
GST = "$GNGST,023034.00,0.8,0.5,0.3,32.6,0.6,0.4,0.9*7D"


def test_checksum_accepts_valid_and_rejects_corrupt():
    assert nmea.checksum_ok(GGA_FIXED)
    assert not nmea.checksum_ok(GGA_FIXED[:-2] + "00")
    assert not nmea.checksum_ok("")
    assert not nmea.checksum_ok("$GNGGA,no-checksum")


def test_parse_gga_rtk_fixed():
    info = nmea.parse_gga(GGA_FIXED)
    assert info["quality"] == 4
    assert info["sats"] == 28
    assert info["hdop"] == 0.6
    assert info["diff_age"] == 1.2
    assert info["alt"] == 12.345
    assert info["utc_time"] == "023034.00"
    assert info["epoch_s"] == 2 * 3600 + 30 * 60 + 34.0
    assert math.isclose(info["lat"], 31 + 12.38608 / 60, abs_tol=1e-9)
    assert math.isclose(info["lon"], 121 + 27.29727 / 60, abs_tol=1e-9)


def test_parse_gga_empty_diff_age():
    info = nmea.parse_gga(GGA_SINGLE)
    assert info["quality"] == 1
    assert info["diff_age"] is None


def test_parse_gga_rejects_other_sentence():
    assert nmea.parse_gga(RMC) is None
    assert nmea.parse_gga("garbage") is None


def test_parse_rmc_speed_course_and_enu():
    info = nmea.parse_rmc(RMC)
    assert info["status"] == "A"
    assert info["valid"] is True
    assert info["utc_time"] == "023034.00"
    speed = 0.514 * 0.514444
    assert math.isclose(info["speed_mps"], speed, abs_tol=1e-9)
    assert info["course_deg"] == 45.0
    assert math.isclose(info["east_mps"], speed * math.sin(math.radians(45.0)), abs_tol=1e-9)
    assert math.isclose(info["north_mps"], speed * math.cos(math.radians(45.0)), abs_tol=1e-9)


def test_parse_rmc_invalid_status_clears_velocity():
    line = RMC.replace(",A,", ",V,")
    checksum = 0
    body = line[1:].rpartition("*")[0]
    for char in body:
        checksum ^= ord(char)
    info = nmea.parse_rmc(f"${body}*{checksum:02X}")
    assert info["status"] == "V"
    assert info["valid"] is False
    assert info["speed_mps"] is None
    assert info["course_deg"] is None
    assert info["east_mps"] is None


def test_parse_rmc_no_fix_mode_is_invalid():
    body = "GNRMC,023034.00,A,3112.38608,N,12127.29727,E,0.514,45.0,061126,,,N"
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    info = nmea.parse_rmc(f"${body}*{checksum:02X}")
    assert info["valid"] is False


def test_parse_gst_sigmas():
    info = nmea.parse_gst(GST)
    assert info["utc_time"] == "023034.00"
    assert info["latitude_sigma_m"] == 0.6
    assert info["longitude_sigma_m"] == 0.4
    assert info["altitude_sigma_m"] == 0.9


def test_parse_gst_rejects_zero_and_other_sentences():
    body = "GNGST,023034.00,0.8,0.5,0.3,32.6,0.0,0.4,0.9"
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    assert nmea.parse_gst(f"${body}*{checksum:02X}") is None
    assert nmea.parse_gst(GGA_FIXED) is None


def test_utc_seconds_validation():
    assert nmea.utc_seconds("023034.00") == 2 * 3600 + 30 * 60 + 34.0
    assert nmea.utc_seconds("") is None
    assert nmea.utc_seconds("246000.00") is None
    assert nmea.utc_seconds("026000.00") is None
    assert nmea.utc_seconds("023460.00") == 2 * 3600 + 34 * 60 + 60.0


def test_parse_vtg_speed():
    info = nmea.parse_vtg(VTG)
    assert math.isclose(info["speed_mps"], 3.704 / 3.6, abs_tol=1e-9)
    assert info["course_deg"] == 45.0


def test_parse_heading_ths_and_tra():
    ths = nmea.parse_heading(THS)
    assert ths == {"source": "THS", "heading_deg": 90.5, "pitch_deg": None}
    tra = nmea.parse_heading(TRA)
    assert tra == {"source": "TRA", "heading_deg": 91.2, "pitch_deg": 1.5}


def test_parse_heading_uses_talker_independent_type():
    assert nmea.sentence_kind("$GNTHS,1.0,A*00") == "THS"
    assert nmea.sentence_kind("$GPGGA,1,2,3*00") == "GGA"
