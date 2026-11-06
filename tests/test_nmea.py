# SPDX-License-Identifier: MulanPSL-2.0
import math

from bt982g1_gnss import nmea

GGA_FIXED = "$GNGGA,023034.00,3112.38608,N,12127.29727,E,4,28,0.6,12.345,M,-8.0,M,1.2,0000*7E"
GGA_SINGLE = "$GNGGA,023035.00,3112.38608,N,12127.29727,E,1,18,1.1,12.345,M,-8.0,M,,*52"
RMC = "$GNRMC,023034.00,A,3112.38608,N,12127.29727,E,0.514,45.0,061126,,,A*7F"
VTG = "$GNVTG,45.0,T,,M,1.852,N,3.704,K,A*2C"
THS = "$GPTHS,90.5,A*0B"
TRA = "$GPTRA,91.2,1.5,0.2,*42"


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
    assert math.isclose(info["lat"], 31 + 12.38608 / 60, abs_tol=1e-9)
    assert math.isclose(info["lon"], 121 + 27.29727 / 60, abs_tol=1e-9)


def test_parse_gga_empty_diff_age():
    info = nmea.parse_gga(GGA_SINGLE)
    assert info["quality"] == 1
    assert info["diff_age"] is None


def test_parse_gga_rejects_other_sentence():
    assert nmea.parse_gga(RMC) is None
    assert nmea.parse_gga("garbage") is None


def test_parse_rmc_speed_and_course():
    info = nmea.parse_rmc(RMC)
    assert info["status"] == "A"
    assert math.isclose(info["speed_mps"], 0.514 * 0.514444, abs_tol=1e-9)
    assert info["course_deg"] == 45.0


def test_parse_rmc_invalid_status_clears_velocity():
    line = RMC.replace(",A,", ",V,")
    checksum = 0
    body = line[1:].rpartition("*")[0]
    for char in body:
        checksum ^= ord(char)
    info = nmea.parse_rmc(f"${body}*{checksum:02X}")
    assert info == {"status": "V", "speed_mps": None, "course_deg": None}


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
