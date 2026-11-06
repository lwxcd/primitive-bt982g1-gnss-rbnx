# SPDX-License-Identifier: MulanPSL-2.0
import json

from bt982g1_gnss.status import build_status

SNAPSHOT = {
    "serial_connected": True,
    "serial_status": "open",
    "port": "/dev/rtk",
    "baud": 115200,
    "valid": True,
    "quality": 4,
    "fix_age_s": 0.123456,
    "measurement_age_s": 0.2,
    "lat": 31.206434666,
    "lon": 121.4549545,
    "alt_m": 12.345,
    "sats": 28,
    "hdop": 0.6,
    "diff_age_s": 1.2,
    "speed_mps": 0.1,
    "course_deg": 45.0,
    "heading_deg": 90.5,
    "heading_pitch_deg": None,
    "heading_source": "THS",
    "counts": {"gga": 1, "rmc": 1, "vtg": 1, "ths": 1, "tra": 0, "other": 0},
    "checksum_errors": 0,
}


def test_status_payload_provides_outdoor_map_fields():
    payload = build_status(SNAPSHOT, {"state": "up"}, "gnss_link")
    assert payload["receiver_quality"] == 4
    assert payload["receiver_quality_name"] == "RTK fixed"
    assert payload["fix_age_s"] == 0.123
    assert payload["measurement_age_s"] == 0.2
    assert payload["valid"] is True
    assert payload["frame_id"] == "gnss_link"
    assert payload["ntrip"] == {"state": "up"}
    json.dumps(payload, allow_nan=False)


def test_status_payload_cleans_non_finite_values():
    snapshot = {**SNAPSHOT, "hdop": float("nan"), "lat": float("inf"), "fix_age_s": float("nan")}
    payload = build_status(snapshot, {"state": "disabled"}, "gnss_link")
    assert payload["hdop"] is None
    assert payload["lat"] is None
    assert payload["fix_age_s"] is None
    json.dumps(payload, allow_nan=False)
