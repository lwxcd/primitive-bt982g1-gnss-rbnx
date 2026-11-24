# SPDX-License-Identifier: MulanPSL-2.0
import json

import pytest

from bt982g1_gnss.status import build_status, position_covariance

SNAPSHOT = {
    "serial_connected": True,
    "serial_status": "open",
    "port": "/dev/rtk",
    "baud": 115200,
    "valid": True,
    "quality": 4,
    "fix_age_s": 0.123456,
    "gga_age_s": 0.123456,
    "measurement_age_s": 0.2,
    "utc_time": "023034.00",
    "fix_stamp_s": 1731000000.55,
    "velocity": {
        "source": "NMEA_RMC",
        "fresh": True,
        "age_s": 0.13,
        "utc_time": "023034.00",
        "stamp_s": 1731000000.57,
        "reference_frame": "ENU",
        "covariance_known": False,
    },
    "sigma": {
        "utc_time": "023034.00",
        "latitude_sigma_m": 0.6,
        "longitude_sigma_m": 0.4,
        "altitude_sigma_m": 0.9,
    },
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
    "counts": {"gga": 1, "rmc": 1, "vtg": 1, "gst": 1, "ths": 1, "tra": 0, "other": 0},
    "checksum_errors": 0,
}


def test_status_payload_provides_outdoor_map_fields():
    payload = build_status(SNAPSHOT, {"state": "up"}, "gnss_link")
    assert payload["receiver_quality"] == 4
    assert payload["receiver_quality_name"] == "RTK fixed"
    assert payload["state"] == "RTK_FIXED"
    assert payload["rtk_fixed"] is True
    assert payload["fix_age_s"] == 0.123
    assert payload["measurement_age_s"] == 0.2
    assert payload["valid"] is True
    assert payload["frame_id"] == "gnss_link"
    assert payload["ntrip"] == {"state": "up"}
    json.dumps(payload, allow_nan=False)


def test_status_payload_provides_localization_contract_fields():
    payload = build_status(SNAPSHOT, {"state": "up"}, "gnss_link")
    assert payload["fix_stamp_s"] == SNAPSHOT["fix_stamp_s"]
    assert payload["utc_time"] == "023034.00"
    assert payload["differential_age_s"] == 1.2
    assert payload["satellites"] == 28
    assert payload["serial_state"] == "OPEN"
    assert payload["velocity"] == {
        "source": "NMEA_RMC",
        "fresh": True,
        "age_s": 0.13,
        "utc_time": "023034.00",
        "stamp_s": SNAPSHOT["velocity"]["stamp_s"],
        "reference_frame": "ENU",
        "covariance_known": False,
    }
    assert payload["position_covariance_source"] == "NMEA_GST"
    json.dumps(payload, allow_nan=False)


@pytest.mark.parametrize("quality,connected,age,state", [
    (None, True, None, "NO_DATA"),
    (4, True, 3.0, "STALE"),
    (4, False, 0.1, "STALE"),
    (5, True, 0.1, "RTK_FLOAT"),
    (2, True, 0.1, "DGPS"),
    (0, True, 0.1, "NO_FIX"),
    (99, True, 0.1, "UNKNOWN_QUALITY"),
])
def test_status_state_gate(quality, connected, age, state):
    snapshot = {**SNAPSHOT, "quality": quality,
                "serial_connected": connected, "gga_age_s": age,
                "fix_age_s": age}
    payload = build_status(snapshot, {"state": "up"}, "gnss_link")
    assert payload["state"] == state
    expected_fixed = state == "RTK_FIXED" and snapshot["valid"]
    assert payload["rtk_fixed"] is expected_fixed


def test_status_passes_position_diff_velocity_source():
    snapshot = {**SNAPSHOT,
                "velocity": {**SNAPSHOT["velocity"], "source": "POSITION_DIFF"}}
    payload = build_status(snapshot, {"state": "up"}, "gnss_link")
    assert payload["velocity"]["source"] == "POSITION_DIFF"
    json.dumps(payload, allow_nan=False)


def test_rtk_fixed_requires_a_published_fix():
    snapshot = {**SNAPSHOT, "valid": False}
    payload = build_status(snapshot, {"state": "up"}, "gnss_link")
    assert payload["state"] == "RTK_FIXED"
    assert payload["rtk_fixed"] is False


def test_position_covariance_uses_gst_or_stays_unknown():
    assert position_covariance(SNAPSHOT["sigma"]) == pytest.approx((0.16, 0.36, 0.81))
    assert position_covariance(None) is None
    assert position_covariance({**SNAPSHOT["sigma"], "longitude_sigma_m": 0.0}) is None
    payload = build_status({**SNAPSHOT, "sigma": None}, {"state": "up"}, "gnss_link")
    assert payload["position_covariance_source"] == "UNKNOWN"


def test_status_payload_cleans_non_finite_values():
    snapshot = {**SNAPSHOT, "hdop": float("nan"), "lat": float("inf"),
                "fix_age_s": float("nan"), "gga_age_s": float("nan"),
                "fix_stamp_s": float("nan"),
                "velocity": {**SNAPSHOT["velocity"], "age_s": float("nan"),
                             "stamp_s": float("inf")}}
    payload = build_status(snapshot, {"state": "disabled"}, "gnss_link")
    assert payload["hdop"] is None
    assert payload["lat"] is None
    assert payload["fix_age_s"] is None
    assert payload["fix_stamp_s"] is None
    assert payload["velocity"]["age_s"] is None
    assert payload["velocity"]["stamp_s"] is None
    assert payload["state"] == "STALE"
    json.dumps(payload, allow_nan=False)
