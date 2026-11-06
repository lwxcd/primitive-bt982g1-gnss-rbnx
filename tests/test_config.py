# SPDX-License-Identifier: MulanPSL-2.0
import pytest

from bt982g1_gnss import config as cfg
from bt982g1_gnss import ntrip


def test_defaults_match_outdoor_mapping_dependency():
    parsed = cfg.parse_config({})
    assert parsed["topics"] == {"fix": "/gps/fix", "rtk_status": "/gps/rtk_status"}
    assert parsed["serial"]["port"] == "auto"
    assert parsed["serial"]["baud"] == 115200
    assert parsed["serial"]["init_commands"] == ["MODE ROVER"]
    assert parsed["ntrip"]["enabled"] is True
    assert parsed["frame_id"] == "gnss_link"
    assert parsed["status_rate_hz"] == 1.0
    assert parsed["sentinel_timeout_s"] == 30.0


def test_explicit_port_and_topics():
    parsed = cfg.parse_config({
        "serial": {"port": "/dev/rtk", "baud": 460800},
        "topics": {"fix": "/rtk/fix"},
    })
    assert parsed["serial"]["port"] == "/dev/rtk"
    assert parsed["topics"] == {"fix": "/rtk/fix", "rtk_status": "/gps/rtk_status"}


@pytest.mark.parametrize("payload", [
    {"status_rate_hz": 0},
    {"status_rate_hz": 21},
    {"sentinel_timeout_s": 0},
    {"frame_id": "gnss link"},
    {"topics": {"fix": "relative"}},
    {"topics": {"fix": "/gps/rtk_status"}},
    {"serial": {"port": "rtk"}},
    {"serial": {"usb_vid": 0x067B}},
    {"serial": {"init_commands": ["MODE\nROVER"]}},
    {"serial": {"init_commands": "MODE ROVER"}},
    {"ntrip": {"enabled": "yes"}},
    {"ntrip": {"reconnect_s": 0}},
    {"unknown": 1},
])
def test_invalid_config_rejected(payload):
    with pytest.raises(ValueError):
        cfg.parse_config(payload)


def test_resolve_explicit_port_does_not_probe_hardware():
    assert cfg.resolve_port({"port": "/dev/rtk"}) == "/dev/rtk"


def test_ntrip_config_requires_complete_credentials():
    assert ntrip.config_from_env({}) is None
    env = {"NTRIP_HOST": "caster", "NTRIP_MOUNT": "MOUNT", "NTRIP_USER": "user"}
    assert ntrip.config_from_env(env) is None
    env["NTRIP_PASS"] = "secret"
    parsed = ntrip.config_from_env(env)
    assert parsed.host == "caster"
    assert parsed.port == 2101
    assert parsed.version == 2
    assert parsed.proxy is None


def test_ntrip_config_password_file(tmp_path):
    secret = tmp_path / "pass"
    secret.write_text("from-file\n", encoding="utf-8")
    env = {"NTRIP_HOST": "caster", "NTRIP_MOUNT": "MOUNT", "NTRIP_USER": "user",
           "NTRIP_PASS_FILE": str(secret), "NTRIP_PROXY": "socks5://127.0.0.5:10808"}
    parsed = ntrip.config_from_env(env)
    assert parsed.password == "from-file"
    assert parsed.proxy.scheme == "socks5"
    assert parsed.proxy.port == 10808


def test_ntrip_config_rejects_bad_proxy():
    env = {"NTRIP_HOST": "caster", "NTRIP_MOUNT": "MOUNT", "NTRIP_USER": "user",
           "NTRIP_PASS": "secret", "NTRIP_PROXY": "not-a-url"}
    assert ntrip.config_from_env(env) is None
