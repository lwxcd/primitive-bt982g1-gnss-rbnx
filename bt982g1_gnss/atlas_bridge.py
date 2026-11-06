# SPDX-License-Identifier: MulanPSL-2.0
"""Robonix primitive bridge for the Unicore BT-982G1 GNSS/RTK receiver."""
from __future__ import annotations

import json
import logging
import math
import os
from pathlib import Path
import threading
import time

from robonix_api import Err, Ok, Primitive
from .config import parse_config, resolve_port
from .ntrip import NtripClient, NtripStatus, config_from_env, load_env
from .receiver import Receiver
from .status import build_status

log = logging.getLogger(__name__)

gnss = Primitive(id=os.environ.get("RBNX_INSTANCE_NAME", "bt982g1_gnss"),
                 namespace="robonix/primitive/gnss")
config = parse_config({})
declared_topics = None
active = None


class _Active:
    def __init__(self, context, executor, node, receiver, ntrip_client,
                 ntrip_status, status_stop, status_thread):
        self.context = context
        self.executor = executor
        self.node = node
        self.receiver = receiver
        self.ntrip_client = ntrip_client
        self.ntrip_status = ntrip_status
        self.status_stop = status_stop
        self.status_thread = status_thread

    def stop(self):
        _teardown(self.receiver, self.ntrip_client, self.status_stop,
                  self.status_thread, self.executor, self.node, self.context)


def _teardown(receiver, ntrip_client, status_stop, status_thread, executor, node, context):
    if status_stop is not None:
        status_stop.set()
    if ntrip_client is not None:
        ntrip_client.join(timeout=3.0)
    if receiver is not None:
        receiver.stop()
        receiver.join(timeout=3.0)
    if status_thread is not None:
        status_thread.join(timeout=3.0)
    if executor is not None:
        try:
            executor.shutdown(timeout_sec=2.0)
        except Exception:  # noqa: BLE001
            pass
    if node is not None:
        node.destroy_node()
    if context is not None and context.ok():
        context.shutdown()


@gnss.on_init
def init(cfg):
    global config
    if active is not None:
        return Err("Deactivate before changing BT-982G1 configuration")
    try:
        parsed = parse_config(cfg)
    except ValueError as exc:
        return Err(str(exc))
    if declared_topics is not None and parsed["topics"] != declared_topics:
        return Err("Restart the provider to change already declared topic endpoints")
    config = parsed
    return Ok()


@gnss.on_activate
def activate():
    global active, declared_topics
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import String

    try:
        port = resolve_port(config["serial"])
    except ValueError as exc:
        return Err(str(exc))
    frame_id = config["frame_id"]
    context = Context()
    executor = node = receiver = ntrip_client = status_thread = None
    status_stop = threading.Event()
    ntrip_status = NtripStatus(enabled=False)
    seen = set()
    ready = False

    def publish_fix(info):
        try:
            message = NavSatFix()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = frame_id
            message.status.service = NavSatStatus.SERVICE_GPS
            message.status.status = NavSatStatus.STATUS_FIX
            message.latitude = float(info["lat"])
            message.longitude = float(info["lon"])
            message.altitude = float(info["alt"]) if info["alt"] is not None else 0.0
            message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            fix_publisher.publish(message)
        except Exception as exc:  # noqa: BLE001
            log.warning("fix publish failed: %s", exc)

    def on_fix_sample(message):
        if math.isfinite(message.latitude) and math.isfinite(message.longitude) \
                and message.status.status >= 0:
            seen.add("fix")

    def on_status_sample(message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(status, dict) or status.get("serial_connected") is not True:
            return
        age = status.get("measurement_age_s")
        if isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age):
            seen.add("status")

    def status_loop():
        interval = 1.0 / config["status_rate_hz"]
        next_time = time.monotonic()
        while not status_stop.is_set():
            message = String()
            message.data = json.dumps(build_status(receiver.snapshot(), ntrip_status.snapshot(),
                                                   frame_id),
                                      ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            status_publisher.publish(message)
            next_time += interval
            status_stop.wait(max(0.0, next_time - time.monotonic()))

    try:
        rclpy.init(context=context)
        executor = SingleThreadedExecutor(context=context)
        node = rclpy.create_node("bt982g1_gnss", context=context)
        executor.add_node(node)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        fix_publisher = node.create_publisher(NavSatFix, config["topics"]["fix"], qos)
        status_publisher = node.create_publisher(String, config["topics"]["rtk_status"], qos)
        node.create_subscription(NavSatFix, config["topics"]["fix"], on_fix_sample, qos)
        node.create_subscription(String, config["topics"]["rtk_status"], on_status_sample, qos)

        receiver = Receiver(port, config["serial"]["baud"], config["serial"]["init_commands"],
                            publish_fix, log.info)
        receiver.start()
        ntrip = config["ntrip"]
        if ntrip["enabled"]:
            env_file = Path(ntrip["env_file"]).expanduser()
            ntrip_cfg = config_from_env(load_env(env_file))
            if ntrip_cfg is None:
                log.warning("NTRIP enabled but credentials are incomplete in %s; "
                            "running without corrections", env_file)
            else:
                ntrip_status = NtripStatus(enabled=True)
                ntrip_client = NtripClient(ntrip_cfg, ntrip_status, status_stop,
                                           receiver.fresh_gga, receiver.write, log.info,
                                           ntrip["reconnect_s"], ntrip["gga_interval_s"])
                ntrip_client.start()
                log.info("ntrip enabled: %s:%s mount=%s", ntrip_cfg.host,
                         ntrip_cfg.port, ntrip_cfg.mount)
        status_thread = threading.Thread(target=status_loop, daemon=True, name="status-publisher")
        status_thread.start()

        # Only samples received after the serial link and status loop start count.
        seen.clear()
        deadline = time.monotonic() + config["sentinel_timeout_s"]
        while len(seen) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if len(seen) != 2:
            return Err("No fresh BT-982G1 GNSS fix/status samples; check the serial port, "
                       "receiver output, antenna sky view and NTRIP settings")
        for key, topic in config["topics"].items():
            if declared_topics is None:
                gnss.declare_ros2_topic(f"robonix/primitive/gnss/{key}", topic, qos="best_effort")
        declared_topics = dict(config["topics"])
        active = _Active(context, executor, node, receiver, ntrip_client,
                         ntrip_status, status_stop, status_thread)
        ready = True
        return Ok()
    except (ValueError, OSError) as exc:
        return Err(str(exc))
    finally:
        if not ready:
            _teardown(receiver, ntrip_client, status_stop, status_thread,
                      executor, node, context)


@gnss.on_deactivate
def deactivate():
    global active
    if active is not None:
        active.stop()
        active = None
    return Ok()


@gnss.on_shutdown
def shutdown():
    global active
    if active is not None:
        active.stop()
        active = None
    return Ok()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    gnss.run()
