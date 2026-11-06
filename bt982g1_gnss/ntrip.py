# SPDX-License-Identifier: MulanPSL-2.0
"""NTRIP client for the BT-982G1/UM982 rover.

Adapted from robot-deep_robotics-lite3/scripts/rtk_imu/ntrip_client.py.
Credentials come from an env file, never from the topic or the command line.
"""
from __future__ import annotations

import base64
import re
import socket
import ssl
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


class NtripError(Exception):
    pass


class AuthError(NtripError):
    pass


@dataclass
class Proxy:
    scheme: str
    host: str
    port: int
    user: str = ""
    password: str = ""


@dataclass
class Config:
    host: str
    port: int
    mount: str
    user: str
    password: str
    version: int
    tls: bool
    host_header: str
    proxy: Optional[Proxy]


def load_env(path: Path) -> dict:
    env = {}
    if not path.exists():
        return env
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.strip()
        if " #" in value:
            value = value.split(" #", 1)[0].rstrip()
        env[key.strip().upper()] = value.strip('"').strip("'")
    return env


def config_from_env(env: dict) -> Optional[Config]:
    password = env.get("NTRIP_PASS", "")
    pass_file = env.get("NTRIP_PASS_FILE", "")
    if pass_file:
        try:
            password = Path(pass_file).expanduser().read_text(encoding="utf-8").strip()
        except OSError:
            return None
    try:
        port = int(env.get("NTRIP_PORT", 2101))
        version = int(env.get("NTRIP_VERSION", 2))
        tls = bool(int(env.get("NTRIP_TLS", 0)))
        proxy = parse_proxy(env.get("NTRIP_PROXY", ""))
    except (ValueError, TypeError):
        return None
    host = env.get("NTRIP_HOST", "").strip()
    mount = env.get("NTRIP_MOUNT", "").strip()
    user = env.get("NTRIP_USER", "").strip()
    if not (host and mount and user and password):
        return None
    return Config(host=host, port=port, mount=mount, user=user, password=password,
                  version=version, tls=tls,
                  host_header=env.get("NTRIP_HOST_HEADER", ""), proxy=proxy)


def parse_proxy(text: str) -> Optional[Proxy]:
    if not text:
        return None
    match = re.match(r"^(socks5h?|http)://(?:([^:@/]+)(?::([^@/]+))?@)?([^:/]+):(\d+)/?$", text)
    if not match:
        raise ValueError(f"bad proxy URL: {text!r} (want socks5://host:port or http://host:port)")
    return Proxy(scheme=match.group(1), host=match.group(4), port=int(match.group(5)),
                 user=match.group(2) or "", password=match.group(3) or "")


class NtripStatus:
    """Thread-safe observable state for the status JSON and consumers."""

    def __init__(self, enabled: bool) -> None:
        self.lock = threading.Lock()
        self.state = "down" if enabled else "disabled"
        self.connects = 0
        self.rtcm_bytes = 0
        self.rtcm_dropped = 0
        self.gga_uploaded = 0
        self.last_error = ""

    def set_state(self, state: str, error: str = "") -> None:
        with self.lock:
            self.state = state
            self.last_error = error

    def connected(self) -> None:
        with self.lock:
            self.state = "up"
            self.connects += 1
            self.last_error = ""

    def add_rtcm(self, size: int, written: bool) -> None:
        with self.lock:
            if written:
                self.rtcm_bytes += size
            else:
                self.rtcm_dropped += size

    def add_gga(self) -> None:
        with self.lock:
            self.gga_uploaded += 1

    def snapshot(self) -> dict:
        with self.lock:
            return {"state": self.state, "connects": self.connects,
                    "rtcm_bytes": self.rtcm_bytes, "rtcm_dropped": self.rtcm_dropped,
                    "gga_uploaded": self.gga_uploaded, "last_error": self.last_error}


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("socket closed")
        data += chunk
    return data


def _read_headers(sock: socket.socket, limit: int = 32768) -> tuple[bytes, bytes]:
    data = bytearray()
    while b"\r\n\r\n" not in data:
        chunk = sock.recv(1024)
        if not chunk:
            raise ConnectionError("connection closed before NTRIP headers completed")
        data.extend(chunk)
        if len(data) > limit:
            raise NtripError("NTRIP header too large")
    head, _, rest = bytes(data).partition(b"\r\n\r\n")
    return head, rest


def _parse_header_lines(head: bytes) -> dict:
    headers = {}
    for line in head.decode("latin-1", "replace").split("\r\n")[1:]:
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key.strip().lower()] = value.strip()
    return headers


def _socks5_handshake(sock: socket.socket, host: str, port: int, proxy: Proxy) -> None:
    methods = [0x00]
    if proxy.user:
        methods.append(0x02)
    sock.sendall(bytes([5, len(methods), *methods]))
    reply = _recv_exact(sock, 2)
    if reply[0] != 5 or reply[1] == 0xFF:
        raise NtripError("socks5: no acceptable auth method")
    if reply[1] == 0x02:
        user, password = proxy.user.encode(), proxy.password.encode()
        sock.sendall(bytes([1, len(user)]) + user + bytes([len(password)]) + password)
        if _recv_exact(sock, 2)[1] != 0:
            raise AuthError("socks5: authentication failed")
    name = host.encode()
    sock.sendall(bytes([5, 1, 0, 3, len(name)]) + name + port.to_bytes(2, "big"))
    reply = _recv_exact(sock, 4)
    if reply[0] != 5 or reply[1] != 0:
        raise NtripError(f"socks5: connect failed (code {reply[1]})")
    if reply[3] == 1:
        _recv_exact(sock, 4)
    elif reply[3] == 3:
        _recv_exact(sock, _recv_exact(sock, 1)[0])
    elif reply[3] == 4:
        _recv_exact(sock, 16)
    else:
        raise NtripError(f"socks5: bad address type {reply[3]}")
    _recv_exact(sock, 2)


def _http_connect(sock: socket.socket, host: str, port: int, proxy: Proxy) -> None:
    request = f"CONNECT {host}:{port} HTTP/1.1\r\nHost: {host}:{port}\r\n"
    if proxy.user:
        token = base64.b64encode(f"{proxy.user}:{proxy.password}".encode()).decode()
        request += f"Proxy-Authorization: Basic {token}\r\n"
    request += "\r\n"
    sock.sendall(request.encode())
    head, _ = _read_headers(sock)
    status = head.split(b"\r\n", 1)[0].decode("latin-1", "replace")
    parts = status.split()
    if len(parts) < 2 or parts[1] != "200":
        raise NtripError(f"proxy CONNECT failed: {status}")


def open_socket(cfg: Config, timeout: float = 10.0) -> socket.socket:
    if cfg.proxy:
        sock = socket.create_connection((cfg.proxy.host, cfg.proxy.port), timeout=timeout)
        try:
            if cfg.proxy.scheme.startswith("socks5"):
                _socks5_handshake(sock, cfg.host, cfg.port, cfg.proxy)
            else:
                _http_connect(sock, cfg.host, cfg.port, cfg.proxy)
        except Exception:
            sock.close()
            raise
    else:
        sock = socket.create_connection((cfg.host, cfg.port), timeout=timeout)
    sock.settimeout(timeout)
    if cfg.tls:
        context = ssl.create_default_context()
        sock = context.wrap_socket(sock, server_hostname=cfg.host)
        sock.settimeout(timeout)
    return sock


def request_bytes(cfg: Config, path: str) -> bytes:
    host_header = cfg.host_header or f"{cfg.host}:{cfg.port}"
    auth = base64.b64encode(f"{cfg.user}:{cfg.password}".encode()).decode()
    if cfg.version == 2:
        text = (f"GET {path} HTTP/1.1\r\n"
                f"Host: {host_header}\r\n"
                "Ntrip-Version: Ntrip/2.0\r\n"
                "User-Agent: NTRIP robonix-bt982g1/1.0\r\n"
                "Accept: */*\r\n"
                f"Authorization: Basic {auth}\r\n"
                "\r\n")
    else:
        text = (f"GET {path} HTTP/1.0\r\n"
                "User-Agent: NTRIP robonix-bt982g1/1.0\r\n"
                "Accept: */*\r\n"
                f"Authorization: Basic {auth}\r\n"
                "\r\n")
    return text.encode()


class ChunkedDecoder:
    """Incremental HTTP/1.1 chunked-transfer decoder for the correction stream."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.state = "size"
        self.remaining = 0

    def feed(self, data: bytes) -> bytes:
        self.buf.extend(data)
        out = bytearray()
        while True:
            if self.state == "size":
                idx = self.buf.find(b"\r\n")
                if idx < 0:
                    break
                line = bytes(self.buf[:idx])
                del self.buf[:idx + 2]
                if not line:
                    continue
                try:
                    size = int(line.split(b";", 1)[0], 16)
                except ValueError:
                    self.state = "done"
                    break
                if size == 0:
                    self.state = "trailer"
                else:
                    self.remaining = size
                    self.state = "data"
            elif self.state == "data":
                if not self.buf:
                    break
                take = min(self.remaining, len(self.buf))
                out += self.buf[:take]
                del self.buf[:take]
                self.remaining -= take
                if self.remaining == 0:
                    self.state = "crlf"
            elif self.state == "crlf":
                if len(self.buf) < 2:
                    break
                del self.buf[:2]
                self.state = "size"
            elif self.state == "trailer":
                idx = self.buf.find(b"\r\n")
                if idx < 0:
                    break
                line = bytes(self.buf[:idx])
                del self.buf[:idx + 2]
                if not line:
                    self.state = "done"
            else:
                break
        return bytes(out)


class NtripClient(threading.Thread):
    def __init__(self, cfg: Config, status: NtripStatus, stop: threading.Event,
                 get_gga, write, log, reconnect_s: float, gga_interval_s: float) -> None:
        super().__init__(daemon=True, name="ntrip-client")
        self.cfg = cfg
        self.status = status
        self.stop = stop
        self.get_gga = get_gga
        self.write = write
        self.log = log
        self.reconnect_s = reconnect_s
        self.gga_interval_s = gga_interval_s

    def run(self) -> None:
        while not self.stop.is_set():
            try:
                self.session()
            except AuthError as exc:
                self.status.set_state("auth-error", str(exc))
                self.log(f"ntrip: {exc}")
                self.stop.wait(30.0)
            except Exception as exc:  # noqa: BLE001
                self.status.set_state("down", str(exc))
                self.log(f"ntrip: {exc}")
                self.stop.wait(self.reconnect_s)

    def session(self) -> None:
        cfg = self.cfg
        where = f"{cfg.host}:{cfg.port} mount={cfg.mount} v{cfg.version} tls={int(cfg.tls)}"
        if cfg.proxy:
            where += f" proxy={cfg.proxy.scheme}://{cfg.proxy.host}:{cfg.proxy.port}"
        self.log(f"ntrip: connecting {where}")
        sock = open_socket(cfg)
        try:
            sock.sendall(request_bytes(cfg, "/" + cfg.mount.lstrip("/")))
            head, rest = _read_headers(sock)
            status_line = head.split(b"\r\n", 1)[0].decode("latin-1", "replace")
            self.log(f"ntrip: caster answered {status_line!r}")
            parts = status_line.split()
            code = parts[1] if len(parts) >= 2 else ""
            if status_line.upper().startswith("SOURCETABLE"):
                raise NtripError("caster returned a sourcetable; mountpoint unusable")
            if code in ("401", "403"):
                raise AuthError(f"authentication rejected ({code})")
            if code != "200":
                raise NtripError(f"caster refused stream: {status_line}")
            chunked = "chunked" in _parse_header_lines(head).get("transfer-encoding", "").lower()
            self.status.connected()
            decoder = ChunkedDecoder() if chunked else None
            sock.settimeout(0.5)
            next_gga = 0.0
            warned = 0.0
            if rest:
                payload = decoder.feed(rest) if decoder else rest
                if payload:
                    self.status.add_rtcm(len(payload), self.write(payload))
            while not self.stop.is_set():
                now = time.monotonic()
                if now >= next_gga:
                    gga = self.get_gga()
                    if gga:
                        sock.sendall(gga)
                        self.status.add_gga()
                    elif now - warned >= 30.0:
                        warned = now
                        self.log("ntrip: no valid GGA for uplink; VRS casters may refuse corrections")
                    next_gga = now + self.gga_interval_s
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise ConnectionError("caster closed the stream")
                payload = decoder.feed(data) if decoder else data
                if payload:
                    self.status.add_rtcm(len(payload), self.write(payload))
        finally:
            try:
                sock.close()
            except OSError:
                pass
            with self.status.lock:
                if self.status.state == "up":
                    self.status.state = "down"
