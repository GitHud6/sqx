"""Probe the device portal without relying on browser behavior."""

from __future__ import annotations

import argparse
import base64
import hashlib
import http.client
import os
import socket
import struct
import time
from concurrent.futures import ThreadPoolExecutor


HTTP_PORT = 8848
WEBSOCKET_PORT = 8849
HTTP_TIMEOUT = 15.0
WEBSOCKET_TIMEOUT = 5.0


def log(cycle: int, message: str) -> None:
    print(f"cycle={cycle} {message}")


def http_get(host: str, path: str, cycle: int) -> None:
    started = time.perf_counter()
    connection = http.client.HTTPConnection(host, HTTP_PORT, timeout=HTTP_TIMEOUT)
    try:
        connection.request("GET", path, headers={"Cache-Control": "no-cache"})
        response = connection.getresponse()
        body = response.read()
        elapsed = (time.perf_counter() - started) * 1000
        log(cycle, f"HTTP {response.status} {path} bytes={len(body)} time_ms={elapsed:.1f}")
    finally:
        connection.close()


def read_until_headers(sock: socket.socket) -> bytes:
    data = bytearray()
    while b"\r\n\r\n" not in data:
        chunk = sock.recv(1024)
        if not chunk:
            raise ConnectionError("WebSocket closed during handshake")
        data.extend(chunk)
        if len(data) > 8192:
            raise ConnectionError("WebSocket handshake headers are too large")
    return bytes(data)


def send_websocket_text(sock: socket.socket, text: str) -> None:
    payload = text.encode("utf-8")
    mask = os.urandom(4)
    masked = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
    length = len(masked)
    if length < 126:
        header = bytes((0x81, 0x80 | length))
    elif length <= 0xFFFF:
        header = bytes((0x81, 0x80 | 126)) + struct.pack("!H", length)
    else:
        header = bytes((0x81, 0x80 | 127)) + struct.pack("!Q", length)
    sock.sendall(header + mask + masked)


def receive_websocket_frame(sock: socket.socket) -> tuple[int, bytes]:
    header = sock.recv(2)
    if len(header) != 2:
        raise ConnectionError("WebSocket closed while reading frame header")
    first, second = header
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", sock.recv(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", sock.recv(8))[0]
    masked = second & 0x80
    mask = sock.recv(4) if masked else b""
    payload = bytearray()
    while len(payload) < length:
        chunk = sock.recv(min(4096, length - len(payload)))
        if not chunk:
            raise ConnectionError("WebSocket closed while reading frame payload")
        payload.extend(chunk)
    if masked:
        payload = bytearray(value ^ mask[index % 4] for index, value in enumerate(payload))
    return first & 0x0F, bytes(payload)


def websocket_probe(host: str, cycle: int) -> None:
    started = time.perf_counter()
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    expected_accept = base64.b64encode(
        hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
    ).decode("ascii")
    sock = socket.create_connection((host, WEBSOCKET_PORT), timeout=WEBSOCKET_TIMEOUT)
    try:
        request = (
            "GET / HTTP/1.1\r\n"
            f"Host: {host}:{WEBSOCKET_PORT}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(request.encode("ascii"))
        response = read_until_headers(sock).decode("ascii", "replace")
        if " 101 " not in response or expected_accept not in response:
            raise ConnectionError(f"invalid handshake: {response.splitlines()[0]}")
        send_websocket_text(sock, "$POS/STATUS")
        sock.settimeout(2.0)
        frames = 0
        text_bytes = 0
        deadline = time.perf_counter() + 2.0
        while time.perf_counter() < deadline:
            try:
                opcode, payload = receive_websocket_frame(sock)
            except socket.timeout:
                break
            frames += 1
            text_bytes += len(payload)
            if opcode == 0x8:
                break
        elapsed = (time.perf_counter() - started) * 1000
        log(cycle, f"WS connected frames={frames} text_bytes={text_bytes} time_ms={elapsed:.1f}")
    finally:
        try:
            sock.close()
        except OSError:
            pass


def run_cycle(host: str, cycle: int) -> None:
    for path in ("/", "/app.css", "/app.js"):
        http_get(host, path, cycle)

    # The page starts the WebSocket and the first storage refresh together.
    with ThreadPoolExecutor(max_workers=2) as executor:
        websocket_future = executor.submit(websocket_probe, host, cycle)
        storage_future = executor.submit(
            http_get, host, "/api/v1/storage/list?path=%2F", cycle
        )
        websocket_future.result()
        storage_future.result()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="device IPv4 address or hostname")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument(
        "--mode",
        choices=("page", "http-only", "ws-only"),
        default="page",
        help="page startup sequence or one transport in isolation",
    )
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error("--cycles must be positive")
    for cycle in range(1, args.cycles + 1):
        try:
            if args.mode == "page":
                run_cycle(args.host, cycle)
            elif args.mode == "http-only":
                for path in ("/", "/app.css", "/app.js", "/api/v1/storage/list?path=%2F"):
                    http_get(args.host, path, cycle)
            else:
                websocket_probe(args.host, cycle)
        except Exception as error:  # noqa: BLE001 - probe must report all failures
            log(cycle, f"FAILED {type(error).__name__}: {error}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
