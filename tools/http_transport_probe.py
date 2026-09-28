"""Validate HTTP page delivery or raw device receive throughput without a browser."""

from __future__ import annotations

import argparse
import concurrent.futures
import socket
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_PORT = 8848
DEFAULT_PATHS = ("/", "/app.css", "/app.js")


@dataclass(frozen=True)
class HttpResult:
    path: str
    local_port: int
    status: int
    expected_bytes: int | None
    received_bytes: int
    connect_ms: float
    first_byte_ms: float
    complete_ms: float


@dataclass(frozen=True)
class BenchmarkResult:
    local_port: int
    sent_bytes: int
    status: int
    response_body: str
    connect_ms: float
    upload_ms: float
    complete_ms: float


def receive_headers(sock: socket.socket) -> tuple[bytes, bytes]:
    data = bytearray()
    while b"\r\n\r\n" not in data:
        try:
            chunk = sock.recv(1024)
        except socket.timeout as error:
            raise TimeoutError(f"timed out waiting response headers after {len(data)} bytes") from error
        if not chunk:
            raise ConnectionError("connection closed before response headers")
        data.extend(chunk)
        if len(data) > 8192:
            raise ConnectionError("response headers exceed 8192 bytes")
    header_end = data.index(b"\r\n\r\n") + 4
    return bytes(data[:header_end]), bytes(data[header_end:])


def parse_headers(raw_headers: bytes) -> tuple[int, dict[str, str]]:
    lines = raw_headers.decode("iso-8859-1").split("\r\n")
    try:
        _, status_text, _ = lines[0].split(" ", 2)
        status = int(status_text)
    except (IndexError, ValueError) as error:
        raise ConnectionError(f"invalid HTTP status line: {lines[0]!r}") from error

    headers: dict[str, str] = {}
    for line in lines[1:]:
        if not line:
            break
        name, separator, value = line.partition(":")
        if not separator:
            raise ConnectionError(f"invalid HTTP header: {line!r}")
        headers[name.strip().lower()] = value.strip()
    return status, headers


def request_on_connection(
    sock: socket.socket, host: str, port: int, path: str, timeout: float, keep_alive: bool,
    connect_ms: float,
) -> HttpResult:
    started = time.perf_counter()
    sock.settimeout(timeout)
    connection = "keep-alive" if keep_alive else "close"
    request = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Accept: */*\r\n"
        "Cache-Control: no-cache\r\n"
        f"Connection: {connection}\r\n\r\n"
    )
    sock.sendall(request.encode("ascii"))
    headers_raw, body = receive_headers(sock)
    first_byte_ms = (time.perf_counter() - started) * 1000
    status, headers = parse_headers(headers_raw)
    expected_text = headers.get("content-length")
    expected_bytes = int(expected_text) if expected_text is not None else None
    if expected_bytes is None:
        raise ConnectionError("response has no Content-Length")
    while len(body) < expected_bytes:
        try:
            chunk = sock.recv(min(4096, expected_bytes - len(body)))
        except socket.timeout as error:
            raise TimeoutError(
                f"timed out waiting response body {len(body)}/{expected_bytes} bytes"
            ) from error
        if not chunk:
            raise ConnectionError(
                f"connection closed with incomplete body {len(body)}/{expected_bytes}"
            )
        body += chunk
    if len(body) != expected_bytes:
        raise ConnectionError(f"received excess body bytes {len(body)}/{expected_bytes}")
    return HttpResult(
        path=path,
        local_port=sock.getsockname()[1],
        status=status,
        expected_bytes=expected_bytes,
        received_bytes=len(body),
        connect_ms=connect_ms,
        first_byte_ms=first_byte_ms,
        complete_ms=(time.perf_counter() - started) * 1000,
    )


def request_once(host: str, port: int, path: str, timeout: float, keep_alive: bool) -> HttpResult:
    started = time.perf_counter()
    sock = socket.create_connection((host, port), timeout=timeout)
    connect_ms = (time.perf_counter() - started) * 1000
    try:
        return request_on_connection(sock, host, port, path, timeout, keep_alive, connect_ms)
    finally:
        sock.close()


def receive_response_body(sock: socket.socket) -> tuple[int, bytes]:
    headers_raw, body = receive_headers(sock)
    status, headers = parse_headers(headers_raw)
    expected_text = headers.get("content-length")
    if expected_text is None:
        raise ConnectionError("response has no Content-Length")
    expected_bytes = int(expected_text)
    while len(body) < expected_bytes:
        chunk = sock.recv(expected_bytes - len(body))
        if not chunk:
            raise ConnectionError(f"connection closed with incomplete response {len(body)}/{expected_bytes}")
        body += chunk
    if len(body) != expected_bytes:
        raise ConnectionError(f"received excess response bytes {len(body)}/{expected_bytes}")
    return status, body


def benchmark_receive_once(
    host: str, port: int, file_path: Path, timeout: float, progress_bytes: int,
) -> BenchmarkResult:
    file_size = file_path.stat().st_size
    if file_size <= 0:
        raise ValueError("benchmark file must not be empty")

    started = time.perf_counter()
    sock = socket.create_connection((host, port), timeout=timeout)
    connect_ms = (time.perf_counter() - started) * 1000
    try:
        sock.settimeout(timeout)
        headers = (
            "POST /api/v1/benchmark/rx HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            "Content-Type: application/octet-stream\r\n"
            f"Content-Length: {file_size}\r\n"
            "Connection: close\r\n\r\n"
        )
        sock.sendall(headers.encode("ascii"))
        sent_bytes = 0
        next_progress = progress_bytes
        with file_path.open("rb") as payload:
            while chunk := payload.read(16 * 1024):
                sock.sendall(chunk)
                sent_bytes += len(chunk)
                if progress_bytes and sent_bytes >= next_progress:
                    print(
                        f"  benchmark sent={sent_bytes}/{file_size} bytes "
                        f"elapsed_ms={(time.perf_counter() - started) * 1000:.1f}",
                        flush=True,
                    )
                    next_progress += progress_bytes
        upload_ms = (time.perf_counter() - started) * 1000
        status, body = receive_response_body(sock)
        return BenchmarkResult(
            local_port=sock.getsockname()[1],
            sent_bytes=file_size,
            status=status,
            response_body=body.decode("utf-8", errors="replace").strip(),
            connect_ms=connect_ms,
            upload_ms=upload_ms,
            complete_ms=(time.perf_counter() - started) * 1000,
        )
    finally:
        sock.close()


def print_result(cycle: int, result: HttpResult) -> None:
    print(
        f"cycle={cycle} HTTP path={result.path} local_port={result.local_port} "
        f"status={result.status} body={result.received_bytes}/{result.expected_bytes} "
        f"connect_ms={result.connect_ms:.1f} first_byte_ms={result.first_byte_ms:.1f} "
        f"complete_ms={result.complete_ms:.1f}"
    )


def print_failure(cycle: int, path: str, mode: str, error: Exception) -> None:
    print(f"cycle={cycle} HTTP path={path} mode={mode} FAILED {type(error).__name__}: {error}")


def print_benchmark_result(cycle: int, result: BenchmarkResult) -> None:
    mib_per_second = result.sent_bytes / (1024 * 1024) / (result.upload_ms / 1000)
    print(
        f"cycle={cycle} benchmark local_port={result.local_port} status={result.status} "
        f"bytes={result.sent_bytes} connect_ms={result.connect_ms:.1f} "
        f"upload_ms={result.upload_ms:.1f} throughput_mib_s={mib_per_second:.3f} "
        f"complete_ms={result.complete_ms:.1f} response={result.response_body}"
    )


def run_serial(host: str, port: int, paths: tuple[str, ...], timeout: float, cycle: int) -> None:
    for path in paths:
        try:
            print_result(cycle, request_once(host, port, path, timeout, keep_alive=False))
        except (ConnectionError, OSError, TimeoutError, ValueError) as error:
            print_failure(cycle, path, "serial", error)


def run_parallel(host: str, port: int, paths: tuple[str, ...], timeout: float, cycle: int) -> None:
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(paths)) as executor:
        futures = {
            executor.submit(request_once, host, port, path, timeout, False): path for path in paths
        }
        for future, path in futures.items():
            try:
                print_result(cycle, future.result())
            except (ConnectionError, OSError, TimeoutError, ValueError) as error:
                print_failure(cycle, path, "parallel", error)


def run_keep_alive(host: str, port: int, paths: tuple[str, ...], timeout: float, cycle: int) -> None:
    # The server currently has no pipelining requirement. Reusing one TCP
    # connection still tests whether a completed response leaves HTTPD usable.
    started = time.perf_counter()
    sock = socket.create_connection((host, port), timeout=timeout)
    connect_ms = (time.perf_counter() - started) * 1000
    try:
        for path in paths:
            try:
                print_result(
                    cycle,
                    request_on_connection(
                        sock, host, port, path, timeout, keep_alive=True, connect_ms=connect_ms
                    ),
                )
            except (ConnectionError, OSError, TimeoutError, ValueError) as error:
                print_failure(cycle, path, "keep-alive", error)
                break
    finally:
        sock.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="device IPv4 address or hostname")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--interval-ms", type=int, default=1000)
    parser.add_argument("--path", action="append", dest="paths")
    parser.add_argument(
        "--benchmark-file",
        type=Path,
        help="stream this local file to POST /api/v1/benchmark/rx; no SD-card write occurs",
    )
    parser.add_argument(
        "--progress-mib",
        type=float,
        default=1.0,
        help="benchmark client progress interval in MiB; 0 disables it (default: 1)",
    )
    parser.add_argument(
        "--mode",
        choices=("serial", "parallel", "keep-alive"),
        default="serial",
        help="one request at a time, browser-like parallel assets, or repeated keep-alive requests",
    )
    args = parser.parse_args()
    if args.cycles < 1 or args.timeout <= 0 or args.interval_ms < 0 or args.progress_mib < 0:
        parser.error("cycles and timeout must be positive; interval-ms and progress-mib cannot be negative")
    if args.benchmark_file is not None and not args.benchmark_file.is_file():
        parser.error(f"benchmark file does not exist: {args.benchmark_file}")

    if args.benchmark_file is not None:
        progress_bytes = int(args.progress_mib * 1024 * 1024)
        for cycle in range(1, args.cycles + 1):
            try:
                print_benchmark_result(
                    cycle,
                    benchmark_receive_once(
                        args.host, args.port, args.benchmark_file, args.timeout, progress_bytes,
                    ),
                )
            except (ConnectionError, OSError, TimeoutError, ValueError) as error:
                print_failure(cycle, "/api/v1/benchmark/rx", "benchmark", error)
            if cycle != args.cycles and args.interval_ms:
                time.sleep(args.interval_ms / 1000)
        return 0

    paths = tuple(args.paths) if args.paths else DEFAULT_PATHS
    runners = {
        "serial": run_serial,
        "parallel": run_parallel,
        "keep-alive": run_keep_alive,
    }
    for cycle in range(1, args.cycles + 1):
        try:
            runners[args.mode](args.host, args.port, paths, args.timeout, cycle)
        except (ConnectionError, OSError, TimeoutError, ValueError) as error:
            print(f"cycle={cycle} FAILED mode={args.mode} error={type(error).__name__}: {error}")
        if cycle != args.cycles and args.interval_ms:
            time.sleep(args.interval_ms / 1000)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
