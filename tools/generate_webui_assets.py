from pathlib import Path
import struct
import zlib


ROOT = Path(__file__).resolve().parents[1]
HTTPD = ROOT / "httpd"


def read_literal(path: Path, opening: str, closing: str) -> bytes:
    source = path.read_text(encoding="utf-8")
    start = source.index(opening) + len(opening)
    end = source.index(closing, start)
    return source[start:end].encode("utf-8")


def gzip_bytes(payload: bytes) -> bytes:
    compressor = zlib.compressobj(level=9, method=zlib.DEFLATED, wbits=-15)
    body = compressor.compress(payload) + compressor.flush()
    trailer = struct.pack("<II", zlib.crc32(payload) & 0xFFFFFFFF, len(payload))
    return b"\x1f\x8b\x08\x00\x00\x00\x00\x00\x02\xff" + body + trailer


def format_array(payload: bytes) -> str:
    rows = []
    for offset in range(0, len(payload), 12):
        row = ", ".join(f"0x{value:02x}" for value in payload[offset : offset + 12])
        rows.append(f"    {row},")
    return "\n".join(rows)


assets = [
    ("Index", "webui_page.hpp", 'R"HTML(', ')HTML";'),
    ("Style", "webui_style.hpp", 'R"CSS(', ')CSS";'),
    ("Script", "webui_script.hpp", 'R"JS(', ')JS";'),
]

output = [
    "#pragma once",
    "",
    "#include <cstddef>",
    "#include <cstdint>",
    "",
    "namespace laserctl::httpd {",
    "",
    "// Generated from webui_page.hpp, webui_style.hpp and webui_script.hpp.",
    "",
]

for name, filename, opening, closing in assets:
    compressed = gzip_bytes(read_literal(HTTPD / filename, opening, closing))
    output.append(f"constexpr std::uint8_t kWebUi{name}Gzip[] = {{")
    output.append(format_array(compressed))
    output.append("};")
    output.append(f"constexpr std::size_t kWebUi{name}GzipSize = sizeof(kWebUi{name}Gzip);")
    output.append("")

output.append("}  // namespace laserctl::httpd")
output.append("")
(HTTPD / "webui_assets_gzip.hpp").write_text("\n".join(output), encoding="utf-8")
